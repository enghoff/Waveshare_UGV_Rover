"""Built TensorRT engines, and the one class that runs one.

The perception models reach this board's GPU through TensorRT rather than
through onnxruntime, and the reason is worth stating once because it is not
obvious and it cost a day to establish. onnxruntime has no build for JetPack 7:
the community Jetson wheel index stops at JetPack 6, and the official aarch64
wheel on PyPI carries compiled kernels for sm_70 through sm_121 -- every
architecture except the Orin's own sm_87 -- so it finds the GPU, builds a
session on it, and then fails at the first kernel launch with "no kernel image
is available for execution on the device". Nothing about installing CUDA fixes
that, because the missing piece is inside the wheel. TensorRT, which NVIDIA
builds for this board, has no such gap.

**An engine is not a model file.** It is compiled for this GPU, this TensorRT
version and the batch sizes it was given, it takes minutes to build, and it is
useless anywhere else -- which is why `install_perception.sh` builds them on the
rover and nothing here is in the repository. If TensorRT refuses to load one it
says so plainly, and `perceive` falls back to the CPU.

**Precision is a per-model decision, and fp16 is not safe by default.** SigLIP2
in genuine fp16 collapses: measured on the rover, all fifty-seven vocabulary
vectors came back within 0.92 of each other, so a single phrase won every region
in the frame. This is invisible under onnxruntime, which has no fp16 kernels and
quietly computes such graphs in fp32. The build script therefore asks for fp16
only where it has been checked against a full-precision reference.
"""
from __future__ import annotations

import gc
import mmap
import os

#: What `install_perception.sh` names the engines it builds. Batch sizes are not
#: in the names: an engine carries its own optimisation profile and the runtime
#: reads the allowed range out of it.
FASTSAM = "fastsam.plan"
DINO = "dinov2.plan"
SIGLIP_VISION = "siglip-vision.plan"
SIGLIP_TEXT = "siglip-text.plan"

#: Every engine a look needs to be able to run. The text engine is in the list
#: because the vocabulary has to be embedded before the first look can be named,
#: even though it is loaded once and then let go.
REQUIRED = (FASTSAM, DINO, SIGLIP_VISION, SIGLIP_TEXT)


class NoEngines(RuntimeError):
    """No usable GPU path on this host.

    Its own type so that `perceive` can tell "this board has no engines" from
    "this engine is broken": the first is an ordinary state that falls back to
    the CPU, and the second is worth saying out loud.
    """


def _imports(vendor: str):
    """TensorRT and the CUDA bindings, or a sentence saying why not.

    TensorRT comes from JetPack and is installed system-wide; the CUDA bindings
    are a 6 MB wheel unpacked into `vendor/` beside the models, because this
    board's Python is externally managed and no deploy script has a sudo
    password.
    """
    import sys
    if os.path.isdir(vendor) and vendor not in sys.path:
        sys.path.append(vendor)
    try:
        import tensorrt
        from cuda.bindings import runtime
    except ImportError as error:
        raise NoEngines(f"no TensorRT path on this host: {error}") from error
    return tensorrt, runtime


def available(directory: str) -> tuple[bool, str]:
    """(ready, why not), loading nothing.

    Cheap on purpose, because the sidecar's health endpoint answers with it and
    a health check that loads half a gigabyte is not a health check.
    """
    try:
        _imports(directory)
    except NoEngines as error:
        return False, str(error)
    missing = [name for name in REQUIRED
               if not os.path.isfile(os.path.join(directory, name))]
    if missing:
        return False, (f"no built engines in {directory}: {', '.join(missing)}. "
                       f"Run install_perception.sh")
    return True, ""


class Engine:
    """One built engine, ready to run.

    Device buffers are allocated once at the largest shape the engine's profile
    allows, so a look never waits for a memory allocation and a smaller batch
    simply uses less of the same buffer. That matters more than it sounds on a
    board whose GPU memory *is* its system memory and where the language model
    is holding three gigabytes of it.
    """

    def __init__(self, path: str, vendor: str | None = None) -> None:
        trt, cudart = _imports(vendor or os.path.dirname(path))
        self._cudart = cudart
        self.path = path
        self.logger = trt.Logger(trt.Logger.ERROR)
        self.runtime = trt.Runtime(self.logger)
        # Mapped, not read. `handle.read()` would hold the whole engine in
        # Python memory while TensorRT allocates its own copy, and for the text
        # tower that is 1.1 GB twice over -- measured here as an OutOfMemory on
        # the second of the two, on a board whose 7.4 GB is shared with the GPU
        # and with three gigabytes of language model. A mapping is paged in and
        # dropped again as TensorRT walks it.
        with open(path, "rb") as handle:
            with mmap.mmap(handle.fileno(), 0, access=mmap.ACCESS_READ) as mapped:
                self.engine = self.runtime.deserialize_cuda_engine(mapped)
        if self.engine is None:
            raise NoEngines(
                f"TensorRT would not load {os.path.basename(path)}. An engine is "
                f"tied to the TensorRT version that built it, so rebuild with "
                f"install_perception.sh --engines")
        self.context = self.engine.create_execution_context()
        if self.context is None:
            # TensorRT answers None rather than raising when it cannot find room
            # for the activation arena, and the next line to touch it is a
            # baffling AttributeError several frames away. On this board that is
            # nearly always memory, so say so.
            raise NoEngines(
                f"no room to run {os.path.basename(path)}: TensorRT would not "
                f"make an execution context for it. Something else on this board "
                f"is holding the memory")
        error, self.stream = cudart.cudaStreamCreate()
        _ok(cudart, error, "cudaStreamCreate")

        self.inputs, self.outputs = [], []
        for index in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(index)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.inputs.append(name)
            else:
                self.outputs.append(name)

        import numpy
        for name in self.inputs:
            self.context.set_input_shape(
                name, self.engine.get_tensor_profile_shape(name, 0)[2])
        self._device, self._dtype = {}, {}
        for name in self.inputs + self.outputs:
            shape = tuple(self.context.get_tensor_shape(name))
            dtype = numpy.dtype(trt.nptype(self.engine.get_tensor_dtype(name)))
            error, pointer = cudart.cudaMalloc(
                int(numpy.prod(shape)) * dtype.itemsize)
            _ok(cudart, error, f"cudaMalloc for {name}")
            self._device[name] = pointer
            self._dtype[name] = dtype
            self.context.set_tensor_address(name, pointer)

    def run(self, feed: dict) -> dict:
        """One forward pass. Inputs by name in, outputs by name out."""
        import numpy
        cudart = self._cudart
        for name, array in feed.items():
            array = numpy.ascontiguousarray(array, dtype=self._dtype[name])
            self.context.set_input_shape(name, array.shape)
            _ok(cudart, cudart.cudaMemcpyAsync(
                self._device[name], array.ctypes.data, array.nbytes,
                cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, self.stream),
                f"copying {name} to the GPU")
        if not self.context.execute_async_v3(self.stream):
            raise NoEngines(f"{os.path.basename(self.path)} refused to run")
        answer = {}
        for name in self.outputs:
            shape = tuple(self.context.get_tensor_shape(name))
            out = numpy.empty(shape, dtype=self._dtype[name])
            _ok(cudart, cudart.cudaMemcpyAsync(
                out.ctypes.data, self._device[name], out.nbytes,
                cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost, self.stream),
                f"copying {name} back")
            answer[name] = out
        _ok(cudart, cudart.cudaStreamSynchronize(self.stream), "waiting for the GPU")
        return answer

    def close(self) -> None:
        """Give the GPU memory back.

        Used for the text tower, which embeds the vocabulary at start-up and is
        then dead weight: it is the largest engine of the four and the board
        shares its memory with everything else.
        """
        cudart = self._cudart
        for pointer in self._device.values():
            cudart.cudaFree(pointer)
        self._device.clear()
        if getattr(self, "stream", None) is not None:
            cudart.cudaStreamDestroy(self.stream)
            self.stream = None
        self.context = None
        self.engine = None
        # The runtime goes too. It is small beside the engine, but it is what
        # owns the engine's plugin registry, and leaving one behind per call
        # means a rover that has answered a hundred searches is carrying a
        # hundred of them.
        self.runtime = None
        gc.collect()


def _ok(cudart, error, what: str) -> None:
    """CUDA calls answer with a status first and a value after."""
    if isinstance(error, tuple):
        error = error[0]
    if error != cudart.cudaError_t.cudaSuccess:
        raise NoEngines(f"{what} failed: {error}")
