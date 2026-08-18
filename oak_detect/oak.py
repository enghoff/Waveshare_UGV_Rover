"""The OAK's VPU as a bare inference stick, over ctypes.

There is no depthai here and no pipeline. The camera's Myriad X sits in its ROM
bootloader until a host uploads firmware, and it does not care whose: the chip in
an OAK-D-Lite is the same one Intel sold as a Neural Compute Stick 2, and it
enumerates identically (03e7:2485 unbooted, f63b once running). So this uploads
Intel's own `usb-ma2x8x.mvcmd`, hands the device a graph compiled ahead of time on
a workstation, and then writes frames into one FIFO and reads detections out of
another. The camera's three image sensors are never mentioned and never used.

Everything below the ctypes layer is Intel's XLink and mvnc, vendored under
`vendor/movidius/` and built by `build.sh` into `liboak.so` -- the same C the
OpenVINO MYRIAD plugin calls, which is why none of this protocol had to be
reverse-engineered. The calls that matter are ncDeviceOpen, which boots the chip,
ncGraphAllocate, which uploads the graph, and the two FIFO calls.

**The device is alive only while this object is.** A booted Myriad kills itself
about 1.5 s after its host stops talking to it, which is what the watchdog thread
started by ncDeviceOpen is for. Dropping the handle drops the camera.
"""

import ctypes
import os

NC_OK = 0
NC_USB = 1
NC_MYRIAD_X = 2480
NC_MAX_NAME_SIZE = 64

NC_RW_LOG_LEVEL = 0
NC_LOG_FATAL = 4

NC_RO_GRAPH_INPUT_TENSOR_DESCRIPTORS = 1004
NC_RO_GRAPH_OUTPUT_TENSOR_DESCRIPTORS = 1005
NC_RW_GRAPH_EXECUTORS_NUM = 1110

NC_FIFO_HOST_RO = 0
NC_FIFO_HOST_WO = 1

# graphHeaderLength in OpenVINO's own terms: a packed ELF32 header followed by
# vpu::mv_blob_header, which is twenty uint32s. ncGraphAllocate wants that prefix
# written on its own before the body, and rejects a header longer than the blob.
BLOB_HEADER_BYTES = 52 + 80

_ERRORS = {
    0: "ok", -1: "device busy", -2: "communication error", -3: "out of memory",
    -4: "no device found", -5: "invalid parameters", -6: "timeout",
    -7: "firmware file not found", -8: "not allocated", -9: "unauthorized",
    -10: "unsupported graph file", -11: "unsupported configuration file",
    -12: "unsupported feature", -13: "error reported by the device",
    -14: "invalid data length", -15: "invalid handle",
}


class OakError(RuntimeError):
    pass


class _DeviceDescr(ctypes.Structure):
    _fields_ = [("protocol", ctypes.c_int),
                ("platform", ctypes.c_int),
                ("name", ctypes.c_char * NC_MAX_NAME_SIZE)]


class _OpenParams(ctypes.Structure):
    _fields_ = [("watchdogHndl", ctypes.c_void_p),
                ("watchdogInterval", ctypes.c_int),
                ("memoryType", ctypes.c_char),
                ("customFirmwareDirectory", ctypes.c_char_p)]


class _TensorDescr(ctypes.Structure):
    _fields_ = [("n", ctypes.c_uint), ("c", ctypes.c_uint), ("w", ctypes.c_uint),
                ("h", ctypes.c_uint), ("totalSize", ctypes.c_uint)]


def _check(status, what):
    if status != NC_OK:
        raise OakError("%s: %s (%d)" % (what, _ERRORS.get(status, "unknown"), status))


class Oak:
    """One device, one graph, one inference at a time.

    Deliberately not thread-safe: the graph is allocated with a single executor
    and the device runs one frame at a time regardless, so the caller holds a
    lock rather than this paying for one it cannot use.
    """

    def __init__(self, blob_path, input_shape=(3, 240, 320), library=None,
                 firmware_dir=None, watchdog_ms=1000):
        here = os.path.dirname(os.path.abspath(__file__))
        self._lib = ctypes.CDLL(library or os.path.join(here, "liboak.so"))
        self._blob_path = blob_path
        self._firmware_dir = (firmware_dir or here).rstrip("/") + "/"
        self._watchdog_ms = watchdog_ms
        self._wd = ctypes.c_void_p()
        self._device = ctypes.c_void_p()
        self._graph = ctypes.c_void_p()
        self._fifo_in = ctypes.c_void_p()
        self._fifo_out = ctypes.c_void_p()
        self._out_buffer = None
        # (c, h, w) the graph was compiled for, which has to be told rather than
        # asked. ncGraphGetOption fills in totalSize and leaves n/c/w/h at 1 --
        # the OpenVINO plugin does not read them either, it takes shapes from the
        # blob's own header, and parsing that is a variable-length walk through a
        # section whose dimension order is a permutation code. Declaring the shape
        # and checking it against totalSize catches the mistake that matters,
        # which is a blob and a shape that do not belong together.
        self.input_shape = tuple(input_shape)
        self.input_bytes = 0
        self.output_bytes = 0
        self._declare()

    def _declare(self):
        lib = self._lib
        lib.ncGlobalSetOption.argtypes = [ctypes.c_int, ctypes.c_void_p, ctypes.c_uint]
        lib.ncDeviceOpen.argtypes = [ctypes.POINTER(ctypes.c_void_p), _DeviceDescr, _OpenParams]
        lib.ncDeviceClose.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_void_p]
        lib.ncGraphCreate.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_void_p)]
        lib.ncGraphSetOption.argtypes = [ctypes.c_void_p, ctypes.c_int,
                                         ctypes.c_void_p, ctypes.c_uint]
        lib.ncGraphAllocate.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p,
                                        ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint]
        lib.ncGraphGetOption.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p,
                                         ctypes.POINTER(ctypes.c_uint)]
        lib.ncFifoCreate.argtypes = [ctypes.c_char_p, ctypes.c_int,
                                     ctypes.POINTER(ctypes.c_void_p)]
        lib.ncFifoAllocate.argtypes = [ctypes.c_void_p, ctypes.c_void_p,
                                       ctypes.POINTER(_TensorDescr), ctypes.c_uint]
        lib.ncGraphQueueInferenceWithFifoElem.argtypes = [
            ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint), ctypes.c_void_p]
        lib.ncFifoReadElem.argtypes = [ctypes.c_void_p, ctypes.c_void_p,
                                       ctypes.POINTER(ctypes.c_uint), ctypes.c_void_p]
        lib.watchdog_create.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        lib.oak_jpeg_to_planar_bgr.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.c_void_p, ctypes.c_int, ctypes.c_int,
            ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
        lib.oak_jpeg_to_planar_bgr.restype = ctypes.c_int

    def open(self, quiet=True):
        """Boot the device and load the graph. A couple of seconds, mostly the upload."""
        lib = self._lib
        if quiet:
            level = ctypes.c_int(NC_LOG_FATAL)
            lib.ncGlobalSetOption(NC_RW_LOG_LEVEL, ctypes.byref(level), 4)

        _check(lib.watchdog_create(ctypes.byref(self._wd)), "watchdog_create")

        descr = _DeviceDescr(protocol=NC_USB, platform=NC_MYRIAD_X, name=b"")
        params = _OpenParams(watchdogHndl=self._wd,
                             watchdogInterval=self._watchdog_ms,
                             memoryType=b"\x00",
                             customFirmwareDirectory=self._firmware_dir.encode())
        _check(lib.ncDeviceOpen(ctypes.byref(self._device), descr, params),
               "ncDeviceOpen (booting the VPU)")

        with open(self._blob_path, "rb") as handle:
            blob = handle.read()
        if len(blob) <= BLOB_HEADER_BYTES:
            raise OakError("%s is too short to be a graph blob" % self._blob_path)

        _check(lib.ncGraphCreate(b"detector", ctypes.byref(self._graph)), "ncGraphCreate")
        executors = ctypes.c_int(1)
        _check(lib.ncGraphSetOption(self._graph, NC_RW_GRAPH_EXECUTORS_NUM,
                                    ctypes.byref(executors), 4), "ncGraphSetOption")
        _check(lib.ncGraphAllocate(self._device, self._graph, blob, len(blob),
                                   blob, BLOB_HEADER_BYTES),
               "ncGraphAllocate (uploading the graph)")

        self._in_descr = self._tensor(NC_RO_GRAPH_INPUT_TENSOR_DESCRIPTORS)
        self._out_descr = self._tensor(NC_RO_GRAPH_OUTPUT_TENSOR_DESCRIPTORS)
        self.input_bytes = self._in_descr.totalSize
        self.output_bytes = self._out_descr.totalSize

        channels, height, width = self.input_shape
        if channels * height * width != self.input_bytes:
            raise OakError(
                "%s wants %d input bytes, but the declared %dx%dx%d is %d -- "
                "the blob and the shape do not belong together"
                % (os.path.basename(self._blob_path), self.input_bytes,
                   channels, height, width, channels * height * width))

        _check(lib.ncFifoCreate(b"input", NC_FIFO_HOST_WO, ctypes.byref(self._fifo_in)),
               "ncFifoCreate(input)")
        _check(lib.ncFifoAllocate(self._fifo_in, self._device,
                                  ctypes.byref(self._in_descr), 2),
               "ncFifoAllocate(input)")
        _check(lib.ncFifoCreate(b"output", NC_FIFO_HOST_RO, ctypes.byref(self._fifo_out)),
               "ncFifoCreate(output)")
        _check(lib.ncFifoAllocate(self._fifo_out, self._device,
                                  ctypes.byref(self._out_descr), 2),
               "ncFifoAllocate(output)")
        self._out_buffer = ctypes.create_string_buffer(self.output_bytes)
        return self

    def _tensor(self, option):
        descr = _TensorDescr()
        length = ctypes.c_uint(ctypes.sizeof(_TensorDescr))
        _check(self._lib.ncGraphGetOption(self._graph, option, ctypes.byref(descr),
                                          ctypes.byref(length)), "ncGraphGetOption")
        return descr

    def jpeg_to_input(self, jpeg, buffer):
        """Decode a JPEG straight into the graph's input layout.

        Returns the frame's own size and the size it was decoded at, or None if
        the bytes were not a picture. The frame size is what the caller needs to
        put boxes back into full-frame pixels.
        """
        src_w, src_h = ctypes.c_int(), ctypes.c_int()
        dec_w, dec_h = ctypes.c_int(), ctypes.c_int()
        _, height, width = self.input_shape
        rc = self._lib.oak_jpeg_to_planar_bgr(
            jpeg, len(jpeg), buffer, width, height,
            ctypes.byref(src_w), ctypes.byref(src_h),
            ctypes.byref(dec_w), ctypes.byref(dec_h))
        if rc != 0:
            return None
        return (src_w.value, src_h.value, dec_w.value, dec_h.value)

    def infer(self, planar):
        """One frame in, the raw output tensor out. Blocks until the device answers."""
        length = ctypes.c_uint(self.input_bytes)
        _check(self._lib.ncGraphQueueInferenceWithFifoElem(
            self._graph, self._fifo_in, self._fifo_out, planar,
            ctypes.byref(length), None), "ncGraphQueueInference")
        out_length = ctypes.c_uint(self.output_bytes)
        user = ctypes.c_void_p()
        _check(self._lib.ncFifoReadElem(self._fifo_out, self._out_buffer,
                                        ctypes.byref(out_length), ctypes.byref(user)),
               "ncFifoReadElem")
        return self._out_buffer.raw[:out_length.value]

    def close(self):
        lib = self._lib
        for fifo in (self._fifo_in, self._fifo_out):
            if fifo:
                lib.ncFifoDestroy(ctypes.byref(fifo))
        if self._graph:
            lib.ncGraphDestroy(ctypes.byref(self._graph))
        if self._device:
            lib.ncDeviceClose(ctypes.byref(self._device), self._wd)
        if self._wd:
            lib.watchdog_destroy(self._wd)
            self._wd = ctypes.c_void_p()

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()
