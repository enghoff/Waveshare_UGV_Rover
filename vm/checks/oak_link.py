"""OAK bring-up probe for the VM.

The question this answers is narrow: reached through VMware USB passthrough, does
the camera survive uploading its firmware and re-enumerating from PID 2485 to
PID F63B, and then stream? Prints the boot latency, because that is the number
that decides whether the device's watchdog beats the host to the reconnection.
"""

import time

import depthai as dai

print("depthai", dai.__version__)

infos = dai.Device.getAllAvailableDevices()
print("available:", [(i.getMxId(), i.state.name, i.protocol.name) for i in infos])
if not infos:
    raise SystemExit("no device visible to the guest -- USB passthrough is not working")

pipeline = dai.Pipeline()
mono = pipeline.create(dai.node.MonoCamera)
mono.setBoardSocket(dai.CameraBoardSocket.CAM_C)
mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
mono.setFps(15)
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("right")
mono.out.link(xout.input)

# monotonic, not wall clock: the guest's clock gets stepped by time sync and a
# step mid-measurement produced a negative duration on the first run.
t0 = time.monotonic()
with dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH) as dev:
    print(f"opened in {time.monotonic() - t0:.1f} s  (includes firmware upload + re-enumeration)")
    print("device:", dev.getDeviceName(), "| product:", dev.getProductName())
    print("usb speed:", dev.getUsbSpeed().name)
    print("cameras:", [c.socket.name for c in dev.getConnectedCameraFeatures()])

    q = dev.getOutputQueue("right", 4, blocking=False)
    frames, start = 0, time.monotonic()
    while frames < 90 and time.monotonic() - start < 20:
        if q.tryGet() is not None:
            frames += 1
    elapsed = time.monotonic() - start
    print(f"frames: {frames} in {elapsed:.1f} s -> {frames / elapsed:.1f} fps")

    if dev.hasCrashDump():
        print("WARNING: device reports a stored crash dump")
