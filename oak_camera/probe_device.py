"""Enumerate attached OAK devices and report what each one actually is.

The USB descriptor only says "Movidius MyriadX" -- the board model, the sensors
populated on it and the factory calibration all live on the device and come back
only after depthai boots it. This script boots each attached device and prints
that.
"""

import sys
import time

import depthai as dai


def open_device(info: dai.DeviceInfo) -> dai.Device:
    """Open the device, preferring USB3 but falling back to the USB2-only firmware.

    The other scripts pin `UsbSpeed.HIGH` outright, because on this cable the
    USB3-enabled firmware usually fails to re-enumerate after boot and dies on
    its own watchdog. This script is the diagnostic, so it asks for USB3 first:
    if that ever starts working, the fallback message is how you find out.
    """
    try:
        return dai.Device(dai.Pipeline(), info, dai.UsbSpeed.SUPER_PLUS)
    except RuntimeError as exc:
        print(f"  usb3 firmware : failed to boot ({exc}); retrying USB2-only")
        return dai.Device(dai.Pipeline(), wait_unbooted(info.getMxId()), dai.UsbSpeed.HIGH)


def wait_unbooted(mxid: str, timeout: float = 45.0) -> dai.DeviceInfo:
    """Block until `mxid` is back in UNBOOTED state, and return its fresh DeviceInfo.

    A device that failed to boot is still running firmware, or crashing on its
    watchdog, for some seconds afterwards; connecting during that window fails
    with X_LINK_INSUFFICIENT_PERMISSIONS. It drops back to UNBOOTED on its own.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for info in dai.Device.getAllAvailableDevices():
            if info.getMxId() == mxid and info.state == dai.XLinkDeviceState.X_LINK_UNBOOTED:
                return info
        time.sleep(1.0)
    raise RuntimeError(f"{mxid} did not return to UNBOOTED within {timeout:.0f}s; replug it")


def describe(info: dai.DeviceInfo) -> None:
    print(f"--- {info.getMxId()} @ {info.name} ---")
    print(f"  protocol      : {info.protocol.name}")
    print(f"  state         : {info.state.name}")

    with open_device(info) as device:
        print(f"  board name    : {device.getDeviceName()}")
        print(f"  product name  : {device.getProductName()}")
        print(f"  usb speed     : {device.getUsbSpeed().name}")
        print(f"  bootloader    : {device.getBootloaderVersion()}")
        print(f"  imu           : {device.getConnectedIMU() or 'none'}")
        print(f"  ir drivers    : {device.getIrDrivers() or 'none'}")
        print(f"  temperature   : {device.getChipTemperature().average:.1f} C")

        cameras = device.getConnectedCameraFeatures()
        print(f"  cameras       : {len(cameras)}")
        for cam in cameras:
            types = "/".join(t.name for t in cam.supportedTypes)
            focus = "AF" if cam.hasAutofocus else "FF"
            print(
                f"    {cam.socket.name:<7} {cam.sensorName:<9} {focus}"
                f"  {cam.width}x{cam.height}  {types}"
            )

        for pair in device.getAvailableStereoPairs():
            print(f"  stereo pair   : {pair.left.name} / {pair.right.name}")

        calib = device.readCalibration()
        print(f"  baseline      : {calib.getBaselineDistance():.2f} cm")
        for cam in cameras:
            print(f"  fov {cam.socket.name:<7}: {calib.getFov(cam.socket):.1f} deg horizontal")


def main() -> int:
    devices = dai.Device.getAllAvailableDevices()
    if not devices:
        print("No OAK device found.", file=sys.stderr)
        print(
            "Check Device Manager for 'Movidius MyriadX' under USB devices; if it "
            "is absent the camera is not enumerating at all.",
            file=sys.stderr,
        )
        return 1

    print(f"depthai {dai.__version__}, {len(devices)} device(s)\n")
    for info in devices:
        describe(info)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
