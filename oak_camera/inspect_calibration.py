"""Dump the calibration stored on the device, per camera socket.

Compares the user calibration in use against the factory copy, so a corrupt or
overwritten user calibration is obvious. On this camera the two are identical,
and the mono distortion coefficients are asymmetric by a wide margin -- CAM_B's
k1 is +41.9 where CAM_C's is -1.7. That is what the factory wrote and both
sensors stream fine under depthai 2.x, so it is not a fault; it is worth knowing
before trusting undistortion on the left camera.
"""

import depthai as dai

SOCKETS = [
    dai.CameraBoardSocket.CAM_A,
    dai.CameraBoardSocket.CAM_B,
    dai.CameraBoardSocket.CAM_C,
]

COEFF_NAMES = ["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6",
               "s1", "s2", "s3", "s4", "tx", "ty"]


def summarise(label: str, calib: dai.CalibrationHandler) -> None:
    print(f"\n=== {label} ===")
    print(f"  stereo pair : {calib.getStereoLeftCameraId().name}"
          f" / {calib.getStereoRightCameraId().name}")
    print(f"  baseline    : {calib.getBaselineDistance():.2f} cm")

    for socket in SOCKETS:
        k = calib.getCameraIntrinsics(socket)
        _, w, h = calib.getDefaultIntrinsics(socket)
        print(f"  -- {socket.name} (calibrated at {w}x{h})")
        print(f"     fx={k[0][0]:.2f} fy={k[1][1]:.2f}"
              f" cx={k[0][2]:.2f} cy={k[1][2]:.2f}")
        dist = calib.getDistortionCoefficients(socket)
        print("     " + "  ".join(
            f"{n}={v:+.4f}" for n, v in zip(COEFF_NAMES, dist)
        ))


def main() -> int:
    # USB2-only firmware; the USB3 build usually fails to boot here. See docs/oak-usb-link.md.
    with dai.Device(dai.Pipeline(), maxUsbSpeed=dai.UsbSpeed.HIGH) as device:
        print(f"device {device.getMxId()} -- {device.getProductName()}")
        for cam in device.getConnectedCameraFeatures():
            print(f"  {cam.socket.name} {cam.sensorName}"
                  f" sensor {cam.width}x{cam.height}")

        summarise("user calibration (in use)", device.readCalibration())
        summarise("factory calibration", device.readFactoryCalibration())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
