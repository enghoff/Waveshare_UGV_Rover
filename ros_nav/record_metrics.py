"""Pure metrics shared by live navigation recording and offline replay."""
import math

def turning_of(poses, floor=0.01):
    """Heading actually turned, with the noise floor taken out.

    Summing every |delta yaw| looks right and is not: at 10 Hz over five
    minutes it adds up three thousand samples of gyro noise and reports
    thirty-four degrees of turning for a rover that never moved. That would
    quietly break the one test that says whether a recording is worth
    analysing. Anything under half a degree between samples is not a turn.
    """
    total = 0.0
    for a, b in zip(poses, poses[1:]):
        step = abs(math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2])))
        if step >= floor:
            total += step
    return total
