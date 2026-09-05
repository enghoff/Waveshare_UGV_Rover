"""OAK pipeline and range settings. Lengths are mm unless named in metres."""

# Aligned RGB/depth at 15 fps: 640x360 JPEG plus 320x180 uint16 depth.
# USB2 is selected in Depth.run; the two-second history covers encoder lag.
# 200..6000 mm is the usable stereo interval. Invalid pixels stay invalid.
# Sector ranges use the 5th percentile; object boxes use the 20th percentile
# and gather the near surface within max(0.30 m, 15% of its range).
# Disparity sigma is an assumed model, pending tape-measure validation.

DEFAULT_PORT = 8770

DEFAULT_BIND = "127.0.0.1"

DEFAULT_FPS = 15

DECIMATION = 2

COLOUR_SIZE = (640, 360)

COLOUR_ISP_SCALE = (1, 3)

JPEG_QUALITY = 90

DEPTH_SIZE = (320, 180)

COLOUR_HISTORY_S = 2.0

FRAME_TIMEOUT_S = 5.0

WAKE_TIMEOUT_S = 40.0

MIN_MM, MAX_MM = 200, 6000

GRID_COLS, GRID_ROWS = 8, 6

BAND = (0.30, 0.70)

NEAR_PERCENTILE = 5

STAT_WINDOW = 100

RANGE_PERCENTILE = 20

RANGE_BAND_FRAC = 0.15

RANGE_BAND_M = 0.30

RANGE_MIN_PIXELS = 12

DISPARITY_SIGMA_PX = 0.2
