/* slam2d -- an LD19 lidar parser, and the room it sees described in words.
 *
 * The name is historical and the history is worth one paragraph. This was the
 * compiled half of the rover's own SLAM: a correlative scan matcher and an
 * occupancy grid, in C rather than in Python because the Pi 1 it first ran on is a
 * 700 MHz armv6 with scalar VFP and no NEON, where numpy's per-call overhead
 * dominates any array this small. The rover now navigates with `slam_toolbox` and
 * Nav2, which do that job with the loop closure this could never afford, so the
 * matcher and the map are gone.
 *
 * What survives is what has no replacement. The parser is one of them -- the
 * 47-byte packets, the CRC-8 over each, and the wrap that ends a revolution, in
 * 0.3 ms where Python takes 25 -- and `slam2d_features` is the other, because a
 * language model asked about a list of 360 ranges will hallucinate over it and a
 * language model told about walls, objects and gaps can say something true.
 *
 * Nothing here allocates after slam2d_create, and nothing here does I/O: the
 * caller owns the serial port and feeds bytes in.
 */
#ifndef SLAM2D_H
#define SLAM2D_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct slam2d slam2d;

typedef struct {
    /* --- sensor ---------------------------------------------------------- */
    float mount_deg;         /* where the sensor's zero points, measured counter-
                              * clockwise from rover forward. 90 on this rover, which
                              * is the same convention as lidar/lidar_view.py. */
    float min_range_m;       /* below this a return is the chassis, not the world */
    float max_range_m;
    /* The rover's own structure, as the sensor sees it: a box behind the lidar that
     * returns inside are discarded from. A ring of range alone cannot do this job --
     * the two mount posts on this rover sit 12 to 16 cm out, well beyond anything it
     * would be safe to blind the sensor to in front, and they were being reported as
     * an obstacle 13 cm away in 59% of revolutions. Measured on the rover; see
     * docs/d500-lidar.md. Zero either field to switch the mask off. */
    float body_back_m;       /* how far behind the lidar the rover's structure runs */
    float body_half_width_m; /* and how far to each side of it */
    int   max_points;        /* decimate each revolution to at most this many points */

    /* --- the rover's own size -------------------------------------------- */
    float rover_width_m;     /* track width plus whatever clearance you want on it.
                              * Sets the narrowest gap worth reporting as passable. */
} slam2d_config;

void slam2d_default_config(slam2d_config *cfg);

/* sizeof(slam2d_config) as this build sees it, so the ctypes binding in slam2d.py
 * can refuse to run against a layout it does not match. Getting that wrong is
 * silent -- the config simply arrives scrambled -- so it is checked rather than
 * assumed. */
int slam2d_config_size(void);
int slam2d_feature_size(void);   /* likewise, for slam2d_feature */

slam2d *slam2d_create(const slam2d_config *cfg);
void    slam2d_destroy(slam2d *s);

/* Drop a half-parsed packet and a half-assembled revolution. Call this when the
 * lidar port is (re)opened: the sensor is already spinning, so the first bytes are
 * mid-packet and the first wrap is a remnant of the revolution the parser joined
 * in the middle of. */
void slam2d_resync(slam2d *s);

/* --- input ---------------------------------------------------------------- */

/* Feed raw bytes from the lidar port. Handles packets split across reads, drops
 * anything failing the LD19 CRC-8, and returns the number of complete revolutions
 * that became available (normally 0 or 1). Only the newest is kept. A wrap that
 * has not covered 270 degrees is not a revolution -- that is the remnant of
 * joining a spinning sensor mid-turn, and it is discarded rather than kept. */
int slam2d_feed_lidar(slam2d *s, const unsigned char *buf, int n);

/* --- output --------------------------------------------------------------- */

/* Nearest return per angular sector of the last revolution, in the rover frame:
 * out[0] is the sector centred on straight ahead and they run counter-clockwise.
 *
 * A sector that saw nothing reads **-1**, not max_range_m. The distinction is not
 * pedantic: about 17% of this sensor's returns are zero, and a zero is "no echo",
 * which is what a black sofa or a glass door looks like. Reporting those as
 * open space is precisely how a robot drives into one, so an empty sector is
 * reported as unknown and every caller has to decide what to do about it. */
void slam2d_sectors(const slam2d *s, float *out, int n_sectors);

/* --- describing the surroundings ------------------------------------------ */

enum {
    SLAM2D_WALL = 0,    /* a long straight run: a wall, a sofa front, a worktop */
    SLAM2D_OBJECT = 1,  /* a small isolated cluster: a chair leg, a box, a foot */
    SLAM2D_GAP = 2,     /* traversable space between two things, wider than the rover */
};

typedef struct {
    int   kind;
    float bearing_deg;      /* rover frame, ccw from forward, -180..180 */
    float range_m;          /* to the nearest part of it */
    float width_m;          /* chord for a wall or object, opening for a gap */
    float span_deg;         /* how much of the horizon it occupies */
    float straightness_m;   /* worst deviation from its own chord; small means flat */
    /* The ends of the run, in rover-frame metres, in increasing bearing order. A
     * wall's nearest point is usually nowhere near either end -- which is exactly
     * why the gap between two features has to be measured from these and not from
     * bearing_deg and range_m. */
    float x0, y0, x1, y1;
} slam2d_feature;

/* Segment the last revolution into things a person -- or a language model -- can
 * reason about, rather than a list of ranges. Returns how many were written.
 *
 * The lidar sees one horizontal slice, so a table is four small objects in a square
 * and never a table; naming it is the caller's job, and this exists to give the
 * caller something nameable. Clusters are split at range discontinuities and then
 * at corners, so a rectangular room comes back as four walls rather than one
 * lumpy ring. Single-point returns are dropped as noise -- which is safe here only
 * because obstacle avoidance reads the raw scan, not this.
 *
 * Takes a non-const handle because it sorts into a scratch buffer inside the
 * instance. Like everything else here it is not internally locked. */
int slam2d_features(slam2d *s, slam2d_feature *out, int max_out);

/* The last revolution in the rover frame as x,y metre pairs. Returns the number of
 * points written, at most max_pts. */
int slam2d_scan_xy(const slam2d *s, float *xy, int max_pts);

#ifdef __cplusplus
}
#endif
#endif /* SLAM2D_H */
