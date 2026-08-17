/* slam2d -- scan-matched 2D localisation and occupancy mapping for the D500.
 *
 * This is the compiled half of the Pi's SLAM. It exists in C rather than in
 * Python because the Pi 1 on the rover is a 700 MHz armv6 with scalar VFP and no
 * NEON, where numpy's per-call overhead dominates any array this small: the same
 * inner loops measured 231 ms per scan under numpy 2.2.4 and 22.8 ms here, and
 * CRC-checking one revolution's 42 packets is 24.7 ms against 0.05 ms. See
 * README.md for the whole table.
 *
 * There is deliberately no loop closure and no pose graph. At the 0.138 ms per
 * candidate pose this hardware manages, the search window slam_toolbox uses in the
 * VM works out to roughly 46 seconds per closure attempt, so what you get here is
 * scan-matched local odometry with honest accumulated drift -- good enough to keep
 * a room-scale occupancy grid and avoid obstacles, not a globally consistent map.
 *
 * Nothing here allocates after slam2d_create, and nothing here does I/O: the
 * caller owns both serial ports and feeds bytes in.
 */
#ifndef SLAM2D_H
#define SLAM2D_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct slam2d slam2d;

typedef struct {
    /* --- map ------------------------------------------------------------- */
    int   grid_cells;        /* map is grid_cells x grid_cells, rover starts centred */
    float resolution_m;      /* metres per cell. 0.03 is what the VM uses; 0.05 here,
                              * because the match cost is dominated by cache misses
                              * into the grid and 5 cm keeps it a third smaller. */

    /* --- sensor ---------------------------------------------------------- */
    float mount_deg;         /* where the sensor's zero points, measured counter-
                              * clockwise from rover forward. 90 on this rover, which
                              * is the same convention as lidar/lidar_view.py. */
    float min_range_m;       /* below this a return is the chassis, not the world */
    float max_range_m;
    int   max_points;        /* decimate each revolution to at most this many points */

    /* --- scan match ------------------------------------------------------ */
    /* Two passes. The coarse one has to span however far the rover can actually
     * have moved between revolutions; the fine one only has to beat the coarse
     * grid's own quantisation. Cost is (2*lin_steps+1)^2 * (2*ang_steps+1). */
    float coarse_lin_m,   coarse_ang_deg;
    int   coarse_lin_steps, coarse_ang_steps;
    float fine_lin_m,     fine_ang_deg;
    int   fine_lin_steps,   fine_ang_steps;
    /* Reject a match whose mean likelihood per point falls below this fraction of
     * the maximum, and fall back to the motion prior instead of believing it. */
    float min_match_score;

    /* --- map update ------------------------------------------------------ */
    int   hit_inc;           /* log-odds added to a cell a beam ended in */
    int   miss_dec;          /* log-odds taken off a cell a beam passed through */
    int   occupied_at;       /* log-odds at or above which a cell counts as occupied */
    int   lik_stamp;         /* peak likelihood written at a hit */
    int   lik_decay;         /* likelihood taken off a cell a beam passed through, so
                              * an obstacle that moves stops attracting the match */

    /* --- the rover's own size -------------------------------------------- */
    float rover_width_m;     /* track width plus whatever clearance you want on it.
                              * Sets the default corridor for the arc check and the
                              * narrowest gap worth reporting as passable. */
} slam2d_config;

/* Defaults measured to run in ~34 ms per revolution on the rover's Pi. */
void slam2d_default_config(slam2d_config *cfg);

/* sizeof(slam2d_config) as this build sees it, so the ctypes binding in slam2d.py
 * can refuse to run against a layout it does not match. Getting that wrong is
 * silent -- the config simply arrives scrambled -- so it is checked rather than
 * assumed. */
int slam2d_config_size(void);
int slam2d_feature_size(void);   /* likewise, for slam2d_feature */

slam2d *slam2d_create(const slam2d_config *cfg);
void    slam2d_destroy(slam2d *s);

/* --- input ---------------------------------------------------------------- */

/* Feed raw bytes from the lidar port. Handles packets split across reads, drops
 * anything failing the LD19 CRC-8, and returns the number of complete revolutions
 * that became available (normally 0 or 1). Only the newest is kept. */
int slam2d_feed_lidar(slam2d *s, const unsigned char *buf, int n);

/* Where the rover thinks it has moved since the last processed revolution: forward
 * metres and yaw radians, counter-clockwise positive. This only centres the search
 * window, so passing 0,0 is legitimate -- at a walking crawl a constant-position
 * prior is inside the coarse window anyway, which is what makes this core useful
 * before the encoder and gyro scale factors have been calibrated. Consumed and
 * cleared by slam2d_update. */
void slam2d_set_prior(slam2d *s, float d_forward_m, float d_yaw_rad);

/* Match the pending revolution, then fold it into the map. Returns 1 if a scan was
 * processed, 0 if none was pending. */
int slam2d_update(slam2d *s);

/* --- output --------------------------------------------------------------- */

void slam2d_pose(const slam2d *s, float *x, float *y, float *theta);
void slam2d_set_pose(slam2d *s, float x, float y, float theta);

/* Mean likelihood per point of the accepted match, 0..1. Zero for the first scan,
 * which has no map to match against. */
float slam2d_score(const slam2d *s);
/* 1 if the last match was rejected as below min_match_score and the prior was used
 * instead. A run where this stays high is lost, whatever the map looks like. */
int   slam2d_rejected(const slam2d *s);
int   slam2d_scans(const slam2d *s);
int   slam2d_points(const slam2d *s);

/* Nearest return per angular sector of the last revolution, in the rover frame:
 * out[0] is the sector centred on straight ahead and they run counter-clockwise.
 *
 * A sector that saw nothing reads **-1**, not max_range_m. The distinction is not
 * pedantic: about 17% of this sensor's returns are zero, and a zero is "no echo",
 * which is what a black sofa or a glass door looks like. Reporting those as
 * open space is precisely how a robot drives into one, so an empty sector is
 * reported as unknown and every caller has to decide what to do about it. */
void slam2d_sectors(const slam2d *s, float *out, int n_sectors);

/* How far the rover can travel along the arc it is about to follow before that arc's
 * corridor is blocked, in metres, capped at max_dist_m.
 *
 * `curvature` is 1/turn-radius in 1/m, positive to the left, 0 for straight ahead.
 * `half_width_m` is half the corridor; pass rover_width_m/2 plus a margin.
 *
 * This is the check that a standoff radius cannot do. A circle around the rover
 * forbids rotating away from a wall that is inside it, whereas the swept arc only
 * objects to what the rover is actually about to drive into -- so turning on the
 * spot beside a wall stays legal, and a wall met at a shallow angle does not read
 * as an obstacle at all. It reads the live scan, never the map, because the map
 * carries drift and stale geometry and this is the decision that must not. */
float slam2d_arc_clearance(const slam2d *s, float curvature, float half_width_m,
                           float max_dist_m);

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

/* Occupancy as signed log-odds, row-major, indexed [ix * grid_cells + iy]. ix runs
 * along rover-forward-at-start, iy along rover-left-at-start. Borrowed, not
 * copied; valid until slam2d_destroy. */
const signed char *slam2d_grid(const slam2d *s);
/* World coordinates in metres of the centre of cell (0,0). */
void slam2d_grid_origin(const slam2d *s, float *ox, float *oy);

#ifdef __cplusplus
}
#endif
#endif /* SLAM2D_H */
