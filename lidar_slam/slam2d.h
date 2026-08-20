/* slam2d -- scan-matched 2D localisation and occupancy mapping for the D500.
 *
 * This is the compiled half of the Pi's SLAM. It exists in C rather than in
 * Python because the Pi 1 on the rover is a 700 MHz armv6 with scalar VFP and no
 * NEON, where numpy's per-call overhead dominates any array this small: the same
 * inner loops measured 231 ms per scan under numpy 2.2.4 and 22.8 ms here, and
 * CRC-checking one revolution's 42 packets is 24.7 ms against 0.05 ms. See
 * README.md for the whole table.
 *
 * There is deliberately no loop closure and no pose graph. At the 0.058 ms per
 * candidate pose this hardware manages, the search window slam_toolbox uses by
 * default works out to roughly 19 seconds per closure attempt, so what you get is
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
    float resolution_m;      /* metres per cell. 0.05 rather than the more usual
                              * 0.03, because match cost is dominated by cache misses
                              * into the grid and 5 cm keeps it a third smaller. */

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

    /* --- recovery -------------------------------------------------------- */
    /* A one-off wide search, asked for with slam2d_request_recovery.
     *
     * The coarse window above spans what the rover can move in one revolution,
     * which is the right size for tracking and far too small for re-finding a pose
     * somebody else has moved. A dead-reckoned turn here is open-loop PWM against a
     * measured rate, and one has been observed 48 degrees out -- five times the
     * coarse window. The match cannot climb back from that and, worse, does not
     * report it as a failure: it returns the largest rotation it was allowed to
     * consider and scores it well, because the scan does fit the map somewhere.
     *
     * Wide in angle and narrow in translation, because a turn errs in heading and
     * hardly moves. Keep recover_ang_deg equal to coarse_ang_deg: the fine pass is
     * sized to beat one coarse step and is reused unchanged after this one. */
    float recover_lin_m,   recover_ang_deg;
    int   recover_lin_steps, recover_ang_steps;
    /* How far away in heading a rival peak has to be before it counts as a
     * different answer rather than the shoulder of the same one. Only meaningful
     * when the search is wider than this, so in practice only during recovery. */
    float ambiguity_sep_deg;

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

/* Throw the map away and stand the rover at the origin of an empty one.
 *
 * Every cell is a claim about a place, and every one of those claims was written
 * from a pose that has been drifting since the first revolution. Once the drift
 * has grown past the point where the map helps -- a corridor stamped in twice, a
 * room a few degrees out of true with itself -- there is nothing to salvage cell
 * by cell, and with no loop closure here nothing that will ever repair it. The
 * cheapest true map is an empty one.
 *
 * The pose goes back to the origin rather than staying where it is, because the
 * grid is finite and centred on the origin: a rover that has driven 6 m from
 * where it started would otherwise get a blank map with a third of it already
 * behind it. So everything the caller is holding in world coordinates -- a route,
 * a driven track, somewhere worth coming back to -- refers to an origin that no
 * longer exists and has to go with it.
 *
 * The likelihood field goes too, not just the occupancy grid: the field is what
 * the matcher slides a scan over, so clearing only the picture would leave the
 * old room still deciding where the rover is. The revolution already parsed is
 * kept and seeds the new map on the next update, exactly as the first one after
 * create does. Allocates nothing. */
void slam2d_reset(slam2d *s);

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
 * processed, 0 if none was pending.
 *
 * A rejected match is never folded in. The pose it would have been written from is
 * one this code has just said it does not believe, and a scan stamped at a pose
 * that is tens of degrees out does not merely go unused -- it becomes part of what
 * the next revolution matches against, at full likelihood, and from then on the
 * wrong answer has evidence for it. Skipping the update leaves the map a little
 * staler and entirely true, which is the trade worth making every time. */
int slam2d_update(slam2d *s);

/* Match, but do not write the map, until told otherwise.
 *
 * For the caller that has just moved the pose itself and cannot yet vouch for
 * where it put it. Matching continues, so the pose keeps being corrected and
 * `score` keeps saying how well it fits, but nothing is stamped -- so a re-seed
 * that turns out to be wrong costs a few revolutions of pose and no map at all.
 * Turn it back on once a match has confirmed the pose. On by default; also turned
 * back on by slam2d_reset, since a map you have just asked to be rebuilt is by
 * definition one you want written. */
void slam2d_set_mapping(slam2d *s, int on);
int  slam2d_mapping(const slam2d *s);

/* Search the recovery window instead of the coarse one on the next update, once.
 *
 * Costs roughly three times a normal match at the defaults, which is why it is a
 * request and not the standing behaviour: it is affordable as a one-off after a
 * turn, and not ten times a second. */
void slam2d_request_recovery(slam2d *s);

/* --- output --------------------------------------------------------------- */

void slam2d_pose(const slam2d *s, float *x, float *y, float *theta);
void slam2d_set_pose(slam2d *s, float x, float y, float theta);

/* Mean likelihood per point of the accepted match, 0..1. Zero for the first scan,
 * which has no map to match against. */
float slam2d_score(const slam2d *s);
/* 1 if the last match was rejected as below min_match_score and the prior was used
 * instead. A run where this stays high is lost, whatever the map looks like. */
int   slam2d_rejected(const slam2d *s);

/* --- how the last match was won, which the score alone does not say ------- */
/*
 * A scan that has snapped onto the wrong-but-self-consistent alignment scores
 * *high* -- scoring high is why that pose won -- so `score` cannot tell a good fix
 * from a confident mistake. These two can.
 */

/* 1 if the winning coarse candidate sat on the rim of the search lattice, in any
 * of x, y or heading. The true pose was then probably outside the window and what
 * came back is the nearest edge of what was searched, not a fit -- the failure the
 * coarse window comment warns about, made visible instead of silent. */
int   slam2d_match_edge(const slam2d *s);

/* The best rival peak as a fraction of the winner, comparing only headings at
 * least ambiguity_sep_deg away, so 0.98 means the room has two answers and this
 * one was picked by a hair. Zero when the search was never wide enough to hold a
 * rival that far out, which is the normal tracking case -- read it after a
 * recovery search, not every revolution. */
float slam2d_ambiguity(const slam2d *s);

/* The whole correlation-against-heading curve of the last coarse pass: `offsets`
 * gets each candidate heading as degrees from the pose the search started at, and
 * `scores` the best likelihood found at it, normalised 0..1. Returns how many
 * were written.
 *
 * This is the one artifact worth logging when a map comes out misaligned, because
 * the three ways a match goes wrong look different in it: a peak against the end
 * of the curve is a window too narrow, two comparable peaks are a room that does
 * not say which way round the rover is, and one broad low hump is a scan with
 * nothing in it worth matching. */
int   slam2d_angle_profile(const slam2d *s, float *offsets, float *scores,
                           int max_n);
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
