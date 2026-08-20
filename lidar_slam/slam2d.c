/* slam2d -- see slam2d.h for what this is and why it is not Python.
 *
 * Three things happen here, in order, once per revolution of the lidar:
 * the raw byte stream is turned into a list of points in the rover's frame, the
 * pose is found by sliding that list over a likelihood field until it fits, and
 * the field and the occupancy grid are updated from where the pose says the
 * points landed.
 */
#include "slam2d.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* M_PI is POSIX, not ISO C, and build.sh compiles with -std=c99 on purpose. */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define PACKET_LEN  47
#define PKT_POINTS  12
#define LUT_BINS    3600            /* 0.1 deg. The sensor resolves 0.72 deg
                                     * (419 points a revolution), so this is
                                     * already finer than the data, and at 28.8 kB
                                     * it has some hope of staying in cache. */
#define RX_CAP      16384           /* one revolution is ~1974 bytes */
#define MAX_POINTS  2048
#define KERN        2               /* likelihood kernel half-width, in cells */
#define MAX_ANG_BINS 129            /* candidate headings one pass may search, so
                                     * recover_ang_steps tops out at 64. The profile
                                     * across them is kept for the caller. */

typedef struct {
    uint16_t bearing;               /* rover-frame, centi-degrees, ccw from forward */
    float    range;                 /* metres */
    float    x, y;                  /* rover frame, metres */
} point;

struct slam2d {
    slam2d_config cfg;
    int   cells;
    float inv_res;

    signed char   *occ;             /* log-odds occupancy, [ix * cells + iy] */
    unsigned char *lik;             /* likelihood field the match slides over */

    uint8_t crc_tab[256];
    float   lut_cos[LUT_BINS], lut_sin[LUT_BINS];
    uint8_t kernel[2 * KERN + 1][2 * KERN + 1];

    unsigned char rx[RX_CAP];
    int  rx_len;
    int  prev_start;                /* last packet's start angle, for wrap detection */
    int  acc_from;                  /* start angle of the first packet in acc, or -1 */

    point pts_a[MAX_POINTS], pts_b[MAX_POINTS];
    point *acc, *pend;              /* accumulating / complete */
    int    acc_n, pend_n;
    int    pend_ready;
    /* Bearing-sorted copy of the pending scan, for slam2d_features. Per instance
     * rather than a file static: the daemon calls tools on connection threads, and
     * two descriptions at once must not share a scratch buffer. The instance as a
     * whole still needs external locking -- see slam2d.py. */
    point sorted[MAX_POINTS];

    float rot_x[MAX_POINTS], rot_y[MAX_POINTS];   /* scratch for the match */

    float x, y, th;                 /* pose, metres and radians */
    float prior_fwd, prior_yaw;
    float score;
    int   rejected, scans;

    int   seeded;                   /* a scan has been written, so there is something
                                     * to match against */
    int   mapping;                  /* 0 = match but write nothing to the map */
    int   recover;                  /* one-shot: search the wide window next update */
    int   edge;                     /* the coarse winner sat on the lattice rim */
    float ambiguity;                /* best distant rival / winner, 0..1 */
    /* The last coarse pass's best score at each candidate heading, and what that
     * heading was as an offset from where the search started. Filled by the pass
     * that is running anyway, so it costs an array and no arithmetic. */
    int   ang_bins;
    float ang_off[MAX_ANG_BINS];
    long  ang_best[MAX_ANG_BINS];
};

/* ------------------------------------------------------------------ setup */

void slam2d_default_config(slam2d_config *cfg)
{
    cfg->grid_cells   = 400;        /* 20 m square at 5 cm */
    cfg->resolution_m = 0.05f;

    cfg->mount_deg    = 90.0f;      /* this rover; matches lidar/lidar_view.py */
    cfg->min_range_m  = 0.12f;
    cfg->max_range_m  = 8.0f;
    /* Fitted to what the sensor actually reports of the rover, not to a drawing: the
     * offending returns span 8.5 to 11.2 cm behind the lidar and 8.2 to 10.7 cm to
     * each side, over 397 revolutions. These are those bounds with about 5 cm of
     * margin, and still comfortably inside the chassis' own 17 cm half-width, so
     * nothing this mask can hide is anywhere but on the rover itself. */
    cfg->body_back_m       = 0.16f;
    cfg->body_half_width_m = 0.14f;
    /* The sensor delivers ~419 points a revolution and every one of them costs a
     * cache miss in every candidate pose, so the scan is thinned to 300. That is
     * still 1.2 deg of angular resolution against a 5 cm grid, and it bought 25 ms
     * a revolution -- the difference between leaving the rest of the Pi some room
     * and not. */
    cfg->max_points   = 300;

    /* +/-0.10 m and +/-9 deg of coarse window: 1.0 m/s and 90 deg/s at 10 Hz.
     *
     * The angular half was +/-6 and that was not enough. Rotation beyond the window
     * does not merely go unmatched, it comes back *under-reported* -- the matcher
     * returns the largest rotation it was allowed to consider -- and a controller
     * closing on that measurement keeps turning to make up a difference that is not
     * there. Commanded 90 degree turns overshot by around 40%. Three extra angles
     * cost about 7 ms a revolution, which is worth it to stop the measurement
     * saturating quietly.
     *
     * Paid for out of the linear window, which was over-generous: +/-0.15 m is
     * 1.5 m/s and the rover's top speed is 0.35, so 3.5 cm a revolution against a
     * 10 cm window is still a threefold margin. Cost goes as the *square* of the
     * linear steps and only linearly in the angular ones, so trading one for the
     * other is not even: 5x5x7 coarse plus 5x5x5 fine is 300 poses against the 370
     * this replaces, so the angular window widens by half and the whole match gets
     * cheaper. */
    cfg->coarse_lin_m     = 0.05f;  cfg->coarse_lin_steps = 2;
    cfg->coarse_ang_deg   = 3.0f;   cfg->coarse_ang_steps = 3;
    /* The fine pass only has to beat the coarse grid, so it spans one coarse step. */
    cfg->fine_lin_m       = 0.0125f; cfg->fine_lin_steps  = 2;
    cfg->fine_ang_deg     = 0.75f;   cfg->fine_ang_steps  = 2;
    cfg->min_match_score  = 0.15f;
    cfg->min_write_score  = 0.35f;

    /* +/-60 deg and +/-0.05 m of recovery window, in the same 3 deg steps as the
     * coarse pass so the fine pass after it still fits.
     *
     * Wide in angle and deliberately narrow in translation. 41 headings x 9 offsets
     * is 369 poses against the coarse pass's 175, so a recovery match costs about
     * half as much again as a normal one rather than three times -- and it has to
     * be affordable every revolution, because a rover that is lost goes on asking
     * for one until it is found. Cost grows as the square of the linear steps and
     * only linearly in the angular ones, and a rover turning on the spot errs by
     * tens of degrees in heading and by centimetres in position, so this is where
     * the budget belongs.
     *
     * 60 rather than 30 because the error being recovered from is a fraction of the
     * whole turn: the worst seen was 48 degrees, on a 90 that physically managed
     * 42. Wider than this starts finding rivals in an ordinary room faster than it
     * finds the answer, which is what ambiguity_sep_deg is there to catch. */
    cfg->recover_lin_m     = 0.05f;  cfg->recover_lin_steps = 1;
    cfg->recover_ang_deg   = 3.0f;   cfg->recover_ang_steps = 20;
    /* Two peaks closer together than this in heading are the same peak. 20 deg is
     * comfortably past the shoulder of a genuine one -- a 300-point scan of a room
     * falls off within a few degrees -- and well inside the symmetries that matter,
     * which arrive at 90 and 180. */
    cfg->ambiguity_sep_deg = 20.0f;

    cfg->hit_inc    = 12;
    cfg->miss_dec   = 3;            /* asymmetric on purpose: a beam that passes
                                     * through is weaker evidence than one that
                                     * stops, so clearing is slower than marking */
    cfg->occupied_at = 20;
    cfg->lik_stamp   = 255;
    cfg->lik_decay   = 8;

    /* The UGV Rover is about 22 cm across the tracks; 0.34 leaves a hand's width
     * either side, which is also roughly the standoff worth planning gaps around. */
    cfg->rover_width_m = 0.34f;
}

int slam2d_config_size(void)  { return (int)sizeof(slam2d_config); }
int slam2d_feature_size(void) { return (int)sizeof(slam2d_feature); }

static void build_crc(uint8_t tab[256])
{
    /* LD19: bytewise CRC-8, polynomial 0x4d, init 0, no reflection. Checking it is
     * what separates a real frame from a 0x54 that happens to fall in a distance
     * field, which is why the parser below can afford to resynchronise by hand. */
    for (int i = 0; i < 256; i++) {
        int c = i;
        for (int k = 0; k < 8; k++)
            c = (c & 0x80) ? ((c << 1) ^ 0x4D) & 0xFF : (c << 1) & 0xFF;
        tab[i] = (uint8_t)c;
    }
}

static void build_lut(slam2d *s)
{
    /* The sensor reports a left-handed bearing: zero at the front of the sensor,
     * growing clockwise. The rover frame is right-handed with x forward and y
     * left. Both the flip and the mount offset collapse into one angle,
     * phi = mount - bearing, so the LUT hands back a rover-frame unit vector
     * directly and no sign fixing survives past this function. With mount_deg at
     * 90 this reduces to x = sin(bearing), y = cos(bearing), which is the same
     * geometry lidar/lidar_view.py draws. */
    for (int i = 0; i < LUT_BINS; i++) {
        double phi = (s->cfg.mount_deg - i * (360.0 / LUT_BINS)) * (M_PI / 180.0);
        s->lut_cos[i] = (float)cos(phi);
        s->lut_sin[i] = (float)sin(phi);
    }
}

static void build_kernel(slam2d *s)
{
    /* A Gaussian at sigma = 1 cell, so a hit raises its own cell and the ring
     * around it. The lidar is good to about +/-20 mm, far tighter than the 10 cm
     * this smears over -- the width is not modelling the sensor, it is widening
     * the basin the correlative match has to fall into, which is what lets the
     * coarse pass step a whole 5 cm cell at a time without walking past the peak. */
    for (int dx = -KERN; dx <= KERN; dx++)
        for (int dy = -KERN; dy <= KERN; dy++) {
            double w = exp(-(double)(dx * dx + dy * dy) / 2.0);
            s->kernel[dx + KERN][dy + KERN] = (uint8_t)(s->cfg.lik_stamp * w + 0.5);
        }
}

slam2d *slam2d_create(const slam2d_config *cfg)
{
    if (!cfg || cfg->grid_cells < 16 || cfg->grid_cells > 4096) return NULL;
    if (cfg->resolution_m <= 0.0f || cfg->max_range_m <= cfg->min_range_m) return NULL;

    slam2d *s = calloc(1, sizeof *s);
    if (!s) return NULL;
    s->cfg = *cfg;
    if (s->cfg.max_points < 1) s->cfg.max_points = 1;
    if (s->cfg.max_points > MAX_POINTS) s->cfg.max_points = MAX_POINTS;
    /* Clamped rather than rejected: the profile buffer is what sets the ceiling,
     * and a caller asking for a wider sweep than it holds wants the widest sweep
     * available, not a NULL handle. */
    if (s->cfg.coarse_ang_steps  > MAX_ANG_BINS / 2) s->cfg.coarse_ang_steps  = MAX_ANG_BINS / 2;
    if (s->cfg.recover_ang_steps > MAX_ANG_BINS / 2) s->cfg.recover_ang_steps = MAX_ANG_BINS / 2;
    if (s->cfg.recover_ang_steps < 0) s->cfg.recover_ang_steps = 0;
    if (s->cfg.recover_lin_steps < 0) s->cfg.recover_lin_steps = 0;
    s->mapping = 1;

    s->cells   = cfg->grid_cells;
    s->inv_res = 1.0f / cfg->resolution_m;

    size_t n = (size_t)s->cells * s->cells;
    s->occ = calloc(n, 1);
    s->lik = calloc(n, 1);
    if (!s->occ || !s->lik) { slam2d_destroy(s); return NULL; }

    build_crc(s->crc_tab);
    build_lut(s);
    build_kernel(s);

    s->acc = s->pts_a;
    s->pend = s->pts_b;
    s->prev_start = -1;
    s->acc_from = -1;
    return s;
}

void slam2d_destroy(slam2d *s)
{
    if (!s) return;
    free(s->occ);
    free(s->lik);
    free(s);
}

static void parser_resync(slam2d *s)
{
    /* The byte stream and the revolution being assembled, not the map or a scan
     * already handed over. Joining mid-packet or mid-revolution is the normal
     * case after a port open, and mixing that remnant with the next wrap is how
     * a restart used to seed a wedge of room. */
    s->rx_len = 0;
    s->acc_n = 0;
    s->prev_start = -1;
    s->acc_from = -1;
}

void slam2d_resync(slam2d *s)
{
    if (s) parser_resync(s);
}

void slam2d_reset(slam2d *s)
{
    if (!s) return;
    size_t n = (size_t)s->cells * s->cells;
    /* Both grids. Clearing the occupancy alone would blank the picture while the
     * likelihood field went on holding the old room -- and the field is the half
     * the matcher actually reads, so the rover would be localised in a map nobody
     * could see. */
    memset(s->occ, 0, n);
    memset(s->lik, 0, n);

    s->x = s->y = s->th = 0.0f;
    /* The prior is a movement not yet accounted for, measured between two poses
     * that have both just ceased to mean anything. */
    s->prior_fwd = s->prior_yaw = 0.0f;
    s->score = 0.0f;
    s->rejected = 0;
    /* The last match described a room that no longer exists. */
    s->edge = 0;
    s->ambiguity = 0.0f;
    s->ang_bins = 0;
    s->recover = 0;
    /* A map somebody has just asked to be rebuilt is by definition one they want
     * written, and leaving mapping suspended here would hand back an empty grid
     * that stayed empty however long the rover stood in the room. */
    s->mapping = 1;
    /* Back to unseeded so the next update takes its first-scan branch and stamps
     * the pending revolution straight in. It has to: there is nothing to match
     * against, and matching an empty field would score zero and reject every scan
     * that followed, leaving the rover dead-reckoning inside a map that never got
     * written. */
    s->seeded = 0;
    s->scans = 0;
}

/* ----------------------------------------------------------------- parsing */

static void finish_revolution(slam2d *s)
{
    /* Hand the accumulated points over as the pending scan, decimating evenly if
     * there are more than the caller wants to pay for. Anything still pending is
     * overwritten: a consumer that fell behind wants the newest scan, not a queue
     * of stale ones. */
    point *tmp = s->pend;
    s->pend = s->acc;
    s->acc  = tmp;

    int n = s->acc_n;
    if (n > s->cfg.max_points) {
        for (int j = 0; j < s->cfg.max_points; j++)
            s->pend[j] = s->pend[(int)((long)j * n / s->cfg.max_points)];
        n = s->cfg.max_points;
    }
    s->pend_n = n;
    s->pend_ready = 1;
    s->acc_n = 0;
}

static void add_point(slam2d *s, int angle_centi, int dist_mm)
{
    float r = dist_mm * 0.001f;
    if (r < s->cfg.min_range_m || r > s->cfg.max_range_m) return;
    if (s->acc_n >= MAX_POINTS) return;

    int idx = (angle_centi / (36000 / LUT_BINS)) % LUT_BINS;
    float px = r * s->lut_cos[idx];
    float py = r * s->lut_sin[idx];

    /* The rover, seen by its own sensor. Two posts behind the lidar came back at 12
     * to 16 cm in most revolutions, and because they move with the rover they were
     * being taken for the nearest obstacle -- which held every turn down to the slow
     * rate and, worse, was stamped into the grid at each new pose, painting a trail
     * of obstacles down the middle of the map as the rover drove.
     *
     * Discarded here, at the one place a return enters, so that the matcher, the map,
     * the sector query and the feature segmentation all agree about what is real.
     * Doing it further out would have left the map corrupted, and the map is the part
     * that does not recover.
     *
     * Only behind. A return this close in front is something the rover is about to
     * hit, and the standoff exists to act on exactly that. */
    if (s->cfg.body_back_m > 0.0f && s->cfg.body_half_width_m > 0.0f
        && px < 0.0f && px > -s->cfg.body_back_m
        && fabsf(py) < s->cfg.body_half_width_m) return;

    point *p = &s->acc[s->acc_n++];
    p->range = r;
    p->x = px;
    p->y = py;
    /* Rover-frame bearing, kept as an integer so the sector query never needs an
     * atan2 -- 420 of those a revolution is a millisecond this host cannot spare. */
    int b = (int)(s->cfg.mount_deg * 100.0f) - angle_centi;
    b %= 36000;
    if (b < 0) b += 36000;
    p->bearing = (uint16_t)b;
}

int slam2d_feed_lidar(slam2d *s, const unsigned char *buf, int n)
{
    if (!s || !buf || n <= 0) return 0;

    if (s->rx_len + n > RX_CAP) {
        /* More than a revolution has queued up behind us without a single valid
         * packet coming out, so sync is gone rather than merely late. Start over
         * from the newest bytes instead of parsing a stale backlog. */
        parser_resync(s);
        if (n > RX_CAP) { buf += n - RX_CAP; n = RX_CAP; }
    }
    memcpy(s->rx + s->rx_len, buf, (size_t)n);
    s->rx_len += n;

    int revolutions = 0, i = 0;
    while (i + PACKET_LEN <= s->rx_len) {
        const unsigned char *q = s->rx + i;
        if (q[0] != 0x54 || q[1] != 0x2C) { i++; continue; }

        uint8_t c = 0;
        for (int k = 0; k < PACKET_LEN - 1; k++) c = s->crc_tab[c ^ q[k]];
        if (c != q[PACKET_LEN - 1]) { i++; continue; }

        int start = q[4] | q[5] << 8;
        int end   = q[42] | q[43] << 8;

        /* The start angle climbs monotonically through a revolution and then wraps,
         * so a start below the previous one is the revolution boundary. The first
         * wrap after we start listening is almost always a remnant -- the port
         * opens onto a spinning sensor -- and stamping that as the seed is how a
         * restart left a wedge of map that later full scans would not match, so
         * mapping froze until someone cleared it. A wrap that has not covered
         * 270 degrees is discarded; the next one is a real revolution. */
        if (s->prev_start >= 0 && start < s->prev_start) {
            int rev_span = (s->acc_from >= 0)
                ? (s->prev_start - s->acc_from + 36000) % 36000
                : 0;
            if (rev_span >= 27000 && s->acc_n > 0) {
                finish_revolution(s);
                revolutions++;
            } else {
                s->acc_n = 0;
            }
            s->acc_from = start;
        }
        if (s->prev_start < 0)
            s->acc_from = start;
        s->prev_start = start;

        int span = (end - start + 36000) % 36000;
        for (int k = 0; k < PKT_POINTS; k++) {
            int dist = q[6 + k * 3] | q[7 + k * 3] << 8;
            if (!dist) continue;                       /* no return, not a hit at 0 */
            add_point(s, (start + span * k / (PKT_POINTS - 1)) % 36000, dist);
        }
        i += PACKET_LEN;
    }

    s->rx_len -= i;
    if (s->rx_len > 0) memmove(s->rx, s->rx + i, (size_t)s->rx_len);
    return revolutions;
}

/* ------------------------------------------------------------ scan matching */

/* Sum the likelihood field under the scan, with the points already rotated into
 * world bearings *and converted to grid units* by rotate_scan, so the candidate
 * translation (ox, oy metres) folds to one add per axis here. This is the hot
 * loop: at the defaults it runs 5 x 5 x 7 coarse plus 5 x 5 x 5 fine times a
 * revolution over every point, so everything that can be hoisted out has been --
 * the metres-to-cells multiply used to be in here, once per point per pose, and
 * moving it into the once-per-angle rotation measured 12% off the whole match
 * (19.7 -> 17.3 ms in selftest on the rover's Pi). */
static long score_pose(const slam2d *s, float ox, float oy)
{
    const int cells = s->cells;
    const float fcells = (float)cells;
    const float dx = ox * s->inv_res, dy = oy * s->inv_res;
    const unsigned char *lik = s->lik;
    const float *rx = s->rot_x, *ry = s->rot_y;
    long sum = 0;

    for (int i = 0; i < s->pend_n; i++) {
        /* Already biased by half the map (in rotate_scan), so the conversion
         * floors correctly either side of the origin instead of folding -0.5 and
         * +0.5 into the same cell; the bounds test is on the float for the same
         * reason. */
        float fx = rx[i] + dx;
        float fy = ry[i] + dy;
        if (fx < 0.0f || fx >= fcells || fy < 0.0f || fy >= fcells) continue;
        sum += lik[(int)fx * cells + (int)fy];
    }
    return sum;
}

static void rotate_scan(slam2d *s, float th)
{
    /* Rotated straight into grid units -- cells, biased by half the map -- so the
     * translation search above never multiplies. The rotation itself absorbs the
     * scaling for free: it is the same multiply-add either way. */
    const float ct = cosf(th) * s->inv_res, st = sinf(th) * s->inv_res;
    const float half = (float)(s->cells / 2);
    for (int i = 0; i < s->pend_n; i++) {
        float px = s->pend[i].x, py = s->pend[i].y;
        s->rot_x[i] = px * ct - py * st + half;
        s->rot_y[i] = px * st + py * ct + half;
    }
}

/* Search a lattice around (cx, cy, cth), leaving the best pose in place.
 *
 * Two things come out besides the pose, and both exist because the score does not
 * say how the match was won. `edge` reports that the winner sat on the rim of the
 * lattice, which means the answer was most likely outside it and what came back is
 * the boundary of what was searched rather than a fit. `profile`, when given, gets
 * the best score found at each candidate heading -- the correlation curve against
 * rotation, which is what tells a window too narrow apart from a room with two
 * answers in it.
 *
 * Ties now go to the centre, and the centre is the prior. The previous version
 * started from -1 and kept the first candidate evaluated, so anywhere every
 * candidate scores the same -- ground the map has never seen, where they are all
 * zero -- it returned a corner of its own lattice rather than the pose it was
 * handed, and would now report that corner as having hit the edge.
 */
static long match_pass(slam2d *s, float *cx, float *cy, float *cth,
                       float lin, int lin_steps, float ang_deg, int ang_steps,
                       int *edge, long *profile)
{
    const float centre_th = *cth;
    float bx = *cx, by = *cy, bth = centre_th;
    int bu = 0, bv = 0, ba = 0;

    rotate_scan(s, centre_th);
    long best = score_pose(s, *cx, *cy);

    for (int a = -ang_steps; a <= ang_steps; a++) {
        float th = centre_th + a * ang_deg * (float)(M_PI / 180.0);
        rotate_scan(s, th);                 /* hoisted: once per angle, not per pose */
        long top = -1;
        for (int u = -lin_steps; u <= lin_steps; u++)
            for (int v = -lin_steps; v <= lin_steps; v++) {
                float ox = *cx + u * lin, oy = *cy + v * lin;
                long sc = score_pose(s, ox, oy);
                if (sc > top) top = sc;
                if (sc > best) {
                    best = sc; bx = ox; by = oy; bth = th;
                    bu = u; bv = v; ba = a;
                }
            }
        if (profile) profile[a + ang_steps] = top;
    }
    *cx = bx; *cy = by; *cth = bth;
    if (edge)
        *edge = ((lin_steps > 0 && (abs(bu) == lin_steps || abs(bv) == lin_steps))
                 || (ang_steps > 0 && abs(ba) == ang_steps));
    return best;
}

/* The best peak in the heading profile far enough from the winner to be a rival
 * answer rather than the shoulder of the same one, as a fraction of the winner.
 *
 * Zero when the pass was never wide enough to hold anything that far out, which is
 * every ordinary tracking revolution -- the number is worth reading after a
 * recovery search and nowhere else. */
static float rival_ratio(const slam2d *s, float ang_deg)
{
    if (s->ang_bins < 3 || ang_deg <= 0.0f) return 0.0f;

    int sep = (int)(s->cfg.ambiguity_sep_deg / ang_deg + 0.999f);
    if (sep < 1) sep = 1;

    int win = 0;
    for (int i = 1; i < s->ang_bins; i++)
        if (s->ang_best[i] > s->ang_best[win]) win = i;
    long top = s->ang_best[win];
    if (top <= 0) return 0.0f;

    long rival = 0;
    for (int i = 0; i < s->ang_bins; i++)
        if (abs(i - win) >= sep && s->ang_best[i] > rival) rival = s->ang_best[i];
    return (float)rival / (float)top;
}

/* ---------------------------------------------------------------- mapping */

static void stamp_hit(slam2d *s, int ix, int iy)
{
    const int cells = s->cells;
    signed char *o = &s->occ[ix * cells + iy];
    int v = *o + s->cfg.hit_inc;
    *o = (signed char)(v > 127 ? 127 : v);

    /* Max, not add: the field is a likelihood, so seeing the same wall on twenty
     * consecutive revolutions should not make it twenty times more attractive than
     * a wall seen once. Decay on the clearing pass is what brings it back down. */
    for (int dx = -KERN; dx <= KERN; dx++) {
        int jx = ix + dx;
        if (jx < 0 || jx >= cells) continue;
        for (int dy = -KERN; dy <= KERN; dy++) {
            int jy = iy + dy;
            if (jy < 0 || jy >= cells) continue;
            unsigned char w = s->kernel[dx + KERN][dy + KERN];
            unsigned char *l = &s->lik[jx * cells + jy];
            if (w > *l) *l = w;
        }
    }
}

static void integrate(slam2d *s)
{
    const int cells = s->cells;
    const float inv = s->inv_res, half = (float)(cells / 2);
    float ct = cosf(s->th), st = sinf(s->th);

    for (int i = 0; i < s->pend_n; i++) {
        float px = s->pend[i].x, py = s->pend[i].y;
        float hx = s->x + px * ct - py * st;
        float hy = s->y + px * st + py * ct;

        /* Walk the beam a cell at a time and clear what it passed through, stopping
         * one step short so the clearing pass never fights the hit it belongs to.
         * Step count scales with range, unlike a fixed sample count, which would
         * leave gaps in cleared space at 8 m and waste work at 30 cm. */
        float dx = hx - s->x, dy = hy - s->y;
        int steps = (int)(sqrtf(dx * dx + dy * dy) * inv);
        if (steps > 1) {
            float sx = dx / steps, sy = dy / steps;
            for (int k = 0; k < steps - 1; k++) {
                float fx = (s->x + sx * k) * inv + half;
                float fy = (s->y + sy * k) * inv + half;
                if (fx < 0.0f || fx >= cells || fy < 0.0f || fy >= cells) continue;
                int idx = (int)fx * cells + (int)fy;
                int v = s->occ[idx] - s->cfg.miss_dec;
                s->occ[idx] = (signed char)(v < -127 ? -127 : v);
                int l = s->lik[idx] - s->cfg.lik_decay;
                s->lik[idx] = (unsigned char)(l < 0 ? 0 : l);
            }
        }

        float fx = hx * inv + half, fy = hy * inv + half;
        if (fx < 0.0f || fx >= cells || fy < 0.0f || fy >= cells) continue;
        stamp_hit(s, (int)fx, (int)fy);
    }
}

/* ----------------------------------------------------------------- driving */

void slam2d_set_prior(slam2d *s, float d_forward_m, float d_yaw_rad)
{
    if (!s) return;
    s->prior_fwd = d_forward_m;
    s->prior_yaw = d_yaw_rad;
}

void slam2d_set_mapping(slam2d *s, int on)
{
    if (s) s->mapping = on ? 1 : 0;
}

void slam2d_request_recovery(slam2d *s)
{
    if (s) s->recover = 1;
}

int slam2d_update(slam2d *s)
{
    if (!s || !s->pend_ready) return 0;
    s->pend_ready = 0;

    const int recovering = s->recover;
    s->recover = 0;

    /* Apply the prior first: drive forward along the heading we had, then turn.
     * This is only the centre of the search window, so its errors are corrected by
     * the match rather than accumulated -- until the match is rejected. */
    float px = s->x + s->prior_fwd * cosf(s->th);
    float py = s->y + s->prior_fwd * sinf(s->th);
    float pth = s->th + s->prior_yaw;
    s->prior_fwd = s->prior_yaw = 0.0f;

    if (!s->seeded || s->pend_n == 0) {
        /* Nothing to match against until a scan has been written: the map is where
         * the first scan is put, and the pose it defines is the origin by
         * definition. Keyed on the map having been seeded rather than on the scan
         * count, because mapping can be suspended -- and counting a suspended
         * revolution as the first one would leave every later scan matching against
         * an empty field, scoring zero and being rejected for ever. */
        s->x = px; s->y = py; s->th = pth;
        s->score = 0.0f;
        s->rejected = 0;
        s->edge = 0;
        s->ambiguity = 0.0f;
        s->ang_bins = 0;
    } else {
        /* One window for tracking, a much wider one for re-finding a pose the
         * caller has moved itself. See the recovery block in slam2d.h. */
        const float lin   = recovering ? s->cfg.recover_lin_m     : s->cfg.coarse_lin_m;
        const int   lsteps= recovering ? s->cfg.recover_lin_steps : s->cfg.coarse_lin_steps;
        const float ang   = recovering ? s->cfg.recover_ang_deg   : s->cfg.coarse_ang_deg;
        const int   asteps= recovering ? s->cfg.recover_ang_steps : s->cfg.coarse_ang_steps;

        float mx = px, my = py, mth = pth;
        match_pass(s, &mx, &my, &mth, lin, lsteps, ang, asteps,
                   &s->edge, s->ang_best);
        s->ang_bins = 2 * asteps + 1;
        for (int i = 0; i < s->ang_bins; i++)
            s->ang_off[i] = (float)(i - asteps) * ang;

        long best = match_pass(s, &mx, &my, &mth,
                               s->cfg.fine_lin_m, s->cfg.fine_lin_steps,
                               s->cfg.fine_ang_deg, s->cfg.fine_ang_steps,
                               NULL, NULL);

        float peak = (float)s->pend_n * s->cfg.lik_stamp;
        s->score = peak > 0.0f ? (float)best / peak : 0.0f;
        s->ambiguity = rival_ratio(s, ang);
        s->rejected = s->score < s->cfg.min_match_score;
        if (s->rejected) {
            /* Believe the prior instead. Dead reckoning drifts; a bad match can
             * teleport the rover and corrupt the map it then matches against. */
            s->x = px; s->y = py; s->th = pth;
        } else {
            s->x = mx; s->y = my; s->th = mth;
        }
    }

    /* Keep the heading in (-pi, pi] so it stays comparable and printable. */
    while (s->th > (float)M_PI)  s->th -= (float)(2.0 * M_PI);
    while (s->th <= -(float)M_PI) s->th += (float)(2.0 * M_PI);

    /* Written only from a pose this code is prepared to defend. Believe and write
     * are different: a weak or edge-of-window match may still be a better pose than
     * the prior, but stamping it would put a wall in the likelihood field at full
     * strength, as attractive to the next revolution as one seen all afternoon, and
     * from then on the wrong answer has evidence for it. The first scan has nothing
     * to match against and is the map by definition, so it is exempt -- but a
     * revolution with no returns in range is not that scan. Letting it stand as the
     * seed marks an empty map as seeded, and every later scan then matches an empty
     * field, scores zero and is rejected, so nothing is ever written. Suspended
     * mapping is the same argument made by the caller, which knows things this does
     * not: that it has just dead-reckoned a turn and cannot yet vouch for the pose. */
    int write = s->mapping && !s->rejected && s->pend_n > 0;
    if (s->seeded)
        write = write && !s->edge && s->score >= s->cfg.min_write_score;
    if (write) {
        integrate(s);
        s->seeded = 1;
    }
    s->scans++;
    return 1;
}

/* ----------------------------------------------------------------- queries */

void slam2d_pose(const slam2d *s, float *x, float *y, float *theta)
{
    if (!s) return;
    if (x) *x = s->x;
    if (y) *y = s->y;
    if (theta) *theta = s->th;
}

void slam2d_set_pose(slam2d *s, float x, float y, float theta)
{
    if (!s) return;
    s->x = x; s->y = y; s->th = theta;
}

float slam2d_score(const slam2d *s)    { return s ? s->score : 0.0f; }
int   slam2d_rejected(const slam2d *s) { return s ? s->rejected : 0; }
int   slam2d_scans(const slam2d *s)    { return s ? s->scans : 0; }
int   slam2d_points(const slam2d *s)   { return s ? s->pend_n : 0; }
int   slam2d_match_edge(const slam2d *s) { return s ? s->edge : 0; }
float slam2d_ambiguity(const slam2d *s)  { return s ? s->ambiguity : 0.0f; }
int   slam2d_mapping(const slam2d *s)    { return s ? s->mapping : 0; }

int slam2d_angle_profile(const slam2d *s, float *offsets, float *scores, int max_n)
{
    if (!s || max_n < 1) return 0;
    int n = s->ang_bins < max_n ? s->ang_bins : max_n;
    /* Normalised against the same peak the score is, so a value here and the score
     * are the same measurement and can be compared to each other. */
    float peak = (float)s->pend_n * (float)s->cfg.lik_stamp;
    for (int i = 0; i < n; i++) {
        if (offsets) offsets[i] = s->ang_off[i];
        if (scores)  scores[i]  = peak > 0.0f ? (float)s->ang_best[i] / peak : 0.0f;
    }
    return n;
}

void slam2d_sectors(const slam2d *s, float *out, int n_sectors)
{
    if (!s || !out || n_sectors < 1) return;
    /* -1 is "nothing came back from this direction", which is not the same claim as
     * "it is clear out to max_range_m" -- see the header. */
    for (int i = 0; i < n_sectors; i++) out[i] = -1.0f;
    for (int i = 0; i < s->pend_n; i++) {
        /* Half a sector of bias puts sector 0 astride straight ahead rather than
         * starting at it, so "is the way forward clear" is one lookup. */
        int k = ((int)s->pend[i].bearing * n_sectors + 18000) / 36000 % n_sectors;
        if (out[k] < 0.0f || s->pend[i].range < out[k]) out[k] = s->pend[i].range;
    }
}

float slam2d_arc_clearance(const slam2d *s, float curvature, float half_width_m,
                           float max_dist_m)
{
    if (!s || max_dist_m <= 0.0f) return 0.0f;
    if (half_width_m <= 0.0f) half_width_m = s->cfg.rover_width_m * 0.5f;

    float best = max_dist_m;

    /* Straight enough that the arc maths would divide by nearly zero: a radius over
     * 200 m across a corridor this short is a straight line by any measure. */
    if (fabsf(curvature) < 0.005f) {
        for (int i = 0; i < s->pend_n; i++) {
            float px = s->pend[i].x, py = s->pend[i].y;
            if (px <= 0.0f || fabsf(py) > half_width_m) continue;
            if (px < best) best = px;
        }
        return best;
    }

    /* Turning. The rover follows a circle through the origin, tangent to its own
     * forward axis, with the centre abeam at (0, R). Work the whole thing in the
     * left-turning case and mirror y for a right turn, so there is one set of signs
     * to get right rather than two. */
    float R = 1.0f / curvature;
    float flip = R < 0.0f ? -1.0f : 1.0f;
    R *= flip;

    for (int i = 0; i < s->pend_n; i++) {
        float px = s->pend[i].x, py = s->pend[i].y * flip;
        /* Radial distance from the turn centre; the corridor is an annulus. */
        float dy = R - py;
        float d = sqrtf(px * px + dy * dy);
        if (fabsf(d - R) > half_width_m) continue;
        /* Swept angle to reach it. Behind the rover is negative and irrelevant, and
         * a whole half turn ahead is further than any corridor this short cares
         * about. */
        float theta = atan2f(px, dy);
        if (theta <= 0.0f) continue;
        float along = R * theta;
        if (along < best) best = along;
    }
    return best;
}

/* --------------------------------------------------------------- features */

static int by_bearing(const void *a, const void *b)
{
    uint16_t x = ((const point *)a)->bearing, y = ((const point *)b)->bearing;
    return x < y ? -1 : (x > y ? 1 : 0);
}

/* Perpendicular distance from p to the line through a and b. */
static float deviation(const point *p, const point *a, const point *b)
{
    float ex = b->x - a->x, ey = b->y - a->y;
    float len = sqrtf(ex * ex + ey * ey);
    if (len < 1e-6f) return hypotf(p->x - a->x, p->y - a->y);
    return fabsf((p->x - a->x) * ey - (p->y - a->y) * ex) / len;
}

#define FEAT_RANGE_JUMP   0.10f   /* metres, or 10% of range, whichever is larger:
                                   * a step this size across neighbouring bearings is
                                   * an edge, not a surface */
#define FEAT_ANGLE_GAP    4.0f    /* degrees. ~5x the sensor's 0.86 deg spacing, so a
                                   * few dropped returns do not split a wall */
#define FEAT_FLATNESS     0.05f   /* metres of deviation that makes it a corner */
#define FEAT_WALL_WIDTH   0.50f   /* a flat run this long is furniture or building */
#define FEAT_MIN_POINTS   3
#define FEAT_MAX_SEGMENTS 64
#define FEAT_CORNER_SPAN  4       /* points either side, for measuring a corner */

/* Whether a run of points is a thing standing in front of something else, rather
 * than a piece of whatever is behind it.
 *
 * A free-standing object has background on both sides: look just past either end and
 * the sensor should be seeing something further away. A sliver of wall left visible
 * between two shadows fails that -- the occluder beside it is *nearer* -- and by
 * geometry alone it is otherwise indistinguishable from a small object. Without this
 * test a table's legs get reported along with a phantom object for every gap they
 * cast on the wall behind. */
static int stands_clear(const point *pts, int n, int lo, int hi, float far_range)
{
    if (n < 3) return 0;
    const point *before = &pts[(lo - 1 + n) % n];
    const point *after  = &pts[(hi + 1) % n];
    const float margin = 0.15f;
    return before->range > far_range + margin && after->range > far_range + margin;
}

/* `may_be_object` is false for the pieces of a cluster that had to be cut at a
 * corner. The two or three points left either side of a corner are geometrically
 * indistinguishable from a small isolated object and are not one, so a piece that
 * does not qualify as a wall in its own right is dropped rather than promoted. */
static void emit_segment(const point *pts, int npts, int lo, int hi,
                         int may_be_object,
                         slam2d_feature *out, int *n, int max_out)
{
    if (*n >= max_out || hi - lo + 1 < FEAT_MIN_POINTS) return;

    float worst = 0.0f, near = 1e9f, far = 0.0f;
    for (int i = lo; i <= hi; i++) {
        float dev = deviation(&pts[i], &pts[lo], &pts[hi]);
        if (dev > worst) worst = dev;
        if (pts[i].range < near) near = pts[i].range;
        if (pts[i].range > far) far = pts[i].range;
    }
    float width = hypotf(pts[hi].x - pts[lo].x, pts[hi].y - pts[lo].y);
    int is_wall = (width >= FEAT_WALL_WIDTH && worst <= FEAT_FLATNESS);
    if (!is_wall && !(may_be_object && stands_clear(pts, npts, lo, hi, far))) return;

    /* Bearing of the middle of the run, taken from the point rather than averaged,
     * because averaging bearings across the -180/180 seam is a trap. */
    const point *mid = &pts[(lo + hi) / 2];
    float span = ((int)pts[hi].bearing - (int)pts[lo].bearing) / 100.0f;
    if (span < 0.0f) span += 360.0f;

    slam2d_feature *f = &out[(*n)++];
    f->kind = is_wall ? SLAM2D_WALL : SLAM2D_OBJECT;
    f->bearing_deg = mid->bearing > 18000 ? mid->bearing / 100.0f - 360.0f
                                          : mid->bearing / 100.0f;
    f->range_m = near;
    f->width_m = width;
    f->span_deg = span;
    f->straightness_m = worst;
    f->x0 = pts[lo].x; f->y0 = pts[lo].y;
    f->x1 = pts[hi].x; f->y1 = pts[hi].y;
}

/* Iterative end-point fit: fit the chord, split at the worst outlier, repeat. This
 * is what turns one continuous ring of returns from inside a rectangular room into
 * four walls -- clustering alone never breaks at a corner, because the range varies
 * perfectly smoothly around one. */
static int worst_outlier(const point *pts, int a, int b, float *worst_out)
{
    int worst_i = -1;
    float worst = FEAT_FLATNESS;
    for (int i = a + 1; i < b; i++) {
        float dev = deviation(&pts[i], &pts[a], &pts[b]);
        if (dev > worst) { worst = dev; worst_i = i; }
    }
    if (worst_out) *worst_out = worst;
    return worst_i;
}

static void split_and_emit(const point *pts, int npts, int lo, int hi,
                           slam2d_feature *out, int *n, int max_out)
{
    /* One flat run already: it is a whole thing, and so allowed to be an object. */
    if (worst_outlier(pts, lo, hi, NULL) < 0) {
        emit_segment(pts, npts, lo, hi, 1, out, n, max_out);
        return;
    }

    int stack[2 * FEAT_MAX_SEGMENTS][2];
    int top = 0;
    stack[top][0] = lo; stack[top][1] = hi; top++;

    while (top > 0 && *n < max_out) {
        int a = stack[--top][0], b = stack[top][1];
        if (b - a + 1 < FEAT_MIN_POINTS) continue;

        int worst_i = worst_outlier(pts, a, b, NULL);
        if (worst_i < 0 || top + 2 > (int)(sizeof stack / sizeof stack[0])) {
            emit_segment(pts, npts, a, b, 0, out, n, max_out);
        } else {
            stack[top][0] = worst_i; stack[top][1] = b; top++;
            stack[top][0] = a; stack[top][1] = worst_i; top++;
        }
    }
}

/* Where to break the circle of bearings open into a line.
 *
 * This matters more than it sounds. Sorting by bearing puts straight-ahead at both
 * ends of the array, so whatever surface happens to lie dead ahead gets reported as
 * two separate features unless the cut is moved somewhere harmless. */
static int find_cut(const point *p, int n)
{
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        float dr = fabsf(p[j].range - p[i].range);
        float tol = FEAT_RANGE_JUMP;
        float scaled = p[i].range * 0.10f;
        if (scaled > tol) tol = scaled;
        float db = ((int)p[j].bearing - (int)p[i].bearing) / 100.0f;
        if (db < 0.0f) db += 360.0f;
        /* A real discontinuity: whatever follows it is a different surface anyway,
         * so cutting here costs nothing. */
        if (dr > tol || db > FEAT_ANGLE_GAP) return j;
    }

    /* No discontinuity anywhere, so the rover is enclosed and the returns form one
     * unbroken ring. Cut at the sharpest corner, which is a boundary the geometry
     * agrees with -- an arbitrary index would split a wall in half. */
    const int k = FEAT_CORNER_SPAN;
    if (n < 3 * k) return 0;
    int best = 0;
    float sharpest = -1.0f;
    for (int i = 0; i < n; i++) {
        const point *a = &p[(i - k + n) % n], *b = &p[i], *c = &p[(i + k) % n];
        float ux = b->x - a->x, uy = b->y - a->y;
        float vx = c->x - b->x, vy = c->y - b->y;
        float lu = hypotf(ux, uy), lv = hypotf(vx, vy);
        if (lu < 1e-6f || lv < 1e-6f) continue;
        /* |sin| of the turn between the two chords, from the cross product: 0 along
         * a flat wall, 1 at a right angle. No acos needed -- it only has to be
         * monotonic to find the maximum. */
        float turn = fabsf(ux * vy - uy * vx) / (lu * lv);
        if (turn > sharpest) { sharpest = turn; best = i; }
    }
    return best;
}

static void reverse_range(point *p, int a, int b)
{
    while (a < b) { point t = p[a]; p[a++] = p[b]; p[b--] = t; }
}

static void rotate_left(point *p, int n, int by)
{
    if (by <= 0 || by >= n) return;
    /* Three reversals, so no second buffer: a scan can be 2048 points and this runs
     * on a host with 474 MB and one core. */
    reverse_range(p, 0, by - 1);
    reverse_range(p, by, n - 1);
    reverse_range(p, 0, n - 1);
}

int slam2d_features(slam2d *s, slam2d_feature *out, int max_out)
{
    if (!s || !out || max_out < 1 || s->pend_n < FEAT_MIN_POINTS) return 0;

    /* Sorted by bearing, because the points arrive in the sensor's order, which runs
     * the other way and wraps in the middle. Copied rather than sorted in place, so
     * this query cannot disturb the scan the matcher is still using. */
    point *sorted = s->sorted;
    int n = s->pend_n;
    memcpy(sorted, s->pend, (size_t)n * sizeof *sorted);
    qsort(sorted, (size_t)n, sizeof *sorted, by_bearing);
    /* Then move the seam somewhere it does no damage -- see find_cut. After this the
     * array is a line rather than a circle and the clustering below can be simple. */
    rotate_left(sorted, n, find_cut(sorted, n));

    int count = 0;
    int start = 0;
    for (int i = 1; i <= n && count < max_out; i++) {
        int cut = (i == n);
        if (!cut) {
            float dr = fabsf(sorted[i].range - sorted[i - 1].range);
            float tol = FEAT_RANGE_JUMP;
            float scaled = sorted[i - 1].range * 0.10f;
            if (scaled > tol) tol = scaled;
            float dbear = ((int)sorted[i].bearing - (int)sorted[i - 1].bearing) / 100.0f;
            cut = (dr > tol) || (dbear > FEAT_ANGLE_GAP);
        }
        if (cut) {
            split_and_emit(sorted, n, start, i - 1, out, &count, max_out);
            start = i;
        }
    }

    /* Openings, from the space between one cluster's far edge and the next one's
     * near edge. Only those the rover would actually fit through are worth saying
     * out loud, and the rover's own width is the only honest threshold for that. */
    float min_gap = s->cfg.rover_width_m;
    int walls = count;
    if (walls < 2) return count;
    for (int i = 0; i < walls && count < max_out; i++) {
        /* Features come out in increasing bearing order, so one segment's far end and
         * the next one's near end are the two sides of the same opening -- and across
         * the last-to-first pair, of the one at the cut. */
        const slam2d_feature *a = &out[i];
        const slam2d_feature *b = &out[(i + 1) % walls];
        float ax = a->x1, ay = a->y1, bx = b->x0, by = b->y0;
        float width = hypotf(bx - ax, by - ay);
        if (width < min_gap) continue;

        float mx = (ax + bx) * 0.5f, my = (ay + by) * 0.5f;
        slam2d_feature *g = &out[count++];
        g->kind = SLAM2D_GAP;
        g->bearing_deg = atan2f(my, mx) * (float)(180.0 / M_PI);
        /* How far to the mouth of it, not to the nearer wall's closest point. */
        g->range_m = hypotf(mx, my);
        g->width_m = width;
        g->span_deg = 0.0f;
        g->straightness_m = 0.0f;
        g->x0 = ax; g->y0 = ay; g->x1 = bx; g->y1 = by;
    }
    return count;
}

int slam2d_scan_xy(const slam2d *s, float *xy, int max_pts)
{
    if (!s || !xy || max_pts < 1) return 0;
    int n = s->pend_n < max_pts ? s->pend_n : max_pts;
    for (int i = 0; i < n; i++) {
        xy[2 * i]     = s->pend[i].x;
        xy[2 * i + 1] = s->pend[i].y;
    }
    return n;
}

const signed char *slam2d_grid(const slam2d *s) { return s ? s->occ : NULL; }

void slam2d_grid_origin(const slam2d *s, float *ox, float *oy)
{
    if (!s) return;
    float o = -(float)(s->cells / 2) * s->cfg.resolution_m;
    if (ox) *ox = o;
    if (oy) *oy = o;
}
