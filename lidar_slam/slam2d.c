/* slam2d -- see slam2d.h for what this is and why it is not Python.
 *
 * Two things happen here, once per revolution of the lidar: the raw byte stream
 * becomes a list of points in the rover's frame, and that list is segmented into
 * walls, free-standing objects and the gaps between them.
 *
 * There used to be a third -- a correlative scan matcher and an occupancy grid,
 * which is what the name is about. `slam_toolbox` does that job now, with the loop
 * closure this could never afford, so the matcher and the map were taken out
 * rather than left to rot beside code that no longer calls them. What is left is
 * the part slam_toolbox has no equivalent of: an LD19 parser fast enough for this
 * board, and the room described in words.
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

typedef struct {
    uint16_t bearing;               /* rover-frame, centi-degrees, ccw from forward */
    float    range;                 /* metres */
    float    x, y;                  /* rover frame, metres */
} point;

struct slam2d {
    slam2d_config cfg;

    uint8_t crc_tab[256];
    float   lut_cos[LUT_BINS], lut_sin[LUT_BINS];

    unsigned char rx[RX_CAP];
    int  rx_len;
    int  prev_start;                /* last packet's start angle, for wrap detection */
    int  acc_from;                  /* start angle of the first packet in acc, or -1 */

    point pts_a[MAX_POINTS], pts_b[MAX_POINTS];
    point *acc, *pend;              /* accumulating / complete */
    int    acc_n, pend_n;
    /* Bearing-sorted copy of the pending scan, for slam2d_features. Per instance
     * rather than a file static: the daemon calls tools on connection threads, and
     * two descriptions at once must not share a scratch buffer. The instance as a
     * whole still needs external locking -- see slam2d.py. */
    point sorted[MAX_POINTS];
};

/* ------------------------------------------------------------------ setup */

void slam2d_default_config(slam2d_config *cfg)
{
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
    /* Enough to keep every return the sensor gives, which is ~419 a revolution and
     * has been seen at 450.
     *
     * This was 300, and 300 was a budget rather than a measurement: every point
     * cost a cache miss in each of 300 candidate poses, so thinning the scan bought
     * 25 ms a revolution off the scan match. With the matcher gone the scan is no
     * longer an input to something expensive -- it *is* the output, published as
     * /scan for slam_toolbox and Nav2 -- and throwing away a third of it to save
     * arithmetic nobody does any more is simply a coarser sensor. */
    cfg->max_points   = 600;

    /* The UGV Rover is about 22 cm across the tracks; 0.34 leaves a hand's width
     * either side, which is also roughly the standoff worth planning gaps around.
     * Only the gap segmentation reads it now: a gap narrower than the rover is not
     * a way through and is not worth reporting as one. */
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

slam2d *slam2d_create(const slam2d_config *cfg)
{
    if (!cfg || cfg->max_range_m <= cfg->min_range_m) return NULL;

    slam2d *s = calloc(1, sizeof *s);
    if (!s) return NULL;
    s->cfg = *cfg;
    if (s->cfg.max_points < 1) s->cfg.max_points = 1;
    if (s->cfg.max_points > MAX_POINTS) s->cfg.max_points = MAX_POINTS;

    build_crc(s->crc_tab);
    build_lut(s);

    s->acc = s->pts_a;
    s->pend = s->pts_b;
    s->prev_start = -1;
    s->acc_from = -1;
    return s;
}

void slam2d_destroy(slam2d *s)
{
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

/* ----------------------------------------------------------------- queries */

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

