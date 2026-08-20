/* Correctness and cost of slam2d, against scans synthesised from a known room.
 *
 * The point of generating the packets rather than replaying a capture is that the
 * true pose is known, so "the pose tracks" is a measured error in millimetres
 * instead of a map that looks about right. Packets are built the way the sensor
 * builds them -- 47 bytes, real start/end angles, real CRC-8 -- so the parser is
 * under test too.
 *
 *   gcc -O2 -o selftest selftest.c slam2d.c -lm && ./selftest
 */
/* clock_gettime is POSIX, and build.sh compiles with -std=c99, which hides it
 * unless this is asked for -- before any header is pulled in. */
#define _POSIX_C_SOURCE 199309L

#include "slam2d.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_SQRT2
#define M_SQRT2 1.41421356237309504880
#endif

#define PTS_PER_REV 420             /* measured: ~419 on the real sensor */
#define PKT_POINTS  12
#define PKTS_PER_REV (PTS_PER_REV / PKT_POINTS)     /* 35 */
#define MOUNT_DEG   90.0f
/* Room for the widest angle profile the library will hand back. The library's own
 * ceiling is internal to slam2d.c, so this is that number restated where the test
 * needs it rather than exported for the sake of one caller. */
#define PROFILE_CAP 129

/* The room, axis-aligned, in metres. The rover starts at the origin facing +x, so
 * these numbers are also the distances the sector query should report. */
#define ROOM_XMIN (-2.0)
#define ROOM_XMAX ( 4.0)
#define ROOM_YMIN (-1.5)
#define ROOM_YMAX ( 1.5)

static int failures;

static void check(int ok, const char *what, double got, double want, double tol)
{
    printf("%-46s %9.4f (want %.4f +/- %.4f)  %s\n",
           what, got, want, tol, ok ? "ok" : "FAIL");
    if (!ok) failures++;
}

static void close_to(const char *what, double got, double want, double tol)
{
    check(fabs(got - want) <= tol, what, got, want, tol);
}

/* For claims that are not "this number is near that one". */
static void check_true(int ok, const char *what)
{
    printf("%-46s %41s\n", what, ok ? "ok" : "FAIL");
    if (!ok) failures++;
}

static double now_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1000.0 + t.tv_nsec / 1e6;
}

/* --------------------------------------------------------------- CRC and packing */

static uint8_t crc_tab[256];

static void build_crc(void)
{
    for (int i = 0; i < 256; i++) {
        int c = i;
        for (int k = 0; k < 8; k++)
            c = (c & 0x80) ? ((c << 1) ^ 0x4D) & 0xFF : (c << 1) & 0xFF;
        crc_tab[i] = (uint8_t)c;
    }
}

/* Round obstacles in the room, as centre and radius. Off by default; the table test
 * switches them on. Four legs of a 0.8 m table, 1.2 m ahead, 10 cm thick -- thick
 * because a 5 cm leg at that range subtends 2.4 degrees and the sensor's spacing is
 * 0.86, which is under the three points a feature needs. That limit is real and
 * belongs in the record: this rover cannot reliably see a slim chair leg across a
 * room, and a test built on one would only be measuring luck. */
static int obstacles_on;

/* When set, every packet parses and no point survives -- a covered sensor. The
 * parser still sees the start angles wrap, so revolutions complete as usual. */
static int no_returns;
static const double LEGS[][3] = {
    {1.2, -0.4, 0.05}, {1.2, 0.4, 0.05}, {2.0, -0.4, 0.05}, {2.0, 0.4, 0.05},
};
#define N_LEGS ((int)(sizeof LEGS / sizeof LEGS[0]))

/* Distance in metres from (x, y) to the nearest surface along world bearing `bear`. */
static double raycast(double x, double y, double bear)
{
    double dx = cos(bear), dy = sin(bear), best = 1e9, t;
    if (fabs(dx) > 1e-9) {
        t = ((dx > 0 ? ROOM_XMAX : ROOM_XMIN) - x) / dx;
        if (t > 0 && t < best) best = t;
    }
    if (fabs(dy) > 1e-9) {
        t = ((dy > 0 ? ROOM_YMAX : ROOM_YMIN) - y) / dy;
        if (t > 0 && t < best) best = t;
    }
    if (!obstacles_on) return best;

    for (int i = 0; i < N_LEGS; i++) {
        /* Ray-circle: |p + t d - c| = r, with d a unit vector so a = 1. */
        double fx = x - LEGS[i][0], fy = y - LEGS[i][1], r = LEGS[i][2];
        double b = 2.0 * (fx * dx + fy * dy);
        double c = fx * fx + fy * fy - r * r;
        double disc = b * b - 4.0 * c;
        if (disc < 0) continue;
        double root = sqrt(disc);
        for (int s = -1; s <= 1; s += 2) {
            t = (-b + s * root) / 2.0;
            if (t > 0 && t < best) best = t;
        }
    }
    return best;
}

/* One revolution of the rover at (x, y, th), as bytes the D500 would have sent. */
static int make_revolution(unsigned char *out, double x, double y, double th)
{
    int n = 0;
    for (int p = 0; p < PKTS_PER_REV; p++) {
        unsigned char *q = out + n;
        int angles[PKT_POINTS], dists[PKT_POINTS];

        for (int k = 0; k < PKT_POINTS; k++) {
            int i = p * PKT_POINTS + k;
            int b = (int)(i * 36000.0 / PTS_PER_REV + 0.5) % 36000;   /* sensor bearing */
            /* Invert what slam2d does: rover-frame bearing is mount - sensor, and
             * the world bearing adds the heading. */
            double phi = (MOUNT_DEG - b / 100.0) * M_PI / 180.0;
            angles[k] = b;
            dists[k]  = no_returns ? 0
                        : (int)(raycast(x, y, th + phi) * 1000.0 + 0.5);
        }

        memset(q, 0, 47);
        q[0] = 0x54; q[1] = 0x2C;
        q[2] = 0x60; q[3] = 0x0E;                       /* ~3680 deg/s, i.e. 10.2 Hz */
        q[4] = angles[0] & 0xFF;  q[5] = angles[0] >> 8;
        for (int k = 0; k < PKT_POINTS; k++) {
            q[6 + k * 3] = dists[k] & 0xFF;
            q[7 + k * 3] = dists[k] >> 8;
            q[8 + k * 3] = 200;                         /* intensity */
        }
        q[42] = angles[PKT_POINTS - 1] & 0xFF;
        q[43] = angles[PKT_POINTS - 1] >> 8;
        uint8_t c = 0;
        for (int k = 0; k < 46; k++) c = crc_tab[c ^ q[k]];
        q[46] = c;
        n += 47;
    }
    return n;
}

/* ------------------------------------------------------------------- the tests */

static unsigned char rev[PKTS_PER_REV * 47];

/* Feed one revolution and process whatever that completed. Returns 1 if a scan was
 * processed. Note the inherent one-revolution lag: the wrap that ends revolution N
 * is only seen when revolution N+1's first packet arrives. */
static int step(slam2d *s, double x, double y, double th)
{
    int n = make_revolution(rev, x, y, th);
    int revs = slam2d_feed_lidar(s, rev, n);
    return revs > 0 ? slam2d_update(s) : 0;
}

static void test_parser_and_room(void)
{
    puts("\n--- parser, frame convention and sectors ---");
    slam2d_config cfg;
    slam2d_default_config(&cfg);
    cfg.mount_deg = MOUNT_DEG;
    cfg.max_range_m = 12.0f;
    cfg.max_points = PTS_PER_REV + 50;     /* no decimation, so the parser is on trial */
    slam2d *s = slam2d_create(&cfg);

    step(s, 0, 0, 0);                      /* primes the wrap detector */
    step(s, 0, 0, 0);                      /* this one gets processed */

    check(slam2d_points(s) > 400, "points recovered from one revolution",
          slam2d_points(s), PTS_PER_REV, 20);

    /* If the handedness or the mount offset were wrong, these four would be a
     * rotation or a mirror of each other rather than the room's real dimensions.
     *
     * 72 sectors, not 4: the query returns the *nearest* return in each sector,
     * which is what an obstacle avoider wants, and 90-degree sectors would report
     * the near corner at 2.12 m for the forward one rather than the wall at 4 m.
     * That is correct behaviour and useless as a test of the frame. At 5 degrees
     * the sector astride each axis sees only that wall. */
    float sec[72];
    slam2d_sectors(s, sec, 72);
    close_to("sector 0, straight ahead (+x wall)",  sec[0],  ROOM_XMAX,  0.05);
    close_to("sector 18, rover's left (+y wall)",   sec[18], ROOM_YMAX,  0.05);
    close_to("sector 36, behind (-x wall)",         sec[36], -ROOM_XMIN, 0.05);
    close_to("sector 54, rover's right (-y wall)",  sec[54], -ROOM_YMIN, 0.05);

    /* And the corner really is where the nearest-return rule says it is, so the
     * wide-sector result above is understood rather than merely tolerated. */
    float quad[4];
    slam2d_sectors(s, quad, 4);
    close_to("4 sectors: forward one sees the corner", quad[0],
             ROOM_YMAX * M_SQRT2, 0.05);

    slam2d_destroy(s);
}

static void test_stationary(void)
{
    puts("\n--- stationary: 30 revolutions, no prior ---");
    slam2d_config cfg;
    slam2d_default_config(&cfg);
    cfg.mount_deg = MOUNT_DEG;
    slam2d *s = slam2d_create(&cfg);

    for (int i = 0; i < 30; i++) step(s, 0, 0, 0);

    float x, y, th;
    slam2d_pose(s, &x, &y, &th);
    /* Half a cell is the most the match can resolve, so that is the tolerance. */
    close_to("drift in x (m)",       x, 0.0, 0.025);
    close_to("drift in y (m)",       y, 0.0, 0.025);
    close_to("drift in heading (deg)", th * 180.0 / M_PI, 0.0, 1.0);
    check(slam2d_score(s) > 0.5, "match score", slam2d_score(s), 1.0, 0.5);
    check(!slam2d_rejected(s), "match accepted", slam2d_rejected(s), 0, 0);

    slam2d_destroy(s);
}

static void test_moving(void)
{
    puts("\n--- driving: 2 cm and 1.5 deg a revolution, 60 revolutions ---");
    slam2d_config cfg;
    slam2d_default_config(&cfg);
    cfg.mount_deg = MOUNT_DEG;
    slam2d *s = slam2d_create(&cfg);

    /* Two poses have to be kept apart. `t*` is where the revolution now being fed
     * was taken; `p*` is where the one that actually gets processed was taken. They
     * differ because the wrap that ends a revolution only arrives with the next
     * one's first packet, so every update is one revolution behind the feed --
     * which is a property of the sensor, not of this test, and the real driver in
     * run_slam.py sees the same lag. */
    double tx = 0, ty = 0, tth = 0;
    double px = 0, py = 0, pth = 0;
    double worst = 0, worst_th = 0;
    int rejects = 0, compared = 0;

    for (int i = 0; i < 60; i++) {
        int processed = step(s, tx, ty, tth);
        if (processed && i >= 2) {
            float x, y, th;
            slam2d_pose(s, &x, &y, &th);
            double e = hypot(x - px, y - py);
            if (e > worst) worst = e;
            double eth = fabs(th - pth) * 180.0 / M_PI;
            if (eth > worst_th) worst_th = eth;
            if (slam2d_rejected(s)) rejects++;
            compared++;
        }
        px = tx; py = ty; pth = tth;
        tth += 1.5 * M_PI / 180.0;
        tx  += 0.02 * cos(tth);
        ty  += 0.02 * sin(tth);
    }
    check_true(compared > 50, "revolutions actually compared");

    /* No prior was supplied, so every revolution's 2 cm and 1.5 deg had to be found
     * by the match alone -- which is the case that matters before calibration. */
    check(worst < 0.06, "worst position error (m)", worst, 0.0, 0.06);
    check(worst_th < 2.5, "worst heading error (deg)", worst_th, 0.0, 2.5);
    check(rejects == 0, "rejected matches", rejects, 0, 0);

    slam2d_destroy(s);
}

static void test_reset(void)
{
    puts("\n--- clearing the map: an empty grid, and a core that maps again ---");
    slam2d_config cfg;
    slam2d_default_config(&cfg);
    cfg.mount_deg = MOUNT_DEG;
    slam2d *s = slam2d_create(&cfg);
    const long n_cells = (long)cfg.grid_cells * cfg.grid_cells;
    const signed char *g = slam2d_grid(s);
    long written;
    float px, py, pth;

    /* Drive a metre and a half up the room, so what gets thrown away is a map
     * built over a run and a pose that has gone somewhere -- which is the state a
     * reset is actually asked for in. */
    double x = 0.0;
    for (int i = 0; i < 30; i++) { x += 0.05; step(s, x, 0.0, 0.0); }

    written = 0;
    for (long i = 0; i < n_cells; i++) if (g[i]) written++;
    check_true(written > 100, "there is a map to throw away");
    slam2d_pose(s, &px, &py, &pth);
    check_true(px > 1.0, "and the rover has driven away from the origin");

    slam2d_reset(s);

    slam2d_pose(s, &px, &py, &pth);
    close_to("x is back at the origin (m)", px, 0.0, 1e-6);
    close_to("y is back at the origin (m)", py, 0.0, 1e-6);
    close_to("heading is back at the origin (deg)", pth * 180.0 / M_PI, 0.0, 1e-6);
    check_true(slam2d_scans(s) == 0, "the scan count starts again");
    written = 0;
    for (long i = 0; i < n_cells; i++) if (g[i]) written++;
    check_true(written == 0, "every cell is empty");

    /* And it maps again from where it stands, which is the half a reset gets wrong
     * quietly: leave the scan count alone and the next revolution is matched
     * against an empty likelihood field, scores nothing, is rejected, and the map
     * is never written again. That looks exactly like a dead lidar and is not one. */
    for (int i = 0; i < 12; i++) step(s, x, 0.0, 0.0);
    written = 0;
    for (long i = 0; i < n_cells; i++) if (g[i]) written++;
    check_true(written > 100, "the new map is being written");
    check(slam2d_score(s) > 0.5, "and it is matching again", slam2d_score(s), 1.0, 0.5);
    check_true(!slam2d_rejected(s), "the match is accepted, not dead reckoned");
    slam2d_pose(s, &px, &py, &pth);
    close_to("standing still, it stays at the new origin (m)",
             hypot(px, py), 0.0, 0.05);

    slam2d_destroy(s);
}

static void test_prior_helps(void)
{
    const double STRIDE = 0.30;             /* per revolution, so 3 m/s */
    const int REVS = 12;                    /* -1.5 m to +1.8 m, all inside the room */

    puts("\n--- driving past the search window: 30 cm a revolution ---");
    /* The coarse window spans +/-0.15 m, so a 0.30 m stride is deliberately outside
     * anything the match can reach on its own. This is the one regime where the
     * motion prior stops being optional, and the contrast is the test. */
    double err[2] = {0, 0};
    for (int use_prior = 0; use_prior <= 1; use_prior++) {
        slam2d_config cfg;
        slam2d_default_config(&cfg);
        cfg.mount_deg = MOUNT_DEG;
        slam2d *s = slam2d_create(&cfg);

        /* Start at the origin, because slam2d's frame origin *is* wherever the
         * first processed scan was taken -- start the truth anywhere else and the
         * whole run carries that offset, which measures nothing. */
        double tx = 0, px = 0, worst = 0;
        for (int i = 0; i < REVS; i++) {
            /* The prior describes motion between the two revolutions being
             * matched, so there is none to declare before the first one is in. */
            if (use_prior && i >= 2) slam2d_set_prior(s, (float)STRIDE, 0.0f);
            int processed = step(s, tx, 0, 0);
            if (processed && i >= 2) {
                float x, y, th;
                slam2d_pose(s, &x, &y, &th);
                double e = fabs(x - px);
                if (e > worst) worst = e;
            }
            px = tx;
            tx += STRIDE;
        }
        err[use_prior] = worst;
        printf("  %-42s worst x error %.3f m\n",
               use_prior ? "with prior" : "without prior", worst);
        slam2d_destroy(s);
    }

    check(err[1] < 0.06, "worst x error with prior (m)", err[1], 0.0, 0.06);
    printf("  %-42s %.1fx\n", "ratio, without prior over with",
           err[1] > 1e-6 ? err[0] / err[1] : 999.0);
    check_true(err[0] > 4 * err[1] && err[0] > 0.15,
               "and it was the prior that made the difference");
}

static void test_unknown_sectors(void)
{
    puts("\n--- an empty sector reads unknown, not clear ---");
    slam2d_config cfg;
    slam2d_default_config(&cfg);
    cfg.mount_deg = MOUNT_DEG;
    /* 1.4 m cuts off the +x wall at 4 m and the corners, leaving the two side walls
     * at 1.5 m out of range too -- so most of the horizon returns nothing at all. */
    cfg.max_range_m = 1.4f;
    slam2d *s = slam2d_create(&cfg);
    step(s, 0, 0, 0);
    step(s, 0, 0, 0);

    float sec[36];
    slam2d_sectors(s, sec, 36);
    int unknown = 0;
    for (int i = 0; i < 36; i++) if (sec[i] < 0.0f) unknown++;
    check(unknown > 30, "sectors reporting unknown", unknown, 36, 6);
    /* The whole point: none of them claims to be clear out to max_range. */
    int claimed_clear = 0;
    for (int i = 0; i < 36; i++) if (sec[i] > 1.39f) claimed_clear++;
    check(claimed_clear == 0, "sectors falsely claiming clear", claimed_clear, 0, 0);
    slam2d_destroy(s);
}

/* One revolution with every return at the same range, whatever the bearing. Not a
 * room -- it is a ring, which is the shape that makes the body mask's geometry
 * readable: whatever survives did so because of where it was, not how far. */
static int make_ring_revolution(unsigned char *out, double range_m)
{
    int n = 0, mm = (int)(range_m * 1000.0 + 0.5);
    for (int p = 0; p < PKTS_PER_REV; p++) {
        unsigned char *q = out + n;
        int first = (int)(p * PKT_POINTS * 36000.0 / PTS_PER_REV + 0.5) % 36000;
        int last  = (int)(((p + 1) * PKT_POINTS - 1) * 36000.0 / PTS_PER_REV + 0.5) % 36000;
        memset(q, 0, 47);
        q[0] = 0x54; q[1] = 0x2C;
        q[2] = 0x60; q[3] = 0x0E;
        q[4] = first & 0xFF;  q[5] = first >> 8;
        for (int k = 0; k < PKT_POINTS; k++) {
            q[6 + k * 3] = mm & 0xFF;
            q[7 + k * 3] = mm >> 8;
            q[8 + k * 3] = 200;
        }
        q[42] = last & 0xFF;  q[43] = last >> 8;
        uint8_t c = 0;
        for (int k = 0; k < 46; k++) c = crc_tab[c ^ q[k]];
        q[46] = c;
        n += 47;
    }
    return n;
}

/* The rover's own mount posts, which it was reporting as the nearest obstacle in most
 * revolutions -- holding every turn down to the slow rate, and being stamped into the
 * grid at each new pose as the rover drove. They sit behind the lidar and inside the
 * chassis, so the mask is a box behind it; the property that has to hold is that it
 * takes the rear and leaves the front, because a return this close in front is
 * something the rover is about to hit. */
static void test_body_mask(void)
{
    puts("\n--- the rover's own body is not an obstacle ---");
    slam2d_config cfg;
    slam2d_default_config(&cfg);
    cfg.mount_deg = MOUNT_DEG;
    slam2d *s = slam2d_create(&cfg);

    /* At 13 cm -- what the posts actually measure -- every bearing is inside the
     * box's width and depth, so the whole rear half should go and the whole front
     * half should stay. */
    int n = make_ring_revolution(rev, 0.13);
    slam2d_feed_lidar(s, rev, n);
    n = make_ring_revolution(rev, 0.13);
    slam2d_feed_lidar(s, rev, n);
    slam2d_update(s);

    float sec[36];
    slam2d_sectors(s, sec, 36);
    /* Sector i spans bearings around i * 10 degrees, counter-clockwise from forward.
     * Rear is anything past 90 degrees either way: sectors 10..26. */
    int rear_seen = 0, front_seen = 0;
    for (int i = 0; i < 36; i++) {
        int deg = i * 10;
        if (deg > 180) deg -= 360;
        if (deg > 100 || deg < -100) { if (sec[i] > 0.0f) rear_seen++; }
        else if (deg < 80 && deg > -80) { if (sec[i] > 0.0f) front_seen++; }
    }
    check(rear_seen == 0, "rear sectors still reporting the body", rear_seen, 0, 0);
    check(front_seen >= 14, "front sectors kept", front_seen, 16, 2);

    /* And it is a box, not a blanket: the same ring further out is all real world. */
    slam2d_destroy(s);
    s = slam2d_create(&cfg);
    n = make_ring_revolution(rev, 0.60);
    slam2d_feed_lidar(s, rev, n);
    n = make_ring_revolution(rev, 0.60);
    slam2d_feed_lidar(s, rev, n);
    slam2d_update(s);
    slam2d_sectors(s, sec, 36);
    int kept = 0;
    for (int i = 0; i < 36; i++) if (sec[i] > 0.0f) kept++;
    check(kept >= 34, "sectors kept at 0.60 m, well outside the box", kept, 36, 2);
    slam2d_destroy(s);
}

static void test_arc_clearance(void)
{
    puts("\n--- swept-arc clearance ---");
    slam2d_config cfg;
    slam2d_default_config(&cfg);
    cfg.mount_deg = MOUNT_DEG;
    slam2d *s = slam2d_create(&cfg);
    /* Sat 0.30 m off the -x wall, facing +x: 3.70 m of room ahead, a wall right
     * behind, and the side walls 1.5 m away. */
    step(s, -1.7, 0, 0);
    step(s, -1.7, 0, 0);

    float half = cfg.rover_width_m * 0.5f;
    /* From x = -1.7 to the wall at x = 4.0 is 5.70 m, not 3.70 -- the rover is 0.30 m
     * off the *back* wall, which is behind it. */
    close_to("straight ahead, clear to the far wall",
             slam2d_arc_clearance(s, 0.0f, half, 8.0f), 5.70, 0.06);
    close_to("and the cap is honoured", slam2d_arc_clearance(s, 0.0f, half, 2.0f),
             2.00, 0.001);

    /* A 0.8 m radius turn to the left curves into the +y wall at 1.5 m. Reaching it
     * along the arc is further than the straight-line distance, which is the whole
     * reason this is not a radius check. */
    float left = slam2d_arc_clearance(s, 1.0f / 0.8f, half, 6.0f);
    check_true(left > 1.5 && left < 3.0, "0.8 m left turn reaches the side wall");
    printf("  %-42s %.2f m along the arc\n", "left turn clearance", left);

    /* Symmetry: the room is symmetric about y=0 and the rover is on that axis, so a
     * mirrored turn must give a mirrored answer. This is the check that catches a
     * dropped sign in the right-turn branch. */
    float right = slam2d_arc_clearance(s, -1.0f / 0.8f, half, 6.0f);
    close_to("and mirrors for the same turn to the right", right, left, 0.05);

    /* A tight enough turn stays inside the room and never leaves the corridor. */
    check_true(slam2d_arc_clearance(s, 1.0f / 0.35f, half, 6.0f) > 1.0,
               "a tight turn is not blocked by the wall behind");
    slam2d_destroy(s);
}

static void test_features(void)
{
    puts("\n--- segmenting the scan into features ---");
    slam2d_config cfg;
    slam2d_default_config(&cfg);
    cfg.mount_deg = MOUNT_DEG;
    cfg.max_range_m = 12.0f;
    slam2d *s = slam2d_create(&cfg);
    step(s, 0, 0, 0);
    step(s, 0, 0, 0);

    slam2d_feature f[32];
    int n = slam2d_features(s, f, 32);

    int walls = 0, objects = 0, gaps = 0;
    for (int i = 0; i < n; i++) {
        if (f[i].kind == SLAM2D_WALL) walls++;
        if (f[i].kind == SLAM2D_OBJECT) objects++;
        if (f[i].kind == SLAM2D_GAP) gaps++;
    }
    printf("  %d features: %d wall, %d object, %d gap\n", n, walls, objects, gaps);
    for (int i = 0; i < n && i < 8; i++)
        printf("    %-6s bearing %+7.1f deg  range %5.2f m  width %5.2f m  "
               "flatness %.3f m\n",
               f[i].kind == SLAM2D_WALL ? "wall" :
               f[i].kind == SLAM2D_OBJECT ? "object" : "gap",
               f[i].bearing_deg, f[i].range_m, f[i].width_m, f[i].straightness_m);

    /* The room is a closed rectangle, so the returns form one unbroken ring with no
     * range discontinuity anywhere -- clustering alone would call that a single
     * lumpy object. Getting four walls out is the corner splitting working. */
    check(walls == 4, "walls found in a rectangular room", walls, 4, 0);
    check(objects == 0, "spurious objects", objects, 0, 0);
    /* Adjacent walls meet at a corner, so there is nowhere to drive out of a sealed
     * room. A gap here would mean the seam had split a wall in two and the "opening"
     * was the crack between the halves. */
    check(gaps == 0, "spurious ways out of a sealed room", gaps, 0, 0);

    /* Each wall's nearest approach must be one of the room's four half-widths. */
    for (int i = 0; i < n; i++) {
        if (f[i].kind != SLAM2D_WALL) continue;
        double r = f[i].range_m;
        int plausible = fabs(r - 1.5) < 0.06 || fabs(r - 2.0) < 0.06 ||
                        fabs(r - 4.0) < 0.06;
        if (!plausible) {
            check_true(0, "a wall at an implausible range");
            printf("      offending range %.3f m\n", r);
        }
    }
    check_true(1, "every wall sits at one of the room's real distances");
    slam2d_destroy(s);
}

static void test_table(void)
{
    puts("\n--- a table in the room: what the rover can actually be told ---");
    obstacles_on = 1;
    slam2d_config cfg;
    slam2d_default_config(&cfg);
    cfg.mount_deg = MOUNT_DEG;
    cfg.max_range_m = 12.0f;
    slam2d *s = slam2d_create(&cfg);
    step(s, 0, 0, 0);
    step(s, 0, 0, 0);

    slam2d_feature f[48];
    int n = slam2d_features(s, f, 48);
    int walls = 0, objects = 0, gaps = 0;
    float widest_gap = 0.0f;
    for (int i = 0; i < n; i++) {
        if (f[i].kind == SLAM2D_WALL) walls++;
        if (f[i].kind == SLAM2D_OBJECT) objects++;
        if (f[i].kind == SLAM2D_GAP) {
            gaps++;
            if (f[i].width_m > widest_gap) widest_gap = f[i].width_m;
        }
    }
    for (int i = 0; i < n; i++)
        printf("    %-6s bearing %+7.1f deg  range %5.2f m  width %5.2f m\n",
               f[i].kind == SLAM2D_WALL ? "wall" :
               f[i].kind == SLAM2D_OBJECT ? "object" : "gap",
               f[i].bearing_deg, f[i].range_m, f[i].width_m);

    /* Four legs, and they are what the lidar reports -- never a table. Turning four
     * small objects in a square into the word "table" is the language model's job,
     * and it can only do it if it is handed objects rather than 37 ranges. */
    /* Three of the four, not four: the far pair sits at 2.04 m where a 10 cm leg
     * subtends 2.8 degrees against a 0.86 degree point spacing, so whether it lands
     * on three samples or two is down to where the sweep happens to fall. That is the
     * sensor's limit rather than the segmenter's, and the right thing for a test to
     * record is the limit, not a lucky run. */
    check_true(objects >= 3, "table legs seen as discrete objects");
    printf("  %-42s %d of %d\n", "legs found", objects, 4);
    check(walls == 4, "and the room's walls are still walls", walls, 4, 0);
    check_true(gaps >= 2, "openings reported around the table");
    check_true(widest_gap > cfg.rover_width_m,
               "at least one of them is wider than the rover");
    printf("  %-42s %.2f m (rover is %.2f m)\n", "widest opening",
           widest_gap, cfg.rover_width_m);

    /* Every leg is where it was put: 1.26 m at +/-18.4 deg, 2.04 m at +/-11.3 deg. */
    for (int i = 0; i < n; i++) {
        if (f[i].kind != SLAM2D_OBJECT) continue;
        double r = f[i].range_m, b = fabs(f[i].bearing_deg);
        int near_pair = fabs(r - 1.21) < 0.10 && fabs(b - 18.4) < 3.0;
        int far_pair  = fabs(r - 2.00) < 0.10 && fabs(b - 11.3) < 3.0;
        if (!near_pair && !far_pair) {
            check_true(0, "an object at an unexpected place");
            printf("      offending: bearing %+.1f deg range %.2f m\n",
                   f[i].bearing_deg, r);
        }
    }
    check_true(1, "each object sits where a leg was put");

    /* The reflex layer must object to driving into the near pair, and must not object
     * to the clear lane down the middle between them. */
    float half = cfg.rover_width_m * 0.5f;
    float straight = slam2d_arc_clearance(s, 0.0f, half, 6.0f);
    printf("  %-42s %.2f m\n", "clearance straight at the gap between legs", straight);
    check_true(straight > 3.5, "the lane between the legs is open to the far wall");

    /* Aim at a leg instead -- a 3 m radius turn puts the corridor over the one at
     * +18 degrees -- and it must be seen. */
    float at_leg = slam2d_arc_clearance(s, 1.0f / 3.0f, half, 6.0f);
    printf("  %-42s %.2f m\n", "clearance on an arc through a leg", at_leg);
    check_true(at_leg < 1.6, "and swinging into a leg is not");

    slam2d_destroy(s);
    obstacles_on = 0;
}

/* A checksum rather than a count of written cells: the map update both marks and
 * clears, so "how many cells are non-zero" can come out identical either side of a
 * write that moved a wall. Position-weighted, so two cells swapping values show. */
static long grid_fingerprint(const signed char *g, long n_cells)
{
    long sum = 0;
    for (long i = 0; i < n_cells; i++) sum += (long)g[i] * (i + 1);
    return sum;
}

static void test_rejected_scan_is_not_mapped(void)
{
    puts("\n--- a match that was not believed is not written down ---");
    /* The rule, and the reason for it: a scan stamped at a pose this code has just
     * rejected does not merely go to waste. The likelihood field takes the maximum,
     * so it lands at full strength -- as attractive to the next revolution as a wall
     * seen all afternoon -- and from then on the wrong answer has evidence for it.
     * One bad stamp is enough, and with no loop closure nothing ever takes it back.
     *
     * Forced with an impossible threshold rather than by contriving a pose that
     * scores badly, because what is under test is "rejected implies not mapped" and
     * not any particular scan's score. */
    slam2d_config cfg;
    slam2d_default_config(&cfg);
    cfg.mount_deg = MOUNT_DEG;
    /* Above 1.0, because the score is the mean likelihood per point over its own
     * maximum and a stationary scan against its own map really does reach 1.0 --
     * 0.99 was not the impossible threshold it looked like. */
    cfg.min_match_score = 1.5f;
    slam2d *s = slam2d_create(&cfg);
    const long n_cells = (long)cfg.grid_cells * cfg.grid_cells;
    const signed char *g = slam2d_grid(s);

    /* Two steps: the first completes no revolution, the second seeds the map. The
     * seeding scan has nothing to match against and so is never rejected. */
    step(s, 0, 0, 0);
    step(s, 0, 0, 0);
    long seeded = grid_fingerprint(g, n_cells);
    check_true(seeded != 0, "the first scan seeded a map");

    for (int i = 0; i < 10; i++) step(s, 0, 0, 0);
    check_true(slam2d_rejected(s), "every later match is refused at this threshold");
    check_true(grid_fingerprint(g, n_cells) == seeded,
               "and not one of them reached the map");

    slam2d_destroy(s);
}

static void test_write_is_stricter_than_believe(void)
{
    puts("\n--- a match can be believed and still not written ---");
    /* min_match_score keeps the pose; min_write_score keeps the map. A threshold
     * above 1.0 makes every later scan fail the write gate while still matching,
     * which is the split this is testing -- not any particular real-world score. */
    slam2d_config cfg;
    slam2d_default_config(&cfg);
    cfg.mount_deg = MOUNT_DEG;
    cfg.min_write_score = 1.5f;
    slam2d *s = slam2d_create(&cfg);
    const long n_cells = (long)cfg.grid_cells * cfg.grid_cells;
    const signed char *g = slam2d_grid(s);

    step(s, 0, 0, 0);
    step(s, 0, 0, 0);
    long seeded = grid_fingerprint(g, n_cells);
    check_true(seeded != 0, "the first scan still seeded a map");

    double x = 0.0;
    for (int i = 0; i < 10; i++) { x += 0.02; step(s, x, 0, 0); }
    check_true(!slam2d_rejected(s), "later matches were believed");
    check_true(grid_fingerprint(g, n_cells) == seeded,
               "and not one of them reached the map");
    float px, py, pth;
    slam2d_pose(s, &px, &py, &pth);
    close_to("the pose still followed the rover (m)", px, x, 0.06);

    slam2d_destroy(s);
}

static void test_edge_match_is_not_mapped(void)
{
    puts("\n--- a winner against the rim of the window is not written ---");
    /* Twelve degrees is past the coarse window (+/-9) and close enough that the
     * rim candidate still fits, so this is an edge match that would have been
     * believed and written before the write gate, not a rejected one. */
    const double LIE_DEG = 12.0;
    slam2d_config cfg;
    slam2d_default_config(&cfg);
    cfg.mount_deg = MOUNT_DEG;
    slam2d *s = slam2d_create(&cfg);
    const long n_cells = (long)cfg.grid_cells * cfg.grid_cells;

    for (int i = 0; i < 12; i++) step(s, 0, 0, 0);
    long fingerprint = grid_fingerprint(slam2d_grid(s), n_cells);
    check(slam2d_score(s) > 0.8, "a map to stain", slam2d_score(s), 1.0, 0.2);

    slam2d_set_pose(s, 0.0f, 0.0f, (float)(LIE_DEG * M_PI / 180.0));
    step(s, 0, 0, 0);
    check_true(slam2d_match_edge(s), "the winner sat on the rim of the window");
    check_true(!slam2d_rejected(s), "but the pose was still believed");
    check_true(grid_fingerprint(slam2d_grid(s), n_cells) == fingerprint,
               "and none of it reached the map");
    float px, py, pth;
    slam2d_pose(s, &px, &py, &pth);
    double heading = pth * 180.0 / M_PI;
    printf("  %-42s %5.1f deg (was %.0f)\n", "pose walked toward the truth",
           heading, LIE_DEG);
    check_true(fabs(heading) < LIE_DEG - 1.0,
               "the window walked toward the true heading");

    slam2d_destroy(s);
}

static void test_blind_revolution_does_not_seed(void)
{
    puts("\n--- a revolution with no returns is not the seed scan ---");
    /* A covered sensor still completes revolutions: every packet parses, no point
     * survives the range filter. Letting one of those stand as the first scan
     * would mark an empty map as seeded, and every later scan would match an
     * empty field, score zero and be rejected -- so nothing would ever be
     * written, and the rover would dead-reckon inside a map that stayed empty
     * however long it stood in the room. */
    slam2d_config cfg;
    slam2d_default_config(&cfg);
    cfg.mount_deg = MOUNT_DEG;
    slam2d *s = slam2d_create(&cfg);
    const long n_cells = (long)cfg.grid_cells * cfg.grid_cells;
    const signed char *g = slam2d_grid(s);

    no_returns = 1;
    for (int i = 0; i < 4; i++) step(s, 0, 0, 0);
    no_returns = 0;
    check_true(grid_fingerprint(g, n_cells) == 0,
               "blind revolutions left the map empty");

    for (int i = 0; i < 6; i++) step(s, 0, 0, 0);
    check_true(grid_fingerprint(g, n_cells) != 0,
               "the first real scan still seeded it");
    check(slam2d_score(s) > 0.8, "and tracking works on it",
          slam2d_score(s), 1.0, 0.2);
    check_true(!slam2d_rejected(s), "with the matches believed");

    slam2d_destroy(s);
}

static void test_mapping_can_be_suspended(void)
{
    puts("\n--- mapping suspended: still matching, writing nothing ---");
    /* What the caller needs after it has moved the pose itself. Matching has to go
     * on, or the matcher can never find its way back; writing must not, or a bad
     * re-seed is in the map before anyone has checked it. */
    slam2d_config cfg;
    slam2d_default_config(&cfg);
    cfg.mount_deg = MOUNT_DEG;
    slam2d *s = slam2d_create(&cfg);
    const long n_cells = (long)cfg.grid_cells * cfg.grid_cells;
    const signed char *g = slam2d_grid(s);

    double x = 0.0;
    for (int i = 0; i < 12; i++) { x += 0.02; step(s, x, 0, 0); }
    long before = grid_fingerprint(g, n_cells);
    check_true(slam2d_mapping(s), "mapping is on to begin with");

    slam2d_set_mapping(s, 0);
    check_true(!slam2d_mapping(s), "and can be turned off");
    for (int i = 0; i < 10; i++) { x += 0.02; step(s, x, 0, 0); }

    check_true(grid_fingerprint(g, n_cells) == before,
               "ten revolutions later the map is untouched");
    check(slam2d_score(s) > 0.5, "but the scan is still being matched",
          slam2d_score(s), 1.0, 0.5);
    float px, py, pth;
    slam2d_pose(s, &px, &py, &pth);
    close_to("and the pose still followed the rover (m)", px, x, 0.06);

    slam2d_set_mapping(s, 1);
    step(s, x, 0, 0);
    check_true(grid_fingerprint(g, n_cells) != before,
               "turning it back on writes the map again");

    slam2d_destroy(s);
}

static void test_recovery_after_a_bad_reseed(void)
{
    puts("\n--- re-finding a heading somebody else got wrong ---");
    /* The failure this exists for. A dead-reckoned turn moves the pose by an
     * open-loop guess, and one has been seen 48 degrees out on a 90 that physically
     * managed 42. The coarse window is +/-9, so the matcher cannot climb back --
     * and does not say so, because the largest rotation it was allowed to consider
     * still fits the room well enough to score. Here the pose is put 35 degrees out
     * while the rover has not moved at all. */
    const double LIE_DEG = 35.0;
    slam2d_config cfg;
    slam2d_default_config(&cfg);
    cfg.mount_deg = MOUNT_DEG;
    slam2d *s = slam2d_create(&cfg);
    const long n_cells = (long)cfg.grid_cells * cfg.grid_cells;

    for (int i = 0; i < 12; i++) step(s, 0, 0, 0);
    check(slam2d_score(s) > 0.8, "a map to come back to", slam2d_score(s), 1.0, 0.2);

    /* Nothing below is allowed to touch the map: the whole point is that a wrong
     * pose costs revolutions and not geometry. */
    slam2d_set_mapping(s, 0);
    long fingerprint = grid_fingerprint(slam2d_grid(s), n_cells);

    float px, py, pth;
    slam2d_set_pose(s, 0.0f, 0.0f, (float)(LIE_DEG * M_PI / 180.0));
    step(s, 0, 0, 0);
    slam2d_pose(s, &px, &py, &pth);
    double normal_err = fabs(pth * 180.0 / M_PI);
    printf("  %-42s %5.1f deg out, score %.2f\n", "normal window",
           normal_err, slam2d_score(s));
    check_true(normal_err > 15.0, "the ordinary window cannot reach the answer");
    check_true(slam2d_match_edge(s),
               "and says so: the winner sat on the rim of the window");

    /* Same lie, one wide search. */
    slam2d_set_pose(s, 0.0f, 0.0f, (float)(LIE_DEG * M_PI / 180.0));
    slam2d_request_recovery(s);
    step(s, 0, 0, 0);
    slam2d_pose(s, &px, &py, &pth);
    double recovered_err = fabs(pth * 180.0 / M_PI);
    printf("  %-42s %5.1f deg out, score %.2f\n", "recovery window",
           recovered_err, slam2d_score(s));
    close_to("recovery finds the true heading (deg)", recovered_err, 0.0, 2.0);
    check_true(!slam2d_match_edge(s), "well inside the window it searched");
    check(slam2d_score(s) > 0.8, "and it fits", slam2d_score(s), 1.0, 0.2);
    check_true(slam2d_ambiguity(s) < 0.6,
               "with no rival heading worth worrying about");
    check_true(grid_fingerprint(slam2d_grid(s), n_cells) == fingerprint,
               "and none of it reached the map");

    /* The profile is the artifact worth logging: it should peak where the answer
     * is, which is LIE_DEG back from where the search was centred. */
    float off[PROFILE_CAP], sc[PROFILE_CAP];
    int nb = slam2d_angle_profile(s, off, sc, PROFILE_CAP);
    int top = 0;
    for (int i = 1; i < nb; i++) if (sc[i] > sc[top]) top = i;
    printf("  %-42s %d headings, peak at %+.0f deg\n",
           "angle profile", nb, off[top]);
    close_to("the profile peaks at the true heading (deg)",
             off[top], -LIE_DEG, 3.5);

    slam2d_destroy(s);
}

static void test_ambiguity_is_reported(void)
{
    puts("\n--- a room that does not say which way round the rover is ---");
    /* At the centre of a rectangle a half turn maps the room exactly onto itself,
     * so the scan fits equally well two ways and the match picks one by rounding.
     * That is not a low score -- both answers fit beautifully -- which is why the
     * score cannot be the only health check, and why the rival peak is measured.
     *
     * The sweep has to be wide enough to hold both peaks or there is nothing to
     * compare, which is exactly why this reads zero during ordinary tracking. */
    const double CX = (ROOM_XMIN + ROOM_XMAX) / 2.0;
    const double CY = (ROOM_YMIN + ROOM_YMAX) / 2.0;

    double amb[2];
    for (int centred = 0; centred <= 1; centred++) {
        double x = centred ? CX : 0.0, y = centred ? CY : 0.0;
        slam2d_config cfg;
        slam2d_default_config(&cfg);
        cfg.mount_deg = MOUNT_DEG;
        cfg.recover_ang_steps = 60;         /* +/-180 deg, so both peaks are in it */
        slam2d *s = slam2d_create(&cfg);

        for (int i = 0; i < 12; i++) step(s, x, y, 0);
        slam2d_set_mapping(s, 0);
        slam2d_request_recovery(s);
        step(s, x, y, 0);
        amb[centred] = slam2d_ambiguity(s);
        printf("  %-42s rival at %.2f of the winner\n",
               centred ? "at the centre of the room" : "off centre", amb[centred]);
        slam2d_destroy(s);
    }

    check_true(amb[1] > 0.9, "the symmetric pose reports a rival answer");
    check_true(amb[0] < amb[1] - 0.05, "and the asymmetric one a smaller one");
}

static void test_timing(void)
{
    puts("\n--- cost per revolution on this host (100 ms budget at 10 Hz) ---");
    slam2d_config cfg;
    slam2d_default_config(&cfg);
    cfg.mount_deg = MOUNT_DEG;

    int n = make_revolution(rev, 0, 0, 0);

    /* Parse alone: feed the same revolution repeatedly. Each feed reports a wrap
     * that is then never processed, which is exactly the parse path. */
    slam2d *s = slam2d_create(&cfg);
    for (int i = 0; i < 4; i++) step(s, 0, 0, 0);
    double t0 = now_ms();
    int reps = 200;
    for (int i = 0; i < reps; i++) slam2d_feed_lidar(s, rev, n);
    double parse = (now_ms() - t0) / reps;
    slam2d_destroy(s);

    /* Map update plus a single pose evaluation: search window collapsed to nothing. */
    slam2d_config flat = cfg;
    flat.coarse_lin_steps = flat.coarse_ang_steps = 0;
    flat.fine_lin_steps = flat.fine_ang_steps = 0;
    s = slam2d_create(&flat);
    for (int i = 0; i < 4; i++) step(s, 0, 0, 0);
    t0 = now_ms();
    reps = 100;
    for (int i = 0; i < reps; i++) { slam2d_feed_lidar(s, rev, n); slam2d_update(s); }
    double integrate = (now_ms() - t0) / reps - parse;
    slam2d_destroy(s);

    /* The real thing. */
    s = slam2d_create(&cfg);
    for (int i = 0; i < 4; i++) step(s, 0, 0, 0);
    t0 = now_ms();
    reps = 50;
    for (int i = 0; i < reps; i++) { slam2d_feed_lidar(s, rev, n); slam2d_update(s); }
    double total = (now_ms() - t0) / reps;
    slam2d_destroy(s);

    /* The recovery sweep. Not part of the per-revolution budget -- it happens once
     * after a dead-reckoned turn -- but it runs inside the same loop, so what it
     * costs is what that loop stalls for. */
    s = slam2d_create(&cfg);
    for (int i = 0; i < 4; i++) step(s, 0, 0, 0);
    t0 = now_ms();
    reps = 20;
    for (int i = 0; i < reps; i++) {
        slam2d_feed_lidar(s, rev, n);
        slam2d_request_recovery(s);
        slam2d_update(s);
    }
    double recover = (now_ms() - t0) / reps;
    slam2d_destroy(s);

    int poses = (2 * cfg.coarse_lin_steps + 1) * (2 * cfg.coarse_lin_steps + 1)
              * (2 * cfg.coarse_ang_steps + 1)
              + (2 * cfg.fine_lin_steps + 1) * (2 * cfg.fine_lin_steps + 1)
              * (2 * cfg.fine_ang_steps + 1);

    printf("  %-34s %7.2f ms\n", "parse + CRC, 35 packets", parse);
    printf("  %-34s %7.2f ms\n", "map update + 1 pose", integrate);
    printf("  %-34s %7.2f ms   (%d poses, %.3f ms each)\n",
           "scan match", total - parse - integrate, poses,
           (total - parse - integrate) / poses);
    printf("  %-34s %7.2f ms   %5.1f%% of one core -> %s\n",
           "TOTAL per revolution", total, total / 100.0 * 100.0,
           total < 100.0 ? "fits" : "OVER BUDGET");
    printf("  %-34s %7.2f ms   (one-off, after a dead-reckoned turn)\n",
           "recovery sweep", recover);
    check(total < 100.0, "total under the 100 ms budget (ms)", total, 0.0, 100.0);
    /* One dropped revolution is the price of a recovery sweep and is affordable;
     * three would put the rover past its own coarse window on the way back. */
    check(recover < 200.0, "recovery sweep under 200 ms", recover, 0.0, 200.0);
}

int main(void)
{
    build_crc();
    printf("slam2d selftest: %d-point revolutions, room %.0f x %.0f m\n",
           PTS_PER_REV, ROOM_XMAX - ROOM_XMIN, ROOM_YMAX - ROOM_YMIN);

    test_parser_and_room();
    test_stationary();
    test_moving();
    test_reset();
    test_prior_helps();
    test_unknown_sectors();
    test_body_mask();
    test_arc_clearance();
    test_features();
    test_table();
    test_rejected_scan_is_not_mapped();
    test_write_is_stricter_than_believe();
    test_edge_match_is_not_mapped();
    test_blind_revolution_does_not_seed();
    test_mapping_can_be_suspended();
    test_recovery_after_a_bad_reseed();
    test_ambiguity_is_reported();
    test_timing();

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "all passed",
           failures, failures == 1 ? "" : "s");
    return failures != 0;
}
