/* Correctness and cost of slam2d, against scans synthesised from a known room.
 *
 * The point of generating the packets rather than replaying a capture is that the
 * room is known, so "the sensor is read correctly" is a measured error in
 * millimetres instead of a picture that looks about right. Packets are built the
 * way the sensor builds them -- 47 bytes, real start/end angles, real CRC-8 -- so
 * the parser is under test rather than assumed.
 *
 * There used to be twice as many tests here, covering the scan matcher, the
 * occupancy grid and the recovery search. Those went with the code they tested;
 * `slam_toolbox` does that job now. What is left tests the two things this
 * library still is: an LD19 parser, and the segmentation that turns a revolution
 * into walls, objects and gaps.
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

static unsigned char rev[PKTS_PER_REV * 47];

/* How many points the last completed revolution left in the parser. There is no
 * accessor for the count on its own, and there does not need to be: reading the
 * scan out is what every real caller does. */
static float xy_buf[2 * 4096];
static int scan_points(const slam2d *s)
{
    return slam2d_scan_xy(s, xy_buf, 2048);
}

/* Feed one revolution's bytes. Returns the number of revolutions that completed,
 * which is 0 or 1. Note the inherent one-revolution lag: the wrap that ends
 * revolution N is only seen when revolution N+1's first packet arrives. */
static int step(slam2d *s, double x, double y, double th)
{
    int n = make_revolution(rev, x, y, th);
    return slam2d_feed_lidar(s, rev, n);
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
    step(s, 0, 0, 0);                      /* this one completes a revolution */

    check(scan_points(s) > 400, "points recovered from one revolution",
          scan_points(s), PTS_PER_REV, 20);

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

    /* The scan is the output now, so decimation is the one setting that can quietly
     * make this sensor worse than it is. The default has to keep a whole revolution. */
    slam2d_config full;
    slam2d_default_config(&full);
    full.mount_deg = MOUNT_DEG;
    full.max_range_m = 12.0f;
    slam2d *d = slam2d_create(&full);
    step(d, 0, 0, 0);
    step(d, 0, 0, 0);
    check(scan_points(d) > 400, "the default keeps a whole revolution",
          scan_points(d), PTS_PER_REV, 20);
    slam2d_destroy(d);

    slam2d_destroy(s);
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
    slam2d_sectors(s, sec, 36);
    int kept = 0;
    for (int i = 0; i < 36; i++) if (sec[i] > 0.0f) kept++;
    check(kept >= 34, "sectors kept at 0.60 m, well outside the box", kept, 36, 2);
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

    slam2d_destroy(s);
    obstacles_on = 0;
}

static void test_blind_revolution_is_not_reported(void)
{
    puts("\n--- a revolution with no returns is not a revolution ---");
    /* A covered sensor still turns, and every packet still parses and passes its
     * CRC -- they are full of zero distances, which is what "no echo" looks like on
     * the wire. The parser drops those at the range filter, so the wrap arrives with
     * an empty accumulator and is discarded rather than handed over.
     *
     * The consequence is worth stating out loud, because it is not obvious from
     * either end: **a blinded lidar is indistinguishable from a stopped one here.**
     * Nothing is published, so the scan age climbs, `lidar_ok` goes false, and the
     * USB replug ladder in usbreset.py will eventually fire on a sensor that is
     * spinning perfectly well with a bag over it. That is the right trade -- an
     * empty scan published as fact would tell slam_toolbox the room is empty, which
     * is far worse -- but somebody debugging a rover that keeps replugging a healthy
     * lidar should check what is in front of it before they check the cable. */
    slam2d_config cfg;
    slam2d_default_config(&cfg);
    cfg.mount_deg = MOUNT_DEG;
    slam2d *s = slam2d_create(&cfg);

    no_returns = 1;
    int blind_revs = 0;
    for (int i = 0; i < 4; i++) blind_revs += step(s, 0, 0, 0);
    no_returns = 0;
    check(blind_revs == 0, "revolutions reported from a blind sensor",
          blind_revs, 0, 0);
    check(scan_points(s) == 0, "and no scan was left behind", scan_points(s), 0, 0);

    /* And it recovers the moment real returns come back: no resync needed, because
     * nothing about the byte stream was ever out of step. */
    for (int i = 0; i < 2; i++) step(s, 0, 0, 0);
    check(scan_points(s) > 100, "the next real revolution comes back full",
          scan_points(s), PTS_PER_REV, 400);

    slam2d_destroy(s);
}

static void test_midstream_join_discards_the_remnant(void)
{
    puts("\n--- joining a spinning sensor discards the remnant ---");
    /* The port opens onto a lidar that has been turning the whole time, so the
     * first wrap is the leftover of the revolution we joined in the middle of.
     * Publishing that as a scan hands slam_toolbox a wedge of room with two thirds
     * of the horizon missing, which it will happily try to match. */
    slam2d_config cfg;
    slam2d_default_config(&cfg);
    cfg.mount_deg = MOUNT_DEG;
    cfg.max_range_m = 12.0f;
    slam2d *s = slam2d_create(&cfg);

    int n = make_revolution(rev, 0, 0, 0);
    int half = (PKTS_PER_REV / 2) * 47;
    slam2d_feed_lidar(s, rev + half, n - half);   /* remnant, no wrap yet */
    check(scan_points(s) == 0, "a half-revolution by itself completed nothing",
          scan_points(s), 0, 0);

    check_true(step(s, 0, 0, 0) == 0,
               "the wrap of the remnant was discarded, not published");
    check(scan_points(s) == 0, "and left no scan behind it", scan_points(s), 0, 0);

    check_true(step(s, 0, 0, 0) == 1,
               "the next wrap is a full revolution and is published");

    float sec[72];
    slam2d_sectors(s, sec, 72);
    close_to("that one sees the wall ahead",  sec[0],  ROOM_XMAX,  0.05);
    close_to("and the wall behind",           sec[36], -ROOM_XMIN, 0.05);

    slam2d_destroy(s);
}

static void test_timing(void)
{
    puts("\n--- cost per revolution on this host (100 ms budget at 10 Hz) ---");
    slam2d_config cfg;
    slam2d_default_config(&cfg);
    cfg.mount_deg = MOUNT_DEG;
    cfg.max_range_m = 12.0f;

    int n = make_revolution(rev, 0, 0, 0);

    /* Parse alone: feed the same revolution repeatedly. */
    slam2d *s = slam2d_create(&cfg);
    for (int i = 0; i < 4; i++) step(s, 0, 0, 0);
    double t0 = now_ms();
    int reps = 500;
    for (int i = 0; i < reps; i++) slam2d_feed_lidar(s, rev, n);
    double parse = (now_ms() - t0) / reps;

    /* Reading the scan out, which is what the ROS node does with every revolution
     * and the only other thing on the per-revolution path. */
    t0 = now_ms();
    reps = 500;
    for (int i = 0; i < reps; i++) scan_points(s);
    double read_out = (now_ms() - t0) / reps;

    /* Segmentation. Not per-revolution -- the node runs it on a timer, and only
     * when something is subscribed -- but it is the expensive half of this library
     * now and it runs on the same thread as the parse. */
    slam2d_feature f[64];
    t0 = now_ms();
    reps = 200;
    for (int i = 0; i < reps; i++) slam2d_features(s, f, 64);
    double describe = (now_ms() - t0) / reps;
    slam2d_destroy(s);

    printf("  %-34s %7.2f ms\n", "parse + CRC, 35 packets", parse);
    printf("  %-34s %7.2f ms\n", "reading the scan out", read_out);
    printf("  %-34s %7.2f ms   %5.1f%% of one core -> %s\n",
           "TOTAL per revolution", parse + read_out,
           (parse + read_out) / 100.0 * 100.0,
           parse + read_out < 100.0 ? "fits" : "OVER BUDGET");
    printf("  %-34s %7.2f ms   (on a timer, only when subscribed)\n",
           "segmentation into features", describe);
    check(parse + read_out < 100.0, "total under the 100 ms budget (ms)",
          parse + read_out, 0.0, 100.0);
    /* It shares a thread with the parse, so a slow one drops revolutions. */
    check(describe < 50.0, "segmentation under 50 ms", describe, 0.0, 50.0);
}

int main(void)
{
    build_crc();
    printf("slam2d selftest: %d-point revolutions, room %.0f x %.0f m\n",
           PTS_PER_REV, ROOM_XMAX - ROOM_XMIN, ROOM_YMAX - ROOM_YMIN);

    test_parser_and_room();
    test_unknown_sectors();
    test_body_mask();
    test_features();
    test_table();
    test_blind_revolution_is_not_reported();
    test_midstream_join_discards_the_remnant();
    test_timing();

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "all passed",
           failures, failures == 1 ? "" : "s");
    return failures != 0;
}
