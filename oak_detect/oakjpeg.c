/* JPEG -> planar BGR, the one thing the Pi still has to do itself.
 *
 * The rover's camera hands out MJPEG and the graph wants planar BGR bytes, so
 * somebody has to decode. On a 700 MHz ARM1176 with no NEON that is the most
 * expensive step in the whole loop -- three times the inference -- which is why
 * it is here in C rather than in the server, and why the graph is compiled for
 * 320x240 rather than the model's native 300x300.
 *
 * That size is not arbitrary. libjpeg-turbo can scale while it decodes, but only
 * by a fixed set of fractions, and 320x240 is exactly half of the camera's
 * 640x480. So the decoder is asked for the frame at half size and lands precisely
 * on the graph's input, and the resize step -- 70 ms of bilinear interpolation on
 * this host, measured, more than the inference it was feeding -- disappears into
 * a plane split. 4:3 in and 4:3 out also means the picture is no longer squashed
 * to a square the way the stock 300x300 input requires.
 *
 * The general path is still here and still correct for any other pair of sizes,
 * because the camera's format is not something this can assume. It just is not
 * the one that runs.
 *
 * Only the runtime library is needed, never the -dev package: these five
 * prototypes are the whole of the TurboJPEG API this uses, and build.sh links
 * libturbojpeg.so.0 by path. The Pi has no headers and no sudo to install them.
 */

#include <stdlib.h>
#include <string.h>

typedef void *tjhandle;
typedef struct { int num, denom; } tjscalingfactor;

extern tjhandle tjInitDecompress(void);
extern int tjDecompressHeader3(tjhandle h, const unsigned char *buf,
                               unsigned long size, int *width, int *height,
                               int *subsamp, int *colorspace);
extern int tjDecompress2(tjhandle h, const unsigned char *buf, unsigned long size,
                         unsigned char *dst, int width, int pitch, int height,
                         int pixelFormat, int flags);
extern tjscalingfactor *tjGetScalingFactors(int *n);

#define TJPF_BGR            1
#define TJFLAG_FASTUPSAMPLE 256
#define TJFLAG_FASTDCT      2048
#define TJSCALED(dim, sf)   (((dim) * (sf).num + (sf).denom - 1) / (sf).denom)

/* One decompressor, one scratch buffer and one set of resize tables for the life
 * of the process. Safe only because the server holds a lock across a whole
 * detection -- the device runs one inference at a time regardless, so there is
 * nothing to gain from making any of this re-entrant. */
static tjhandle g_tj;
static unsigned char *g_scratch;
static size_t g_scratch_len;
static int *g_xmap;                 /* x0, x1 and weight per output column */
static int g_xmap_w, g_xmap_sw;

static int ensure_scratch(size_t want)
{
    unsigned char *grown;

    if (g_scratch_len >= want)
        return 0;
    grown = realloc(g_scratch, want);
    if (!grown)
        return -1;
    g_scratch = grown;
    g_scratch_len = want;
    return 0;
}

int oak_jpeg_header(const unsigned char *jpeg, unsigned long len, int *w, int *h)
{
    int subsamp, colorspace;

    if (!g_tj && !(g_tj = tjInitDecompress()))
        return -1;
    if (tjDecompressHeader3(g_tj, jpeg, len, w, h, &subsamp, &colorspace) < 0)
        return -1;
    return 0;
}

/* The smallest scaled size libjpeg can produce that still covers outw x outh.
 * Falls back to full size when nothing fits, which is what happens whenever the
 * frame is already smaller than the graph's input. */
static void pick_scale(int sw, int sh, int outw, int outh, int *dw, int *dh)
{
    tjscalingfactor *factors;
    int n = 0, i, best_w = sw, best_h = sh;

    factors = tjGetScalingFactors(&n);
    if (!factors)
        goto done;
    for (i = 0; i < n; i++) {
        int cw = TJSCALED(sw, factors[i]);
        int ch = TJSCALED(sh, factors[i]);

        if (cw < outw || ch < outh)
            continue;                       /* would have to be upscaled again */
        if (cw * (long)ch < best_w * (long)best_h) {
            best_w = cw;
            best_h = ch;
        }
    }
done:
    *dw = best_w;
    *dh = best_h;
}

/* Interleaved BGR to three planes, same size. The path that actually runs. */
static void split_planes(const unsigned char *src, unsigned char *out, int count)
{
    unsigned char *b = out, *g = out + count, *r = out + 2 * count;
    int i;

    for (i = 0; i < count; i++) {
        *b++ = *src++;
        *g++ = *src++;
        *r++ = *src++;
    }
}

/* Bilinear, in 16.16 fixed point, with the horizontal mapping computed once for
 * the whole image rather than once per pixel: the VFP here is scalar, and this
 * runs a quarter of a million samples for every frame. */
static int resize_to_planes(const unsigned char *src, int sw, int sh,
                            unsigned char *out, int ow, int oh)
{
    const int plane = ow * oh;
    unsigned int y_ratio = sh > 1 ? ((unsigned)(sh - 1) << 16) / (oh > 1 ? oh - 1 : 1) : 0;
    unsigned int x_ratio = sw > 1 ? ((unsigned)(sw - 1) << 16) / (ow > 1 ? ow - 1 : 1) : 0;
    int x, y;

    if (g_xmap_w != ow || g_xmap_sw != sw) {
        int *grown = realloc(g_xmap, (size_t)ow * 3 * sizeof(int));

        if (!grown)
            return -1;
        g_xmap = grown;
        for (x = 0; x < ow; x++) {
            unsigned int sx = x * x_ratio;
            int x0 = sx >> 16;

            g_xmap[x * 3 + 0] = x0 * 3;
            g_xmap[x * 3 + 1] = (x0 + 1 < sw ? x0 + 1 : x0) * 3;
            g_xmap[x * 3 + 2] = sx & 0xffff;
        }
        g_xmap_w = ow;
        g_xmap_sw = sw;
    }

    for (y = 0; y < oh; y++) {
        unsigned int sy = y * y_ratio;
        int y0 = sy >> 16;
        int wy = sy & 0xffff;
        const unsigned char *row0 = src + (size_t)y0 * sw * 3;
        const unsigned char *row1 = src + (size_t)(y0 + 1 < sh ? y0 + 1 : y0) * sw * 3;
        unsigned char *b = out + (size_t)y * ow;
        unsigned char *g = b + plane;
        unsigned char *r = g + plane;
        const int *map = g_xmap;

        for (x = 0; x < ow; x++, map += 3) {
            const unsigned char *a0 = row0 + map[0], *a1 = row0 + map[1];
            const unsigned char *c0 = row1 + map[0], *c1 = row1 + map[1];
            int wx = map[2];
            int top, bot;

            top = a0[0] + (((a1[0] - a0[0]) * wx) >> 16);
            bot = c0[0] + (((c1[0] - c0[0]) * wx) >> 16);
            b[x] = (unsigned char)(top + (((bot - top) * wy) >> 16));

            top = a0[1] + (((a1[1] - a0[1]) * wx) >> 16);
            bot = c0[1] + (((c1[1] - c0[1]) * wx) >> 16);
            g[x] = (unsigned char)(top + (((bot - top) * wy) >> 16));

            top = a0[2] + (((a1[2] - a0[2]) * wx) >> 16);
            bot = c0[2] + (((c1[2] - c0[2]) * wx) >> 16);
            r[x] = (unsigned char)(top + (((bot - top) * wy) >> 16));
        }
    }
    return 0;
}

/* Returns 0 on success. src_w/src_h come back as the frame's own size, so the
 * caller can put boxes into full-frame pixels without parsing the JPEG itself,
 * and dec_w/dec_h as the size it was decoded at, which is how you tell from the
 * outside whether the resize was skipped. */
int oak_jpeg_to_planar_bgr(const unsigned char *jpeg, unsigned long len,
                           unsigned char *out, int outw, int outh,
                           int *src_w, int *src_h, int *dec_w, int *dec_h)
{
    int sw = 0, sh = 0, dw = 0, dh = 0, subsamp, colorspace;

    if (!g_tj && !(g_tj = tjInitDecompress()))
        return -1;
    if (tjDecompressHeader3(g_tj, jpeg, len, &sw, &sh, &subsamp, &colorspace) < 0)
        return -1;

    pick_scale(sw, sh, outw, outh, &dw, &dh);
    if (ensure_scratch((size_t)dw * dh * 3) < 0)
        return -1;
    if (tjDecompress2(g_tj, jpeg, len, g_scratch, dw, dw * 3, dh, TJPF_BGR,
                      TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE) < 0)
        return -1;

    if (dw == outw && dh == outh)
        split_planes(g_scratch, out, outw * outh);
    else if (resize_to_planes(g_scratch, dw, dh, out, outw, outh) < 0)
        return -1;

    *src_w = sw;
    *src_h = sh;
    *dec_w = dw;
    *dec_h = dh;
    return 0;
}
