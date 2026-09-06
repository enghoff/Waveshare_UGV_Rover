# The rover on two pages

A landscape document of how this rover is put together, meant to be read on a
screen or a wall rather than in a terminal. One page per thing that has its own
shape:

| Page | File | What it draws |
|---|---|---|
| 1 | [`Main.dc.html`](Main.dc.html) | how the rover finds its way: the hardware and the program that owns it, the three pieces written here, the mapper and the seven Nav2 servers |
| 2 | [`WorldState.dc.html`](WorldState.dc.html) | what the rover has seen: a look through the gimbal camera, the three models that measure it, and how two of those looks become a thing with a place on the map |

**These pages are for somebody who does not know the rover.** They answer "what
is this and how does it work", in plain English, once. Everything a reader
cannot use at that altitude belongs somewhere else and not here: port numbers
and baud rates, why a module lives in the process it lives in, what was
benchmarked at how many milliseconds, and which parts are still unproven. That
material is the component READMEs' job — [`ros_nav/`](../../ros_nav/README.md)
and [`world_state/`](../../world_state/README.md) — and duplicating it here only
gives it a second place to go stale. Each box therefore leads with what the
thing *does* and carries its source name underneath in a smaller, greyer face.

Each page is an artboard for the design-canvas skill, which wraps them in an
editor; `canvas.json` says where they sit and in what order. Opened on their own
the artboards are inert, because the `<script src="./support.js">` line is a
placeholder that tool replaces. The whole drawing is hand-written HTML —
absolutely positioned boxes over an SVG edge layer, no assets and nothing to
build.

Neither page describes anything that runs, so nothing here is deployed. What
they must not do is disagree with what does: where a page and the source differ,
the source is right and the page is wrong.

## Remaking the PDF

```bash
node make-pdf.mjs
node check-fit.mjs
chrome --headless=new --disable-gpu --no-pdf-header-footer \
       --virtual-time-budget=10000 \
       --print-to-pdf=<absolute path>/rover-architecture.pdf \
       <absolute path>/print.html
```

`make-pdf.mjs` turns the artboards into one printable page in `canvas.json`'s
order; the committed PDF comes out of a real browser's printer rather than the
canvas's own Export PDF button, because that one rasterises each artboard to a
JPEG with a text layer over it and the result reads as a photograph of a slide.
Chrome wants absolute paths for both files and returns before it has finished
writing, so check the size rather than the exit code.

`check-fit.mjs` is the step worth not skipping. Every box on these pages has a
fixed height and visible overflow, so a sentence one line too long prints over
the border below it instead of clipping — invisible in the browser at a glance
and permanent on a wall. It measures each box against its text with the real
webfont loaded, names anything that does not fit, and leaves `page-1.png` and
`page-2.png` beside the PDF to look at.

It does not check width. A label in the SVG edge layer that outgrows the gap it
sits in will print across the boxes on either side and nothing will complain, so
look at the PNGs after changing one.
