# Printed parts

Parametric models, written in [build123d](https://build123d.readthedocs.io) so
that the dimensions that matter are named constants rather than numbers buried
in a mesh. Each script builds its parts, checks the fit against a model of
whatever it has to mate with, and writes STEP and STL into `cad/out/` (which is
gitignored — regenerate, don't commit). Alongside the two parts it writes
`oak_rail_mount.step`, both of them named and where they sit, for anything that
wants the assembly rather than two things to print.

Nothing here is deployed. build123d is not in `requirements.txt` either, because
the rover never needs it; install it into a scratch environment when you want to
change a part.

```
python -m venv /tmp/cad && /tmp/cad/bin/pip install build123d ocp_vscode
/tmp/cad/bin/python cad/oak_rail_mount.py
```

## Looking at a part

`bernhard-42.ocp-cad-viewer` is installed in VS Code here. Open its panel from
the command palette ("OCP CAD Viewer: Open viewer"), then

```
python cad/oak_rail_mount.py --show
python cad/oak_rail_mount.py --show --section -23
```

which throws the clamp, the wedge, a length of reference rail and a ghost of the
camera into the panel, to spin around. The section form slices everything at
that station along the rail, and is the only way to actually see what the jaws
are doing — from outside, the plate hides the whole clamp.

Without VS Code, `python -m ocp_vscode --port 3939` serves the same viewer at
<http://127.0.0.1:3939> and `--show` will find it there. Set `OCP_PORT` if you
need a different one.

## Drawings on paper

```
python cad/drawing.py            cad/out/oak_rail_mount.pdf, one A4 page per part
python cad/drawing.py --svg      the same two sheets as SVG, to open in a browser
```

Each page is the front, top and right views in third angle, at 1:1, with the
hidden detail dashed and an isometric alongside. The views are cut by
build123d's hidden line removal, so they are projections of the solid itself
rather than of a mesh of it, and the sheet carries a 100 mm rule — print at
100%, because a print dialogue set to fit the page will quietly ruin the one
thing a full size drawing is for.

`drawing.py` runs its own checks and exits non-zero if any fail: that each view
came out the size of the part, and then that the written PDF, read back and its
own drawing operators replayed, holds the points and labels that were drawn,
in millimetres, inside the border. `sheet.py` underneath it is a sheet of paper
in millimetres that writes itself as PDF or SVG, and depends on nothing.

## `oak_rail_mount.py` — OAK-D-Lite on a Picatinny rail

Two parts that hang the OAK-D-Lite off a MIL-STD-1913 rail: a **body** carrying
the clamp roof, the fixed jaw, a recoil lug and the vertical plate the camera
bolts to, and a **wedge** that is the moving jaw. Two M4 screws drop through the
roof, outboard of the rail, into two T-nuts lying in a slot cut down the length
of the wedge; tightening them pulls the wedge up so its 45 degree face rides the
rail's undercut, which squeezes the rail against the fixed jaw and pulls the roof
down onto the rail's top face. The lug drops into one cross slot and is what
actually stops the mount creeping fore and aft.

The camera hangs off the two M4 holes in the back of its case rather than the
tripod socket underneath. Those holes are the only mount on the camera that is
symmetric about the lens: they sit 75.00 mm apart — exactly the stereo baseline
— directly behind the two mono cameras and 0.6 mm below the line the three
lenses sit on. The tripod socket, by contrast, is 30 mm off the middle of the
case, so hanging the camera from it would put the optical axis nowhere near the
rail. Every camera dimension in the script was measured off Luxonis's published
enclosure model, not read off the datasheet drawing.

Two raised pads on the plate, one around each screw, hold the plate about 2 mm
off the rest of the back so the heatsink fins still see air.

Every outside edge on both parts carries a 0.6 mm chamfer. That is done by
rule rather than by hand — the script works out which edges are outside corners
and breaks all of them, and the only exceptions are the four edges that bound
the faces gripping the rail, where a chamfer would just be contact area given
away. Picking edges by hand is how you end up with a part that is mostly
chamfered, so `checks()` fails if any outside edge comes through sharp.

### The fasteners

The whole mount runs on **four M4 x 10 socket head screws and two 20-series M4
T-nuts**, which is a short screw for both jobs it has to do. Rather than give
away thread, both pairs of heads sink into counterbores: 2 mm into the roof and
3 mm into the back of the plate. A camera screw then takes 5 mm of the 6.45 mm
tapped into the case and still stops 1.45 mm short of the bottom, and a clamp
screw takes 3 of the 4 mm of thread a T-nut this size has. Either head shape
fits — a button head sits about flush, a cap head stands a couple of
millimetres proud.

The wedge holds its nuts the way the extrusion they were made for would: a
T-slot running its length, a chamber for the flanges with a 6.4 mm neck above it
for the boss and the screw. The nuts slide in from either end, the chamber walls
stop them turning, and they pull up on a 3 mm ledge, 1.8 mm of bearing a side
and 22 mm2 in all. That is slightly less than the hex nuts this replaced had, so
the plastic rather than the steel is what limits how hard the clamp can be done
up — the ledge under a nut flange and the roof under a screw head both give out
somewhere around a kilonewton, where the screw itself is good for four. Firm,
not hard.

The nut is much the wider part, so the clamp had to grow from 44 mm across to 54
to keep 2 mm of wedge either side of the slot. The plate is 89 mm wide
regardless, so the mount's envelope did not change at all; the body goes from
33.5 to 35.8 cm3 and the wedge from 2.0 to 2.7.

The slot has a floor, and that floor is the only thing tying its two sides
together. Cut the chamber out of the underside instead and the wedge stops being
one part: the neck takes the top of it apart and the open bottom takes the rest,
leaving two loose rails with the nuts lying between them. Making room for the
floor is why the wedge hangs 1.4 mm below the fixed jaw, so the rail wants
10.4 mm of clear beside it rather than 9. Between the two nuts the neck is left
filled as well, closing the top of the wedge over its middle. That fill can only
go there: each nut reaches its station by sliding in from the near end of the
wedge with its boss riding in the neck, so filling either end would leave the
nuts no way in.

### What you need

| | |
|---|---|
| Screws | 4 x M4 x 10 socket head — two for the camera, two for the clamp |
| Nuts | 2 x 20-series M4 T-nuts, sliding into the wedge's T-slot |
| Material | PETG or ABS rather than PLA — it sits in the sun on the gimbal |
| Print, body | plate face down on the bed, 4 perimeters, 40% infill, supports under the jaws |
| Print, wedge | underside on the bed, no supports; the slot's roof bridges 10.4 mm |

### Before you print

**These nuts were measured, not looked up.** No vendor publishes a dimensioned
drawing of a 20-series M4 T-nut — they give the thread and the slot width and
stop there — so the `TNUT_` block is calipers: 10 x 6 x 4 mm overall, of which
3 mm is flange and the last 1 mm is boss. A different nut changes the part,
because everything downstream is derived from those numbers rather than typed:

| | what it sets |
|---|---|
| `TNUT_HEAD_W`, across the flanges | how wide the clamp is, at two millimetres of clamp per millimetre of nut |
| `TNUT_BOSS_H`, flange top to boss top | the thread standing above the seat, and the only reason an M4 x 10 reaches at all |
| `TNUT_FLANGE_T`, flange top to the bottom | where the slot floor goes, and with it how far the wedge hangs down |

The boss is the tight one. At 1 mm it is about half what a nut this wide usually
carries, and it leaves the clamp screw 3 mm of thread out of the 4 the nut has.
That is still more than the screw is worth — 3 mm of M4 in a steel nut strips
somewhere above 6 kN and the screw parts at about 4 — but there is nothing left
to give away, so a nut with no boss at all would want either a thinner ledge or
longer clamp screws.

The script assumes a rail on the gimbal's tilt platform running fore and aft,
with the camera looking along it, and at least 40 mm of rail with a cross slot
roughly in the middle of that. It also assumes the rail is to spec: the clamp is
cut against the maximum-material profile from MIL-STD-1913 Figure 1, so a
generous aftermarket rail will be loose and an oversize one will not go on. If
the printed body rocks on the rail, take a tenth off `FIT` and print again.

`CAM_BOTTOM_Z` is the one number worth thinking about. It sets how far the
camera's bottom edge floats above the rail, and at the default 10 mm the optical
axis ends up 29 mm above the rail's top surface. It is that high only so a
right-angle USB-C plug fits under the case, because the port points straight
down. If the rail stops short of the camera and the cable can hang free, drop it
to about 3 and the optical axis comes down to 22 mm — less mass off the tilt
axis, and less parallax to correct for.
