# AvataCraft-desktop — a fork of orbis-reframe for the DJI Avata 360

**Upstream:** [Schabo/orbis-reframe](https://github.com/Schabo/orbis-reframe), a
native Linux editor for 360° camera footage (Insta360 `.insv`).
**This fork:** [pcwizardsinc/AvataCraft-desktop](https://github.com/pcwizardsinc/AvataCraft-desktop),
which teaches it to open, stitch, stabilize, reframe and export the **DJI
Avata 360** drone's `.OSV` files, so Avata footage can be edited on Linux
without DJI Studio (Windows-only).

Everything upstream does, this fork does the same way. The DJI support is
additive: a second camera family beside the Insta360 reader, chosen by file
extension. Nothing in the Insta360 path is changed, so upstream's own
documentation ([`README.md`](README.md), [`docs/manual/`](docs/manual/README.md))
still applies, and this fork tracks upstream `main`.

> **AI disclaimer**, as upstream's: the DJI code in this fork was written with
> Claude (Anthropic); the direction, the footage and every real-world test are
> Andrew Chadick's.

## Why a fork, and why this one

DJI Studio is the only editor for Avata 360 `.OSV` files and it runs on
Windows only. The earlier attempts at this ([`pcwizardsinc/DJIStudioCraft`](https://github.com/pcwizardsinc/DJIStudioCraft),
built on [OpenOSV](https://github.com/Kemerd/OpenOSV)) proved the file could be
read but would have meant writing an editor from scratch. orbis-reframe
already has the timeline, media pool, titles, transitions, audio mixing,
colour panels, export and subject tracking; what it lacked was a DJI reader.
Forking it was the shortest path to a complete tool.

## What changed

All DJI code lives in `src/camera/Dji*.{h,cpp}` and is registered as a camera
family in `DjiAvataReader`. The upstream stitcher, stabilizer and UI are used
unchanged.

| Area | File(s) | What it does |
|---|---|---|
| Media pool | `src/camera/MediaInfo.cpp` | `.OSV` recognised as a 360 clip; the import dialog's glob offers both `.OSV` and `.osv` (GTK's file chooser is case-sensitive and DJI writes uppercase) |
| Reader | `DjiAvataReader.{h,cpp}` | Claims `.OSV` files; the two HEVC lens streams, the `.LRF` proxy beside the clip, model, resolution, fps; skips the MJPEG cover stream so a two-lens clip isn't seen as three lenses; the Avata mount (−90° pitch, see below) |
| Telemetry | `DjiSrt.{h,cpp}` | The `.SRT` DJI writes beside the clip: GPS track and the colour mode (`color_md: dlog_m`), so a D-Log M clip is labelled as log and a Normal clip is not |
| Protobuf | `DjiProto.h` | A minimal wire-format scanner (varint / fixed32 / fixed64 / length-delimited), bounds-checked, no schema, no dependency |
| Metadata track | `DjiMetaTrack.{h,cpp}` | Reads the samples of the `djmd` tracks (sample-entry code, not handler name) from the `.OSV` or `.LRF` via FFmpeg; a capped read for the calibration, a full timed read for the attitude |
| Calibration | `DjiDewarp.{h,cpp}` | Decodes DJI's `DewarpParams` records and picks the slot pair the Avata actually fills (`native_refine_far`), rejecting the empty placeholder slots |
| Lens model | `DjiLensFit.{h,cpp}` | Converts DJI's Kannala-Brandt lens into the MEI model upstream's stitcher uses, by least squares; sub-pixel from axis to rim |
| Attitude | `DjiAttitude.{h,cpp}` | One fused attitude quaternion per frame from the `djmd` track, in OpenOSV's verified conventions, for stabilization and horizon lock |
| Tests | `tests/camera_test.cpp` | Unit tests for the protobuf scanner, the calibration decoder, the lens-model conversion and the attitude decoder, on synthetic records |

### History, as it happened

The reader grew as a series of patches applied and tested on real footage
(the DangTree clip: 1920×1920 per lens, 100 fps, HEVC Main10, D-Log M; and a
3840×3840 / 59.94 fps Halloween clip as a second mode). Each step below is
something that was wrong, why, and what fixed it — kept because each one
cost a day.

1. **Approximate calibration** (stereographic, 190°, centred circle) got a
   picture on screen with ghosting at the seam.
2. **Combined frame.** `LensCalibExt::w` is the side-by-side *combined*
   frame, not one lens; getting it wrong put the whole picture in one
   quadrant. For two 1920×1920 lenses: `w = 3840`, lens 1 `cx = 2880`.
3. **Lens pose.** The calibration's `yaw/pitch/roll` are a *correction* to
   the nominal front/back pair, not the pose. Adding 180° made both lenses
   cover the same hemisphere and half the sphere went black.
4. **Real calibration, first attempt — black preview.** DJI's record is
   **Kannala-Brandt** (the polynomial multiplies the off-axis angle), not
   MEI (it multiplies a normalised radius), even though the record has an
   `xi` field: DJI writes `xi = 0` because the model has no such term, not
   because it is a pinhole. Copying `k1..k4` into MEI's `K1..K4` with
   `xi = 0` rendered nothing — a pinhole cannot represent a 190° lens.
   `DjiLensFit` converts the models instead (fix `xi = 1`, solve focal and
   `K1..K4` to match the radial curve to the rim; ≤ 0.33 px error on the
   DangTree lenses).
5. **"The house is on the wrong side of the street."** Both hemispheres
   rendered but were turned 180° against each other along the seam. This
   was *not* a swapped lens order (lens order is stream order: record 1 →
   stream 0, record 2 → stream 1, as OpenOSV ties slot 3/4 to video 0/1).
   It was upstream's stitcher subtracting a fixed 90° Insta360 sensor clock
   (`StitchParams::tzNominalDeg`) from a roll we had written as 0, which
   rolled each lens −90° about its own axis and, the lenses facing opposite
   ways, twisted the sphere in half. Writing `roll = 90` cancels it. That
   was the fix that put the house across the street from the trike.
6. **Attitude.** The Avata records a fused attitude batch per frame in the
   `djmd` track (4 kHz measured). One quaternion per frame, converted with
   OpenOSV's conventions (stored x,y,z,w; world→body; IMU axes relabelled
   by K: (x,y,z)→(y,−z,−x); world +Z up), becomes an orientation track the
   upstream stabilizer levels against. *Being tested on footage as this is
   written; the axis map is an environment knob until confirmed.*

### Facts verified on the Avata 360 (fw 01.00.06.00)

- Streams: `0,1` HEVC 1920×1920 (lenses); `2,3` `djmd`; `4,5` `dbgi`;
  `6` `camd`; `7` MJPEG 960×540 cover. The `.LRF` proxy carries the same
  `djmd` tracks at ~6 % of the size: read it by preference.
- DJI calibrates in a **3840×3840 frame**; the stream may be 1920×1920, so
  every pixel quantity scales by `streamW / 3840`.
- The usable calibration is in the `native_refine_far` slot pair; the
  primary slots are empty placeholders and must be rejected.
- DangTree lens 0: `fx 1035.88 fy 1036.05 cx 1907.74 cy 1924.52`,
  `k = [0.0910 −0.0437 0.0272 −0.0108 0.0013]`, `ypr = [−179.117 90.090 0.254]`;
  lens 1: `fx 1039.39 fy 1039.51 cx 1933.66 cy 1912.35`,
  `ypr = [−0.073 90.084 0.861]`. So the two lenses are within 0.9° of the
  nominal front/back pair, and the pose is carried by the nominal pair and
  the mount, not by DJI's angles (which are left out for now, and logged).
- The Avata flies the camera with one lens up and one down. The mount is
  **−90° pitch** (measured: of the four quarter-turns, −90° puts mounted-up
  within 0.962 of true up against 0.193 for none). Upstream OpenOSV lacks
  this; it came from DJIStudioCraft.
- The `.SRT` has **no attitude** — ISO, shutter, EV, colour temperature,
  `color_md`, GPS only.
- Lens guards must stay **None** (bare lenses); "Premium" visibly warps the
  image. "New stitcher (experimental)" looks best.

## Building on Fedora

Fedora 45 ships Qt, OpenCV and FFmpeg new enough; no local workarounds.

```bash
git clone https://github.com/pcwizardsinc/AvataCraft-desktop ~/AvataCraft-desktop
cd ~/AvataCraft-desktop
conda deactivate 2>/dev/null   # if conda auto-activates, it breaks the build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j$(nproc)
./build/camera_test            # should end in "all passed"
./build/orbis-reframe          # the binary is not renamed yet
```

Tested on a 2017 27" iMac (AMD Radeon Pro 580X, 32 GB). The Radeon can't do
HEVC decode through Vulkan, so playback is CPU-decoded and sluggish on this
machine; VA-API decode is on the list below.

## Diagnostic knobs

All read once when a clip is opened, and logged to stderr as
`[DjiAvataReader] …`. None are needed on a correctly working build; they
exist so a wrong guess can be checked on footage without rebuilding.

| Variable | Effect |
|---|---|
| `AVATA_CALIB=approx` | ignore the file's calibration and use the stereographic stand-in |
| `AVATA_SWAP_LENSES=1` | feed stream 1 to lens 0 and stream 0 to lens 1 |
| `AVATA_LENS_ROLL=<deg>` | the roll written for both lenses (default 90, cancelling the stitcher's sensor clock) |
| `AVATA_IMU_AXES=<map>` | the attitude's axis map (default `x,z,-y`: aircraft right/forward/up → sphere +X/−Z/+Y) |
| `AVATA_IMU_RAW=1` | skip the IMU axis relabel |
| `AVATA_IMU_B2W=1` | read the stored attitude as body→world instead of world→body |

## What's left

1. Confirm the attitude track on footage: Horizon lock level and steady.
2. Apply DJI's sub-degree per-lens angles (or let upstream's seam fit absorb them).
3. "GPS: Not recorded" in Media information although `readGps()` parses the `.SRT` — a wiring gap.
4. Thumbnail from the MJPEG cover (`readThumbnail()` is a TODO).
5. Rename the binary and window title to **Avata 360 Editor**; a desktop entry.
6. VA-API decode for AMD.
7. Upstream: once the reader is stable, offer it to orbis-reframe as a pull request, and the Avata mount to OpenOSV.

## Where the earlier work went

The two repositories this grew out of were folded into [`reference/`](reference/README.md)
in November 2026 and deleted on GitHub:

- **DJIStudioCraft** (OpenOSV-based) — our seven commits are kept as a patch
  series: the Avata levelling fix, the Fedora build, VA-API decode, and the
  build guide, roadmap and UI notes.
- **AvataCraft** (the Gemini browser mock-up) — its React source is kept as a
  design reference; its synthetic footage and bundled tarball are not.

The planning documents from that period are in `docs/`:
[`INTERFACE-SPEC.md`](docs/INTERFACE-SPEC.md) (the target UI, from DJI Studio
screenshots) and [`METADATA-PORT-PLAN.md`](docs/METADATA-PORT-PLAN.md)
(historical: the plan for the port, with notes on where it was wrong).

[`Kemerd/OpenOSV`](https://github.com/Kemerd/OpenOSV) remains the open `.OSV`
reader whose protobuf schema, slot selection and attitude conventions this
fork's decoders follow.
