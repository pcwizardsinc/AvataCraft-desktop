# Porting DJI's .OSV metadata into AvataCraft-desktop

> **Historical (written before the port; kept for the reasoning).** Steps
> 1–4 are done; the outcome, including where this plan was wrong (the lens
> model is Kannala-Brandt and had to be converted, and the hemisphere twist
> was the stitcher's roll clock, not a calibration or lens-order fault), is
> in [`../FORK.md`](../FORK.md) and [`formats/dji-360.md`](formats/dji-360.md) §0.

The last structural gap. It fixes **both** remaining defects at once:

| Defect seen on the DangTree clip | Cause | Fixed by |
|---|---|---|
| Ghosting / doubling at the seam | lens calibration is a stereographic *approximation* (assumed 190 deg FOV, perfectly centred circle) | the file's real `DewarpParams` |
| Horizon does not hold; Stabilization Mode does nothing | `DjiAvataReader::readImu()` returns nothing, so there is no attitude track | the file's attitude records |

## Where the data is

ffprobe on a real Avata 360 clip (DJI_20261018153702_0001_D_2026.OSV):

```
0,hevc,video,1920,1920     lens 0
1,hevc,video,1920,1920     lens 1
2..6,unknown,data          <-- DJI metadata ('meta' handler, 'djmd' sample entry)
7,mjpeg,video,960,540      embedded cover (skipped as a lens; see LensDemuxer)
```

The metadata tracks carry **protobuf** records. OpenOSV calls the lens record
`DewarpParams`; the Avata 360 writes its usable pair in the slot OpenOSV names
`native_refine_far` (slots 3/4), leaving `native_refine` (1/2) with no
intrinsics and `native` (11/12) as a placeholder. DJI's own tools fall back the
same way, so the selector must too.

## DewarpParams -> insv::LensCalibExt

Field numbers from OpenOSV's `DjmdDecoder::decodeDewarp` (all fixed32 floats
unless noted):

| # | DJI field | Goes to |
|---|---|---|
| 1, 2 | fx, fy | `fx`, `fy` |
| 3, 4 | cx, cy | `cx`, `cy` (see frame note) |
| 5-8, 15-19 | k1..k9 radial | `k[0..3]` (orbis keeps four) |
| 9 | xi (unified model; DJI writes 0) | `xi` |
| 10, 11 | width, height (written as **floats**, e.g. 3840.0) | calibration frame |
| 12, 13, 14 | yaw, pitch, roll (Euler, deg) | lens pose correction |
| 20 | tangential p1, p2 | `p[0..1]` |
| 28 | camExtriQ (body->lens quaternion) | lens pose; **the Avata writes its extrinsic only via the Euler fields 12/13/14**, so `camExtriQFromEuler` applies |

### Frame conventions (learned the hard way)
* `LensCalibExt::w/h` is the **side-by-side combined** frame, not one lens:
  `FisheyeProjection` does `halfW = w/2` and subtracts it for lens 1. For two
  1920x1920 lens tracks that is w = 3840, lens 1 `cx` = 2880.
  (Getting this wrong put the whole picture in one quadrant.)
* `yaw/pitch/roll` are a **correction to the nominal pose**, not the pose.
  `FisheyeProjection` already turns lens 1 by `nominalYaw1Deg` (180 deg);
  adding another 180 here made both lenses cover the same hemisphere and half
  the sphere came out black.
* DJI calibrates in a 3840x3840 frame; the stream is 1920x1920, so
  `focalRefW` must describe the window `fx/fy` are expressed in and the
  projection scales by `streamW / focalRefW`.

## Attitude -> insv::ImuTrack

The Avata 360 records a dense attitude track (4 kHz measured). Needed for
`readImu()`, which gives Stabilization and Horizon lock something to work with.

**Carry over the Avata mount correction** already proven in this project: the
aircraft flies the 360 camera with one lens up and one down, and its IMU sits a
quarter-turn in pitch from the nose-forward, sky-up frame. Measured on real
footage (fw 01.00.06.00, 4K/100): of the four quarter-turns about the mounted
right axis, **-90 deg** puts mounted-up within 0.962 of true up, against 0.193
for no turn. Upstream OpenOSV does not have this fix; it came from
DJIStudioCraft and is already in `DjiAvataReader::mountFor()`.

## Port order

1. **Protobuf scan** - a small wire-format reader (varint / fixed32 / fixed64 /
   length-delimited). Self-contained; OpenOSV's has no container dependency.
2. **Locate the records** - the `meta` track whose stsd sample entry is `djmd`.
3. **DewarpParams** - decode, pick the slot pair (native_refine_far on the
   Avata), map to `LensCalibExt` per the table above, set `calibrated = true`.
4. **Attitude** - decode into `insv::ImuTrack`, apply the mount, return it from
   `readImu()`.
5. **Thumbnail** - the MJPEG cover (stream 7) is far cheaper than decoding
   video for the media pool grid; `readThumbnail()` is still a TODO.
6. **GPS** - from the `.SRT` beside the clip, or the metadata; `readGps()` TODO.

Steps 1-3 remove the ghosting; step 4 makes the horizon hold.

## Meanwhile

orbis-reframe can already compensate for an imperfect calibration from the
footage itself: `EstimateSeamAlignment` solves for radial scale and lens-1
centring, and the Stitching panel exposes **Seam flow** (warps the overlap to
match) and **Seam colour blend**. Worth turning on while the port is in
progress.
