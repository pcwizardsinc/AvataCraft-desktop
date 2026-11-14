# Avata 360 Editor: interface specification

> The target interface for this fork, as DJI Studio presents it. The editor
> is orbis-reframe's UI today; this is what it should grow toward where the
> two differ. The prose notes it supersedes are in
> [`../reference/djistudiocraft/UI-REFERENCE.md`](../reference/djistudiocraft/UI-REFERENCE.md).

The target interface, from 20 annotated DJI Studio screenshots (2026-11-11).
This is the authoritative reference for what the editor should become; it
supersedes the prose notes in `UI-REFERENCE.md` (under `reference/djistudiocraft/`) where they differ.

## Top-level chrome
- **Left toolbar tabs:** File, Music, Filter, Effects, Camera (tricks),
  Transitions, Text, Dashboard.
- **Top-right:** Export Task, Export.
- **Center:** Player (preview) with a single-view / grid toggle.
- **Right:** context inspector with tabs (see below).
- **Bottom:** multi-track timeline.

## Loading (img 1)
- Open an `.OSV` and it plays immediately in a Panoramic Video preview window
  with transport controls (play/pause, step, fullscreen) and a scrubber.
- A **Create** button turns the opened clip into an editable project.

## Inspector tabs (right panel)
The inspector is **tabbed**, not one flat panel. Tabs: Video, Image, Adjust,
Beauty, Speed.

### Video tab (img 2, 20)
- **Select Perspective:** Camera View (replay the pilot's recorded flight
  view), Free View (reframe freely), Direction Lock (hold the camera's
  direction during capture). "Apply to All" button.
- **RockSteady** stabilization toggle.
- **Horizon Leveling** toggle.
- **Manual Framing** presets: Asteroid, Ultra-wide, Wide, Dewarp.
- **Zoom** (e.g. 183°), **FOV** (e.g. 60°), **Correction angle** (e.g. 1.00).
- **Pan angle, Tilt angle, Roll angle** (degrees).
- **Intelligent Tracking** button.
- **Keyframe Animation** with "Apply to all".

### Image tab (img 13)
- **Flight Mode:** FPV Mode / Follow Mode toggles.
- **Chromatic Aberration Control:** correction toggle + reduction level.
- **Stitching Optimization:** Optical Flow Stitching, High-Precision Smart
  Stitching, Stitching Correction.
- **Distortion Optimization:** Dewarp toggle.

### Adjust tab (img 4)
- Sub-tabs: **Basic / Color Wheel / Filter**.
- **Color Recovery** with a Name dropdown (Auto Detect D-Log M, etc.).
- **Basic Correction — Light:** Exposure, Luminance, Contrast, Highlights,
  Shadows, Whites, Blacks.
- **Color:** Temperature, Tint, Saturation, Vibrance.
- "Apply to All".

### Beauty tab (img 6)
- Presets Min/Default/Med/Max; Skin (Smoothness, Brightness, Tan); Face (Slim
  Face, Enlarge Eyes, Remove dark circles, Slim Nose, Resize Mouth, Whiten
  Teeth, Slim Chin); Makeup. *(Low priority for drone footage.)*

### Speed tab (img 18)
- Constant Speed / Variable Speed; a speed slider (0.5x..) for slow motion.

## Camera tricks (img 7, 8, 9, 14)
A library of preset camera moves, categorized: Recommended, Classic, Creative,
Asteroid, Forward, Rotation, Flight. Named moves include CW/CCW Sky Flip, Pull
& Rotate, 360 Orbit, Push In, Tilt Down, Asteroid unfold, Half Asteroid, Fwd
to Selfie. Each is a canned keyframe animation; dropping one onto a clip adds
an orange trick segment on the timeline (img 7, 14). Moves can be favorited.

## Filters (img 5, 12)
LUT-based filter library: Pro, Urban, Nature, Portrait, Retro Movie, Food,
Movie, Night, Texture, Black & White, Aerial. Applied with an Intensity slider
(0..100). Custom LUT import.

## Effects (img 10)
Visual effects: Basic, Split, Light, Retro — Snow, Autumn vibe, Red/Falling
leaves, Mini world, Blinds, Dissolve, etc.

## Transitions (img 19)
Between-clip transitions: Slide, Zoom Out/In, Stretch, Twist Roll, Roll, etc.

## Text (img 15, 16)
Text overlays that **free-rotate in 3D** on the sphere (img 15 shows "Default
text" skewed in perspective on the road). Font, Size, Style, Color, Alignment,
Opacity, Stroke. Text is its own timeline track.

## Music (img 3, 17)
Built-in library (Recommend, Travel, Vlog, Romantic, Cheerful, Tempo, Cute,
Relaxed) + Import. Music is a waveform track under the video, multitrack (img
3). Sound effects category too.

## Timeline (img 3, 7, 15)
Multiple stacked tracks: Text (T), Video (the clip, showing per-lens thumbnails
with labels "Camera View", "Color Recovery D-LOG M", "Filter Vista Pro"), and
Music (waveform). Camera-trick keyframe segments appear inline on the video
track. Standard trim/split/cut tools in the toolbar.

## Export (img 11)
- **Export as:** Videos (reframed flat) or Panoramic Video (full 360).
- Media Type, File Name, Storage Location.
- **Export Range:** Full Timeline / selection.
- **Export Method:** Single Clip / etc.
- **Export Preset:** Custom.
- **Resolution:** up to 8K 7680×3840.
- **Frame Rate:** up to 60 fps.
- **Bitrate:** High / etc.
- **Color Space:** SDR Rec709.
- Note shown: exporting panoramic discards framing, transitions, and overlays.
- Live estimated file size (e.g. 3236.5 M for 8K/60 full timeline).
