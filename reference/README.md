# Reference material from the earlier repositories

The two repositories this project grew out of were folded in here in
November 2026 and then deleted on GitHub. Nothing under `reference/` is
built or run; it is kept because it holds measurements and work that the
editor may still draw on.

## `djistudiocraft/` — from `pcwizardsinc/DJIStudioCraft`

The earlier attempt, built on [OpenOSV](https://github.com/Kemerd/OpenOSV)
(the open `.OSV` reader and renderer). That repository was OpenOSV's full
history plus seven commits of our own; only those seven are kept, as a
`git format-patch` series against OpenOSV at `2b83aad` (its 0.6.4), with
their messages and dates (`COMMITS.txt`). To rebuild that tree:
`git clone https://github.com/Kemerd/OpenOSV && cd OpenOSV && git checkout 2b83aad && git am ../patches/*.patch`.

What is in them, and why it still matters:

- **0002 — Fedora 45 build support and the Avata 360 levelling fix.** The
  −90° pitch mount for the Avata, measured on real footage (see
  [`FORK.md`](../FORK.md)). OpenOSV upstream does not have it; this is the
  patch to offer them.
- **0005 — VA-API decode, encoder auto-selection.** Hardware HEVC decode on
  AMD through FFmpeg. The editor currently CPU-decodes on the Radeon; this
  is the prior art for fixing that.
- **0003, 0004 — build guides and roadmap** (`BUILD-FEDORA.md`,
  `ROADMAP.md`, also copied beside the patches), and `UI-REFERENCE.md`, the
  notes on how DJI Studio works that the editor's interface follows.
- **0006, 0007 — an editor shell on OpenOSV's GUI** (project model, tabbed
  inspector, live preview). Superseded: orbis-reframe already has all of it.
  Kept for completeness only.
- **0001** is an OpenOSV pull request (#11) that was on our branch, not our
  work; it is here so the series applies in order.

## `ui-mockup/` — from `pcwizardsinc/AvataCraft`

A React / TypeScript browser mock-up of the intended interface, made with
Google Gemini (`ORIGINAL-README.md` is its README, which describes a Qt6
application that never existed in that repository). Useful as a design
reference next to [`INTERFACE-SPEC.md`](../docs/INTERFACE-SPEC.md); not a
build target. Its sample `DangTree.OSV` was synthetic (generated from FFmpeg
test patterns by `scripts/generate_dangtree_footage.py`) and is not kept, nor
is the binary tarball it bundled. Real footage lives outside the repository.
