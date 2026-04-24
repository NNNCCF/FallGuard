# PAGE 4 Pseudo-3D Cloud Design

## Summary

`PAGE 4` should stop behaving like a raw debug text page and become a compact visual point-cloud page for the `128x64` SSD1306 OLED.

The selected direction is a `pseudo-3D wireframe box` that renders radar targets as `stable point clusters`, not as a literal "one point equals one person" scatter plot.

This decision is based on two confirmed constraints:

- The display is `128x64`, monochrome, and too small for dense raw 3D scatter rendering to stay readable.
- In real measurements, one person produces `multiple points` that are `generally clustered and stable`.

The design therefore optimizes for:

- `technical feel`: the page should still look like a 3D radar/point-cloud view
- `readability`: users should understand "where the person is" faster than they understand individual points
- `auto mode`: single-target detail, multi-target simplification

## Goals

- Make `PAGE 4` visually explain target position and shape more clearly than the current numeric layout.
- Preserve a clear `3D-like` presentation without requiring full perspective rendering.
- Support both `single target` and `multi-target` scenes on the same page.
- Reduce visual noise from empty frames, transient drops, and point jitter.

## Non-Goals

- This page is not intended to be a literal scientific point-cloud viewer.
- This page will not render every point in every frame when doing so harms readability.
- This page will not rotate the camera, animate perspective, or implement a true 3D engine.
- This change does not redefine the radar protocol or the cluster semantics coming from `ld6002c`.

## Constraints

- Display: `SSD1306 128x64`, monochrome
- Existing page system rotates every `5s`
- Radar frames may alternate between:
  - populated point-cloud frames
  - empty `target_num=0` frames
- A single person can generate multiple points that form a visually stable cluster
- Multi-target rendering must degrade gracefully instead of becoming unreadable

## Approaches Considered

### 1. Raw 3D Scatter

Draw all points directly in a pseudo-3D scene.

Pros:

- strongest "engineering/debug" feel
- simplest conceptual mapping from cloud data to pixels

Cons:

- too noisy on `128x64`
- visually weak when one person already produces many points
- multi-target scenes degrade quickly

Decision: rejected

### 2. Pure Target Blob / Human Occupancy View

Render only a compact body-like blob or occupancy volume, with minimal point detail.

Pros:

- easiest for non-technical users to understand
- most stable visual output

Cons:

- weaker point-cloud feel
- loses the technical identity the user requested

Decision: not selected

### 3. Pseudo-3D Box with Cluster-Centric Rendering

Render a fixed 3D wireframe box, then draw points as a clustered target with centroid, shadow, and hull.

Pros:

- preserves 3D/point-cloud identity
- readable on small monochrome OLED
- naturally supports single-target detail and multi-target simplification

Cons:

- more rendering logic than text-only layout
- requires careful downsampling and hold logic

Decision: selected

## Chosen Design

### Page Layout

`PAGE 4` is divided into three fixed zones:

- `Top 8 px`: title and mode
- `Middle 48 px`: pseudo-3D scene
- `Bottom 8 px`: compact status line

#### Top line

Display short mode text:

- `CLOUD 1T` for single-target mode
- `CLOUD nT` for multi-target mode
- `CLOUD 0T` or equivalent for no-target state if needed

#### Middle scene

Render a fixed oblique pseudo-3D observation box.

Inside that box:

- draw cluster points
- draw a centroid cross
- draw a ground shadow projected onto the base plane
- draw a compact cluster hull / outline

#### Bottom line

Keep one short summary line only.

Examples:

- single target: `D1.8 H1.2 P7`
- multi target: `T3 N1.4`
- no target: `NO TARGET`

This line exists to preserve exact numerical context without polluting the scene.

## Projection Model

Use a fixed oblique projection, not a true perspective camera.

Recommended mapping:

- horizontal placement comes mainly from `x`
- depth contribution comes from `y`
- vertical placement comes from `z`

Reference transform:

- `screen_x = center_x + x * sx - y * sy`
- `screen_y = floor_y - z * sz - y * dy`

This gives:

- readable left/right positioning
- a clear but stable depth cue
- direct vertical interpretation for height

The projection should stay constant across frames. No camera movement or angle switching.

## Rendering Rules

### Single-Target Mode

Single-target mode is entered when the current cloud represents one visible target cluster.

Render:

- sampled cluster points
- centroid cross
- ground shadow
- compact hull

Bottom line:

- distance
- height
- point count

The user should perceive this as `one person-shaped point cluster in space`.

### Multi-Target Mode

Multi-target mode is entered when more than one visible target exists.

Do not render every point for every target.

Render:

- one compact blob/hull per target
- one centroid marker per target
- optional tiny target ID if legible

Bottom line:

- target count
- nearest target distance

The user should perceive this as `how many targets exist and where they roughly are`, not a detailed cloud.

### No-Target Mode

Keep the wireframe box visible.

Behavior:

- hold the last valid populated scene briefly
- then transition to an empty box with `NO TARGET`

This is preferable to instantly clearing the entire page on the first empty frame.

## Anti-Jitter Behavior

The display must remain stable even though cloud frames can jitter or temporarily disappear.

Rules:

- retain the last non-empty cluster view for a short hold window
- slightly smooth centroid movement
- keep sampling stable enough to reduce sparkle
- do not over-smooth so much that motion or a fall appears delayed

The existing point-cloud hold behavior should be extended to support the new scene logic.

## Cluster Semantics

The page must assume:

- one person may produce multiple points
- those points usually form a stable cluster

Therefore:

- visual emphasis is on the `cluster as a target`
- individual points are secondary cues
- no design element should imply `one point == one person`

## What To Avoid

- dense raw rendering of all points in multi-target scenes
- too many labels or metrics
- rotating or animated camera movement
- full perspective simulation
- per-point text labels
- geometry so dense that the 3D box becomes unreadable

## Implementation Notes

Recommended internal structure:

- keep current page-selection structure in `oled_display`
- refactor `PAGE 4` into dedicated helpers:
  - cloud mode selection
  - projection
  - single-target rendering
  - multi-target rendering
  - compact metrics rendering

Rendering should remain deterministic and fast enough for the current display refresh cadence.

## Testing Plan

### Functional

- single target with multiple clustered points renders as one cluster-centered scene
- multi-target scene falls back to simplified aggregate view
- empty cloud frames do not immediately erase a valid scene
- long empty interval returns to `NO TARGET`

### Readability

- a user can distinguish:
  - no target
  - one target
  - multiple targets
- a user can infer rough left/right, near/far, high/low position from the scene

### Stability

- cluster display does not flicker excessively when points move slightly
- empty interleaved frames do not cause rapid page collapse
- the bottom status line remains legible in both single-target and multi-target mode

## Open Decisions Already Resolved

Resolved in this design:

- optimize for `auto mode`
- use `pseudo-3D box`
- treat a person as a `cluster`
- prefer `single-target detail` and `multi-target simplification`

No unresolved placeholders remain in this version.
