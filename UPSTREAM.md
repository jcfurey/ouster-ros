# Fork provenance

This is a fork of [`ouster-lidar/ouster-ros`](https://github.com/ouster-lidar/ouster-ros)
— the official Ouster ROS 2 driver.

| Field           | Value                                              |
|-----------------|----------------------------------------------------|
| Upstream remote | `https://github.com/ouster-lidar/ouster-ros` (`upstream`) |
| Fork remote     | `https://github.com/jcfurey/ouster-ros` (`origin`) |
| Tracked branch  | `ros2` (both)                                      |
| Last sync base  | `186c388` — *[ROS2] Fix the flags field, enable the variable columns per packet (#537)* |

The fork is **upstream-trackable** — every patch lives as a commit on top of
the upstream `ros2` history with no rewrites or squashes, so a `git rebase`
against a refreshed upstream is the supported sync path. Verify with:

```bash
git rev-list --count upstream/ros2..HEAD   # commits ahead of upstream
git rebase upstream/ros2                   # pick up new upstream work
```

## Local divergence

Seven commits sit on top of upstream `ros2`. Read in the same order they apply:

| Commit    | Category   | Subject                                                              |
|-----------|------------|----------------------------------------------------------------------|
| `34f13f3` | chore      | bump ouster-sdk submodule                                            |
| `fde48f6` | feat       | publish CameraInfo from os_image node                                |
| `1356dc6` | fix        | silence pedantic/deprecation warnings for C++20 pedantic build       |
| `a28cfc4` | feat       | gate CameraInfo publication and let frame_id be configured           |
| `c2127fc` | fix        | os_image_node: distortion_model `equidistant` → `plumb_bob`          |
| `88afb09` | feat       | populate `CameraInfo.roi` from sensor `column_window`                |
| _new_     | feat       | os_pinhole node + PinholeProcessor: cardinal pinhole panels          |

### Pinhole panel publisher (newest commit)

`os_pinhole` is a new node co-located with `os_image` in this fork. It
consumes the same `lidar_packets` + `metadata` stream and produces N
pinhole rectified panels — by default front / left / rear / right at
yaw 0 / 90 / 180 / 270° about `lidar_frame` +Z — with each panel carrying
its own optical-convention `frame_id` (`{ns}/panels/{name}_optical_frame`)
broadcast via `StaticTransformBroadcaster`. Each panel publishes
`range_image`, `signal_image`, `reflec_image`, `nearir_image`, and a
matching pinhole `CameraInfo`, so Foxglove/Trillium and standard
`image_proc`-style consumers see proper pinhole cameras.

Why a separate node (vs. cropping `os_image`):
`os_image` produces the natural cylindrical/equirectangular panorama —
the right thing for pixel-perfect, single-source-of-truth metadata. It
is NOT a pinhole camera, so cropping it and lying about the intrinsics
breaks geometric consumers. `os_pinhole` does the resample-with-correct-K
in a self-contained component instead.

Implementation: `PinholeProcessor` (header-only) builds a per-output-pixel
LUT mapping `(panel_row, panel_col)` → `(source_row, source_destaggered_col)`
once at metadata reception. Per-scan work is field decode + auto-exposure
on the full panorama (so all panels share dynamic range) + LUT-sample into
each panel's `Image` buffer. Beam altitudes drive nearest-neighbor row
selection; columns map linearly from azimuth, with an
`azimuth_offset_columns` knob for per-platform constant offsets.

Configuration (see `src/settings/params/sensor/lidar/ouster/lidar0.yaml`):

| Parameter                   | Default                                | Notes                                              |
|-----------------------------|----------------------------------------|----------------------------------------------------|
| `panel_names`               | `["front","left","rear","right"]`      | Source of truth for panel count                    |
| `panel_yaws_deg`            | `[0,90,180,270]`                       | CCW about `lidar_frame` +Z                         |
| `panel_pitches_deg`         | `[0.0]`                                | Per-panel array OR singleton; `+pitch` → up        |
| `panel_hfovs_deg`           | `[90.0]`                               | Per-panel array OR singleton broadcast             |
| `panel_widths`              | `[256]`                                | Per-panel array OR singleton broadcast             |
| `panel_heights`             | `[0]`                                  | 0 → auto-fit lidar VFOV at square pixels           |
| `panel_vfovs_deg`           | `[0.0]`                                | 0 + height>0 → derived from height/fy              |
| `azimuth_offset_deg`        | `0.0`                                  | Az(deg) of destaggered col 0 in `lidar_frame`      |
| `optical_frame_template`    | `"{ns}/panels/{name}_optical_frame"`   | `{ns}` = `lidar_namespace`, `{name}` = panel name  |
| `parent_frame`              | `os_lidar`                             | Static-TF parent (typically `lidarN/lidar_frame`)  |
| `lidar_namespace`           | `lidar0`                               | Substituted for `{ns}` in topic + frame templates  |

Static TF rotation is composed in tf2 as `q = R_z(yaw) * R_y(-pitch) * R_b2o`,
where `R_b2o` (rpy = `-π/2, 0, -π/2`) maps body axes (`+X` forward, `+Y`
left, `+Z` up) to the standard ROS optical convention (`+Z` forward, `+X`
right, `+Y` down). Pitch is applied in the post-b2o panel-body frame so a
positive `pitch_rad` always points the panel up regardless of yaw.

Gated per-sensor in `bringup_erdc/lidar/launch/types/ouster.launch.xml`
by `run_pointcloud_to_pinhole`, which reads `${lidar_name_index}_run_pinhole`
(default False). Default-off; flip the env var to enable per-LiDAR.

## CameraInfo divergence from upstream noetic PR

Upstream merged a `camera_info` publisher on its **noetic** branch only
(`ouster-lidar/ouster-ros#441`, two commits: `66ad471` "Define a camera_info
topic" and `c769ff87` "Set the right message type for the topic"). It was
never ported to the `ros2` branch, so `fde48f6` re-implements the feature
here and `a28cfc4` / `c2127fc` then improve on it. The implementations
diverge on six points — every divergence is intentional and a strict
upgrade.

| Aspect              | Upstream noetic (`c769ff87`)                          | Our `ros2` (`os_image_node.cpp`)                                     |
|---------------------|-------------------------------------------------------|----------------------------------------------------------------------|
| Topic name          | `camera_info`                                         | `camera_info` *(matches)*                                            |
| QoS / latching      | Latched, queue=1                                      | Sensor-data (or system-default) QoS, **per-frame republish**         |
| Publish cadence     | Once at startup, in metadata callback                 | Every frame, header.stamp synced to the image                        |
| `frame_id`          | Hardcoded `"os_lidar"`                                | Param `sensor_frame` (default `os_lidar`)                            |
| Gating              | Always on                                             | Param `publish_camera_info` (default true)                           |
| `fx`, `cx`          | `W/(2π)`, `W/2`                                       | `W/(2π)`, `W/2` *(matches)*                                          |
| `fy`                | **`H/(2π)`** — implicitly assumes 2π VFOV             | `H / vfov_rad`, where `vfov_rad` is derived from `beam_altitude_angles` |
| `cy`                | `H/2`                                                 | `H/2 − mean_alt·fy` — corrects for asymmetric beam pattern           |
| Degenerate fallback | None                                                  | If `beam_altitude_angles.size() < 2`, log WARN and fall back to upstream's 2π formula |
| `distortion_model`  | `"equidistant"` with **5 D coefficients**             | `"plumb_bob"` with 5 zero D coefficients                             |
| Timestamp           | `ros::Time::now()` at startup                         | `image->header.stamp` per frame                                      |

Notes on the more substantive deltas:

- **`fy` from beam-altitude angles.** A 16-beam OS1 has a vertical FOV of
  ~33°; upstream's `H/(2π)` formula treats the panorama as if it were a
  full vertical sphere, giving `fy` ~10× too small and producing K matrices
  that no `image_geometry::PinholeCameraModel` consumer can use sensibly.
  Reading the actual `beam_altitude_angles` from sensor metadata gives the
  true VFOV; the per-beam non-uniformity that a single `fy` cannot capture
  is small enough to be useful for projection / overlay work.

- **`cy` shift for asymmetric beams.** Ouster sensors aren't symmetric about
  0° elevation (e.g. OS1-16-A0 is biased downward). `cy` is the row whose
  ray points at horizon; computing it from the mean of min/max altitudes
  matches that geometry rather than just centering on the image.

- **`plumb_bob` over `equidistant`.** See the dedicated subsection below —
  the short version is that **neither** REP-104 distortion model is correct
  for an equirectangular 360° image, and `plumb_bob` is the least-wrong
  label for our K matrix.

- **Per-frame vs. latched.** The one dimension where upstream's choice has
  any merit: a late-joining subscriber gets one CameraInfo immediately on a
  latched topic. Our 10 Hz per-frame cadence means a late subscriber waits
  at most ~100 ms for the next CameraInfo, which is fine for `image_proc`-
  style consumers and avoids the QoS-mismatch landmines that
  `transient_local` durability creates under `rmw_zenoh_cpp` (see
  `feedback_zenoh_qos.md`). If strict late-join parity ever matters, add
  `transient_local` durability to the `camera_info` publisher *only* — keep
  the image publishers on `sensor_data`.

### Distortion model: why `plumb_bob`, not `equidistant`

The published K matrix encodes `fx = W/(2π)`, which makes column index a
**linear function of azimuth** (`azimuth = (v − cx)/fx` rad). That is an
**equirectangular (cylindrical) projection** — not pinhole, not Kannala-
Brandt fisheye. REP-104 has no `distortion_model` value for it
(`omnidir` / `sphere` exist in CV literature but are not standardized in
ROS), so the field has to be set to *something* that lies in a known
direction. With `D = zeros`, no distortion math is applied either way; the
label only tells consumers how to *interpret* the K matrix and which
rectification path to dispatch.

**What each label dispatches in `image_pipeline`:**

| Label                | Rectify dispatch                                  | Behaviour with our K and `D=0`                                          |
|----------------------|---------------------------------------------------|-------------------------------------------------------------------------|
| `plumb_bob`          | `cv::initUndistortRectifyMap` (Brown-Conrady)     | Identity remap — published image is the rectified image. **Safe.**      |
| `equidistant`        | `cv::fisheye::initUndistortRectifyMap`            | Expects a fisheye "new K" from `estimateNewCameraMatrixForUndistortRectify`; with our equirectangular K it over-undistorts or refuses the calibration outright |
| `rational_polynomial`| 8-coefficient Brown-Conrady                       | Same identity-remap result as `plumb_bob` with our `D` shape, but consumers default-expect 5 coeffs — extra friction for nothing |
| *(empty)*            | Implementation-defined                            | Several ROS consumers default-assume `plumb_bob`; many bail out         |

`equidistant` is also internally inconsistent in upstream: the Kannala-
Brandt model expects 4 distortion coefficients (k1..k4) but upstream
populates 5 zeros.

**Where `plumb_bob` does lie.** A naive
`image_geometry::PinholeCameraModel::projectPixelTo3dRay` consumer computes
`ray = ((v − cx)/fx, (u − cy)/fy, 1)` and implicitly treats `(v − cx)/fx`
as `tan(azimuth)` rather than the azimuth itself. That is only accurate
for small angles near `cx` (forward direction); at ±90° from forward the
error is large. The same lie would apply under `equidistant` — the
underlying K is *not* a pinhole intrinsic in the first place.

**Practical contract for consumers:**

| Consumer pattern                                                                 | Works under `plumb_bob`? |
|----------------------------------------------------------------------------------|--------------------------|
| Pipe through `image_proc::RectifyNode` then process the rectified image          | Yes — identity remap     |
| Project 3D LiDAR points into the image with `azimuth → column` math themselves   | Yes — uses K linearly    |
| Treat columns as azimuth bins via `azimuth = (v − cx)/fx`                        | Yes — label is ignored   |
| Call `PinholeCameraModel::projectPixelTo3dRay` blindly at wide angles            | **No** — fisheye-style mistake under either label |
| Pipe into a fisheye-aware library that *requires* `equidistant`                  | **No** — would need an override; the library is misclassifying the image |

The empirical signal that drove `c2127fc`: with `equidistant`,
`lidar_image_panels` projected misaligned overlays; with `plumb_bob`, the
same code path produced correct overlays. That is the load-bearing
downstream consumer in this workspace.

**When to override.** If a future deployment needs a different label for a
specific consumer (e.g., a fisheye-aware VO package), the right move is to
expose a `distortion_model` ROS parameter on the node (~5 LoC) so each
deployment can pick its poison without recompiling, rather than flipping
the default for everyone.

## Workspace integration notes

- The image node is launched per-LiDAR through
  `src/bringup/bringup_erdc/lidar/launch/ouster.launch.xml` (gated by the
  per-sensor `robot_lidarN_run_image_node` env flag in `10-lidars.bashrc`).
- `publish_camera_info` and `sensor_frame` are composed into the image-node
  parameter file at launch time; default frame name follows the
  `${prefix}_lidar` convention from the per-LiDAR namespace.
- The `plumb_bob` choice is load-bearing for the `lidar_image_panels`
  package — do not revert to `equidistant` without revisiting that
  consumer's rectification pipeline.
- The corrected `fy` from `beam_altitude_angles` (commit `fde48f6`) is
  similarly load-bearing for `lidar_image_panels`'s panel auto-height
  calculation — reverting to the upstream `H/(2π)` formula would inflate
  the panels' computed source vFOV by ~10× on the OS1-16.
- The Ouster driver is **source-built** here (the apt
  `ros-jazzy-ouster-ros` binary is no longer installed in the image); see
  the `Dockerfile` if you need to disable the source build temporarily.

## Out of scope: panel splitting / virtual cameras lives downstream

A recurring question is whether this fork should also emit cardinal-
direction "virtual camera" panels (front/left/rear/right pinhole sub-
images) directly from the image node. The workspace's answer is **no** —
that feature is implemented as a separate downstream node,
`src/packages/perception/lidar_image_panels`, and gated by the
`run_lidar_image_panels` env flag. Reasons in summary; full rationale in
that package's `README.md`:

- Keeps this fork **upstream-trackable** (small surgical patches) rather
  than promoting it to a structural divergence no upstream PR would land.
- The splitter works on **any** equirectangular LiDAR source
  (`ouster-ros` on hardware, `gz_sensors_ouster` in sim, etc.), not just
  this driver — vendor independence would be lost by in-driver
  implementation.
- Driver job (publish raw data) is conceptually distinct from perception/
  visualization (re-project into virtual cameras). ROS already enforces
  this split for camera drivers vs. `image_proc`.
- The latency win (~1–2 ms saved on a publish/subscribe round trip) is
  not measurable on the workspace's i7 NUC at 10 Hz / 16 beams.

The architectural boundary the workspace settled on:
**this driver publishes the raw equirectangular panorama + correct
`CameraInfo`; everything that re-projects, slices, crops, or virtualizes
views happens downstream.**

## Sync procedure

```bash
# Inside src/packages/drivers/ouster-ros:
git fetch upstream
git log --oneline upstream/ros2..HEAD               # what we have on top
git log --oneline HEAD..upstream/ros2               # what's new upstream
git rebase upstream/ros2                            # rebase the fork
# Resolve conflicts, then update the "Last sync base" line above.
git push origin ros2 --force-with-lease             # publish to fork
```

After syncing, update the **Last sync base** line in this file with the new
merge-base commit and a one-line label, and commit alongside the rebase.
