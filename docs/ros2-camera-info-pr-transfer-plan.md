# ROS 2 CameraInfo PR transfer plan

This is the working handoff plan for moving the camera-specific improvements
from `cam-wip` back to `pr/ros2-camera-info`, the source branch for
[ouster-lidar/ouster-ros#564](https://github.com/ouster-lidar/ouster-ros/pull/564).
It records the branch state as of 2026-09-03 (America/Chicago). Recheck remote
heads before carrying out the commands below.

This document is for the `cam-wip` branch only. It is not part of the proposed
camera PR transfer set.

## Current state

- Tested camera implementation tip on `cam-wip`: `e8ec0a5`
- Camera PR branch tip: `8bd6636`
- Camera PR base branch tip: `origin/ros2` at `d72664f`
- PR #553 has merged as `d72664f`; the camera branch has already been rebased
  directly onto it.
- PR #564 contains one camera feature commit on top of `origin/ros2`.

Do not merge `cam-wip` wholesale into the camera branch. It contains packet,
replay, timestamp, lifecycle, and point-cloud changes that would unnecessarily
broaden PR #564.

## Camera improvements completed on `cam-wip`

The camera work now includes:

- calibrated pinhole `depth_image` and `depth_image2` topics;
- `32FC1` optical-axis depth in metres, with `NaN` for invalid pixels;
- beam-origin compensation using the SDK XYZ lookup table;
- pixel-centred principal points and independently configurable horizontal and
  vertical focal lengths;
- automatic vertical FOV fitting that accounts for panel pitch and asymmetric
  lidar beam angles;
- nearest-beam support boundaries based on the actual edge-beam spacing;
- source selection and depth projection consistent with a nonzero configured
  column-zero azimuth;
- validation and masking of normal or seam-wrapped metadata `column_window`s;
- optional physical cropping of unsupported pinhole borders, enabled by
  default, with full-panel `CameraInfo` calibration and an exact ROI describing
  the emitted image;
- truthful full-resolution panorama ROI when `os_image` still emits a complete
  zero-padded raster whose row-dependent destaggered support cannot be
  represented by one rectangle;
- matching timestamps and optical frame IDs across images and `CameraInfo`;
- finite-parameter, sensor-geometry, allocation, wrapping, and buffer-access
  validation; and
- documentation distinguishing radial `range_image` from metric
  `depth_image` and showing a `depth_image_proc` invocation.

The packet-backed launch test checks the real `os_pinhole` and `os_image`
executables, including:

- default opt-out and explicit opt-in behavior for panorama `CameraInfo`;
- pinhole range, depth, dual-return depth, and `CameraInfo` messages;
- independent `fx`/`fy`, half-pixel principal points, ROI, encodings,
  dimensions, steps, timestamps, and frame IDs;
- pitched optical-frame orientation on `/tf_static`;
- rejection of undersized packets;
- idempotent metadata handling and recovery after malformed metadata; and
- parsing of all modified camera-related launch files.

Humble does not provide `launch_testing_ros.actions.EnableRmwIsolation`, so the
test uses that action conditionally. Its node names and topics remain dedicated
when running on Humble.

## Verification snapshot

The `e8ec0a5` tree passed:

- all 55 C++ gtests in Release mode;
- all 55 C++ gtests with Debug assertions enabled;
- all packet-backed launch-test cases in both builds;
- `flake8` and byte-compilation for `camera_nodes_launch_test.py`;
- the seven existing `cardinal_direction` estimator tests; and
- `git diff --check`.

The focused camera tests cover normal and wrapped column windows, cropped and
padded output, independent FOV intrinsics, asymmetric pitched auto-FOV,
cardinal panel axes, metric depth, and sampled-lidar reprojection with a
nonzero column-zero azimuth.

## Commit disposition

| `cam-wip` commit | Camera PR action | Reason |
| --- | --- | --- |
| `8bd6636` | Already present | The rebased camera feature commit at the tip of `pr/ros2-camera-info`. |
| `b1da874` | Include | Calibrated pinhole depth implementation, unit tests, configuration comments, and documentation. |
| `f16accf` | Include with adaptation | End-to-end camera launch test and test dependencies. |
| `972287a` | Include with `f16accf` | Humble compatibility for the launch test. |
| `e8ec0a5` | Include after the three commits above | Geometry, FOV, column-window, cropping, ROI, panorama-honesty, and regression fixes from this audit. |
| `4c2e2c4` | Skip | This transfer-plan document only. |

The other commits between `8bd6636` and `b1da874` are unrelated `cam-wip`
hardening work and should not be pulled into PR #564.

## Recommended transfer procedure

### 1. Refresh and protect the target branch

Fetch the remote, switch to the dedicated camera branch, and create a fresh
backup ref before rewriting it:

```bash
git fetch origin
git switch pr/ros2-camera-info
git branch backup/pr-ros2-camera-info-before-camera-fixes
```

Verify the expected ancestry before proceeding:

```bash
git merge-base --is-ancestor d72664f pr/ros2-camera-info
git log --oneline origin/ros2..pr/ros2-camera-info
```

The second command should initially show only `8bd6636` (or its current
patch-equivalent if the branch has since been rewritten).

### 2. Apply calibrated depth support

```bash
git cherry-pick -x b1da874
```

The configuration-file hunk may need a small manual resolution because its
`cam-wip` context contains the later metadata-file parameter. Retain the target
branch's parameter set and bring across only the new range/depth behavior and
documentation unless metadata-file loading is intentionally added separately.

### 3. Port the integration test without broadening the PR

Apply the launch-test commits without committing immediately so their Humble
compatibility and target-specific adaptations can form one coherent commit:

```bash
git cherry-pick --no-commit f16accf 972287a
```

Before committing, adapt `camera_nodes_launch_test.py` so it does not depend on
the unrelated `cam-wip` metadata-file or packet-QoS work:

1. Remove `metadata` and `lidar_packet_reliable` from the common node
   parameters.
2. Remap both `os_image` nodes' `metadata` topic to
   `/camera_test/metadata`, matching the pinhole node's namespace.
3. Wait for all three metadata subscriptions before activation.
4. Publish the fixture metadata from the test.
5. Then wait for all three lidar-packet subscriptions and output publishers.
6. Publish the same metadata once more to exercise the idempotent path.
7. Preserve the malformed-metadata recovery check and conditional
   `EnableRmwIsolation` behavior.

Using system-default QoS keeps the test's reliable packet publisher compatible
without importing the later `lidar_packet_reliable` parameter. Commit the
resulting target-specific test, retaining `f16accf` and `972287a` in its commit
message for traceability.

### 4. Apply the geometry and ROI audit fixes

```bash
git cherry-pick -x e8ec0a5
```

Resolve the launch-test hunk against the adapted test from step 3, retaining
the pitched panel, independent `fx`/`fy`, principal-point, ROI, and TF checks.
Do not copy this transfer-plan document into the PR branch.

### 5. Validate the rebased branch

Use a clean build directory and verify at least:

- Release build and complete test suite;
- Debug build with assertions enabled and the complete test suite;
- `BUILD_PCAP=OFF` configuration;
- Python lint for the launch test;
- all modified launch files with `ros2 launch ... --show-args`; and
- `git diff --check`.

Inspect the final history and diff against the actual PR base:

```bash
git log --oneline origin/ros2..HEAD
git diff --stat origin/ros2...HEAD
git diff --check origin/ros2...HEAD
```

The resulting range should contain the original camera feature, calibrated
depth support, camera tests, and the geometry/ROI audit fixes—not the unrelated
`cam-wip` hardening commits.

### 6. Update PR #564

Review the range locally before updating the rewritten remote branch. When it
is ready, use lease protection:

```bash
git push --force-with-lease origin pr/ros2-camera-info
```

After pushing, confirm the PR diff and commit list, wait for every supported
ROS/RMW job, update the PR description with the depth and ROI semantics, and
request review again.

## Downstream `cardinal_direction` follow-up

This is not part of the ouster-ros PR, but the downstream estimator must be
updated before enabling cropped panels in that stack:

- derive its working camera matrix from rectified `P`, subtracting ROI offsets
  and applying binning, instead of consuming the full-resolution raw `K`
  unchanged; and
- obtain the optical-to-output-child rotation from TF instead of duplicating
  only panel yaw, so configured pitch and any lidar-to-base mounting transform
  are represented correctly.

It should also reset a panel's tracking state if its image dimensions,
effective camera matrix, or frame ID changes after a metadata/pipeline rebuild.
