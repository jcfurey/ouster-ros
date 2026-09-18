# NITROS investigation for cam-wip

Investigated 2026-09-18 against the local `jcfurey/ouster-ros` `cam-wip`
checkout at `ad443b8`. Scope: camera outputs and LiDAR point clouds. This is a
source-based feasibility assessment and proposed implementation sequence;
no Ouster NITROS implementation or performance measurement was made.

**Recommendation: begin with CUDA generation and NITROS publication of pinhole
depth, then extend to the other images.** Add a separate GPU XYZ cloud only
when a downstream consumer can use it. Keep the existing rich Ouster cloud
available for algorithms that require per-point time and other sensor fields.

| Output | Fit with NITROS 4.6 | Recommendation |
|---|---|---|
| Pinhole `depth_image[2]` | `NitrosImage`, `32FC1`; calibrated optical-axis Z in metres | First implementation target |
| Pinhole/panorama display images | `NitrosImage`, existing `mono16` and optional `rgb8` | Supported formats; port resampling before stateful exposure/tone mapping |
| XYZ / XYZ+RGB cloud | `NitrosPointCloud`; packed float XYZ, optionally packed RGB | Separate opt-in output, after identifying a typed GPU consumer |
| Original/native/XYZI/XYZIR/color-point cloud | Stock adapter cannot describe all existing fields | Preserve `PointCloud2`; a lossless GPU representation needs additional design |
| CameraInfo, IMU, metadata, telemetry | Small ordinary ROS messages | Retain current interfaces |

The format assessment is based on NVIDIA's pinned
[image builder](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros/blob/v4.6-0/isaac_ros_nitros_type/isaac_ros_nitros_image_type/src/nitros_image_builder.cpp)
and [point-cloud adapter](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros/blob/v4.6-0/isaac_ros_nitros_type/isaac_ros_nitros_point_cloud_type/src/nitros_point_cloud.cpp).
Image-format support alone does not establish downstream algorithm compatibility.

**Current integration boundaries**

[LidarPacketHandler](../ouster-ros/src/lidar_packet_handler.h) assembles packets
into reusable host `LidarScan` objects and invokes processor callbacks on its
worker thread. GPU transport would still begin after CPU packet reception and
decoding. NITROS does not introduce GPUDirect acquisition or GPU packet decoding.

[PinholeProcessor](../ouster-ros/src/pinhole_processor.h) already precomputes
per-panel source-row/column maps, depth scales and depth offsets. Its depth
sampling operation is a good first CUDA kernel: gather a raw range, compute
`range * scale + offset`, and preserve the existing invalid-depth rules.
Calibration, ROI selection and lookup-table construction can remain on the CPU
and upload when metadata changes. Upload each needed scan field once per scan
and share it across panels and outputs.

[OusterPinhole](../ouster-ros/src/os_pinhole_node.cpp) publishes the result in
`publish_panels()`. This is the initial transport boundary, but GPU production
also requires a device-output representation: merely replacing that publisher
would leave CPU sampling and host image allocation in place.

[ImageProcessor](../ouster-ros/src/image_processor.h) produces panorama images
for both [OusterImage](../ouster-ros/src/os_image_node.cpp) and the combined
[OusterDriver](../ouster-ros/src/os_driver_node.cpp). Reuse one transport helper
across these nodes when extending image coverage. Panorama CameraInfo is not a
substitute for the calibrated pinhole projection expected by a depth-camera
consumer.

[PointCloudProcessor](../ouster-ros/src/point_cloud_processor.h) currently runs
Cartesian conversion on the CPU, composes a PCL cloud, and stages conversion to
`PointCloud2`. A useful GPU cloud implementation would operate directly on scan
ranges and the XYZ lookup table, avoiding those intermediate host structures
for the GPU output. The ordinary path remains necessary where its full schema
is required.

The nodes are registered components already, but the current
[pinhole launch](../ouster-ros/launch/pinhole.launch.py) and
[live launch](../ouster-ros/launch/pinhole_sensor.launch.py) start standalone
executables. Add composition with GPU consumers and intra-process communication;
topic remapping alone does not share device storage between processes. NVIDIA
documents the same-process requirement for the ordinary NITROS path; a NITROS
bridge is a separate integration.
[NITROS system assumptions](https://nvidia-isaac-ros.github.io/v/release-4.6/concepts/nitros/index.html#system-assumptions)

**Version and API decisions**

Pin the first implementation to the buffer-based **4.6.x** API. The source
review used NVIDIA tag `v4.6-0`, commit
`ddeee2f3a1be0342588f57bcbed115d71b6f1db4`.

In that tag, `ManagedNitrosPublisher<T>` wraps `rclcpp::Publisher<T>`, enables
intra-process communication and does not use its `format` argument. Its header
also includes `nitros_type_view.hpp`, which is absent from the tag's tree.
For this integration, use typed rclcpp publishers directly, as the local Lucid
driver does. Do not base the implementation on older negotiated/GXF examples
without qualifying a different release.
[Managed publisher source](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros/blob/v4.6-0/isaac_ros_managed_nitros/include/isaac_ros_managed_nitros/managed_nitros_publisher.hpp)

`NitrosImageBuilder` accepts the three existing image encodings above and
computes a packed row stride. Give it completed device data and a release
callback retaining the allocation owner. Alternatively, use `from_external()`
with the actual producer stream and finalize its write handle before publishing.
The CUDA stream associated with the write must cover the producer's work.
[Builder source](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros/blob/v4.6-0/isaac_ros_nitros_type/isaac_ros_nitros_image_type/src/nitros_image_builder.cpp),
[buffer synchronization](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros/blob/v4.6-0/isaac_ros_nitros/include/isaac_ros_nitros/types/nitros_buffer.hpp)

The stock point-cloud adapter reconstructs only `x,y,z` or `x,y,z,rgb`. It does
not preserve an arbitrary `PointCloud2.fields` description. Its ROS-to-device
conversion infers color from point stride rather than inspecting field names;
feeding an arbitrary PCL/Ouster layout to it is unsafe. Explicitly pack XYZ or
XYZRGB, and never reinterpret intensity as color.
[Adapter source](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros/blob/v4.6-0/isaac_ros_nitros_type/isaac_ros_nitros_point_cloud_type/src/nitros_point_cloud.cpp)

The [point-cloud builder](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros/blob/v4.6-0/isaac_ros_nitros_type/isaac_ros_nitros_point_cloud_type/src/nitros_point_cloud_builder.cpp)
sets `height=1`. For organized XYZ, use the
[type's `from_external()` interface](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros/blob/v4.6-0/isaac_ros_nitros_type/isaac_ros_nitros_point_cloud_type/include/isaac_ros_nitros_point_cloud_type/nitros_point_cloud.hpp)
with explicit width, height and packed strides. Suggested additional topic:
`points_xyz`, plus its second-return counterpart. A richer GPU scan/cloud type
would require a field/timestamp schema, conversion code and matching consumers;
put that behind a demonstrated odometry use case.

**Downstream assessment**

Nvblox is the strongest initial application candidate. Its 4.6 image path uses
NITROS image subscribers, while its LiDAR `pointcloud` subscription is explicitly
`sensor_msgs::msg::PointCloud2`. Consequently, publishing a GPU cloud alone
would still incur host fallback at that input. Its depth input supports floating
point metres, matching the pinhole depth output. This supports testing one depth
panel plus CameraInfo and externally supplied pose/TF first.
[Nvblox subscriber source](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/blob/release-4.6/nvblox_ros/src/lib/nvblox_node.cpp),
[input requirements](https://nvidia-isaac-ros.github.io/v/release-4.6/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/api/topics_and_services.html)

Visual SLAM also subscribes to typed NITROS images, but its documented RGBD
input is uint16 depth with a scale factor. Transport support therefore does not
make the existing float depth a drop-in RGBD input. Check grayscale encoding,
explicit depth conversion, invalid values, timing and calibration before making
it an acceptance target.
[Visual SLAM subscriptions](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/release-4.6/isaac_ros_visual_slam/src/visual_slam_node.cpp),
[RGBD input contract](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html#ros-topics-subscribed)

For either consumer, validate these Ouster-specific semantics:

- `range_image` is radial range encoded in 4 mm counts; `depth_image` is optical
  Z in metres. Do not relabel range as depth.
- Cropped panels retain full-panel intrinsics and express the crop in
  `CameraInfo.roi`. Confirm the consumer applies ROI; otherwise provide an
  explicit calibration adapter or start with an uncropped, supported panel.
- Pinhole resampling can duplicate/omit returns. Back-projected pixel-centre
  XYZ is not identical to original LiDAR XYZ.
- A scan header does not make all rotating-LiDAR samples simultaneous. NITROS
  adds neither deskew nor a stereo baseline between virtual panels.

These properties are described by the current
[pinhole implementation](../ouster-ros/src/pinhole_processor.h) and
[driver documentation](../README.md#pinhole-panels).

**Proposed implementation sequence**

1. Establish an application baseline using the existing `sensor_msgs/Image`
   outputs and a typed NITROS consumer. The image type adapter already uploads
   ordinary ROS images, so this can establish correctness before driver changes.
   Compare in the intended component topology. Measure CPU time, latency,
   drops and transfer volume; adding a host-to-GPU relay alone is not evidence
   of a performance gain.
2. Add an optional pinhole CUDA/NITROS backend, initially one `32FC1` panel and
   one return. Reuse CPU calibration/LUT creation, upload ranges once, and write
   depth directly into independently owned device output. Publish CameraInfo
   with matching stamps and frames. Validate typed GPU delivery and ROS fallback.
3. Extend to all panels, both returns and selected display/RGB outputs. Preserve
   the existing exposure, beam-uniformity correction and RGB tone-mapping
   semantics; keeping those operations on the CPU initially is a valid staged
   design, with its remaining costs measured explicitly. Extend the shared image
   publisher to panorama nodes if the workload needs them.
4. Add direct CUDA XYZ generation and a separate `points_xyz` output for a
   confirmed typed consumer. Match organization, destaggering, filtering,
   transforms, invalid points and return selection. Evaluate a richer type only
   if consumer requirements include per-point time or other Ouster channels.

A reasonable packaging choice is opt-in `OUSTER_ENABLE_CUDA` and
`OUSTER_ENABLE_NITROS` CMake options, with a CPU default and a runtime backend
selection. Package manifest conditions must also keep rosdep/colcon ordering
correct; a CMake switch alone does not make manifest dependencies optional.
Require and version-check the image type package for image support, and the
point-cloud type package when implementing cloud support. Reject an explicitly
requested unavailable backend. The NVIDIA image type also depends on VPI.
[Image package build](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros/blob/v4.6-0/isaac_ros_nitros_type/isaac_ros_nitros_image_type/CMakeLists.txt)

**Ownership and validation requirements**

Reusable scan slots and the current reusable `sensor_msgs::Image` buffers cannot
be retained as mutable storage for asynchronous GPU consumers. Finish an upload
before recycling its source, or give the transfer owned staging storage. Retain
device outputs and calibration generations until dependent CUDA work completes.
Stop callbacks and join processing before tearing down the corresponding GPU
resources, including during metadata replacement and shutdown.

Bound retained frames and bytes across all panels/channels/returns, and report
budget drops and transfer counts. Consumer completion events must release the
budget, not just the return from `publish()`. ROS viewers and bag recorders invoke
device-to-host adaptation; test both a GPU-only graph and a mixed graph.
[Image adaptation](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros/blob/v4.6-0/isaac_ros_nitros_type/isaac_ros_nitros_image_type/src/nitros_image.cpp),
[buffer ownership](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros/blob/v4.6-0/isaac_ros_nitros/include/isaac_ros_nitros/types/nitros_buffer.hpp)

Acceptance checks should cover CPU/GPU numeric parity, exact invalid masks,
both returns, calibrated ROI/window/seam cases, RGB profile gating, metadata
recovery, retained outputs across reconfiguration and destruction, and delayed
CUDA consumers. Prove same-process device-pointer identity; successful RViz
display only proves ROS fallback. For clouds, assert the complete field schema
on both the existing rich topic and new XYZ topic.

Use the existing pinhole and camera-node regression cases as the geometry and
lifecycle reference. The workspace also has recorded OSDome packets and metadata
under `results/ouster_osdome_20260916`, suitable for a reproducible replay fixture.
Benchmark the existing CPU path, CPU output with NITROS adaptation, and direct
CUDA output with NITROS under the same consumer load. No speedup is established
by this investigation.

**Reusable local work and environment**

The neighboring `src/lucid_camera_driver` checkout already implements the
4.6 typed-rclcpp pattern, image release callbacks, organized XYZ publication,
output budgets and GPU/ROS mixed-subscriber tests. Its `NITROS.md` and
`lucid_camera_driver/test/validation/*NITROS-2026-09-17.md` explain the tested
limits. Adapt these patterns without adding an Ouster dependency on the camera
driver. Its validation used a restricted source-built NVIDIA harness; it does
not qualify a full vendor installation or an Ouster pipeline.

Read-only checks here found Ubuntu 26.04, ROS Lyrical, CUDA toolkit 12.4.131 and
an accessible RTX 3090 with driver 610.57.04. GPU access was confirmed outside
the execution sandbox after the sandboxed query failed. NVIDIA's documented
4.6 x86 target is Ubuntu 24.04 / ROS Jazzy, CUDA 13.2+ and driver 595+.
Use a matching Jazzy/Isaac ROS environment for application qualification;
treat native Lyrical source builds as a separate compatibility experiment.
[NVIDIA platform requirements](https://nvidia-isaac-ros.github.io/v/release-4.6/getting_started/index.html#system-requirements)

This investigation inspected local and upstream source, release metadata,
existing validation records and the host environment. It did not install
dependencies, run a new GPU pipeline, connect to a sensor or alter driver code.
