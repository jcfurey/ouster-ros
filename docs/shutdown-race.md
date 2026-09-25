# Standalone node shutdown ordering

On ROS 2 Lyrical (`rclcpp` 32.0.3), stopping `os_pinhole` after packet replay
intermittently exited with:

```
Unexpected condition: string capacity was zero for allocated data! Exiting.
```

The crash was reproduced with both full and cropped cardinal panels. GDB
placed the failure in `rosidl_runtime_c__String__fini`, called while
`rcl_node_fini` destroyed a cached type description. A separate trace caught
two concurrent calls to `rcl_node_type_cache_unregister_type` for the same
node: `rcl_logging_rosout_fini` on ROS's deferred signal-handler thread, and
`rcl_subscription_fini` from `OusterPinhole` destruction on the main thread.
The cache API is explicitly not thread-safe.

The standard component executable destroys its node after `spin()` returns,
then calls `rclcpp::shutdown()`. A signal can wake the executor before the
shutdown thread finishes removing the rosout publisher. Consequently, node
destruction can race with that removal even though the driver's own scan
worker is joined correctly.

## Fix

`ouster-ros/cmake/node_main.cpp.in` is based on the ROS component executable
template, with `rclcpp::shutdown()` moved before node destruction. The utility
also uninstalls and joins the signal-handler thread when the default context
has already been invalidated. Merely checking `rclcpp::ok()` would not provide
that synchronization.

CMake uses this template for all standalone driver executables, including
`os_image`, `os_pinhole`, and `os_cloud`. Component registration and the existing
worker teardown are preserved. This is an executable-level fix: shared
components do not shut down their container's context or change its signal
handlers. A container that has the same teardown order needs an equivalent
fix in its own entry point or the ROS runtime.

Relevant upstream implementations:

- [Component executable template](https://github.com/ros2/rclcpp/blob/rolling/rclcpp_components/src/node_main.cpp.in)
- [Shutdown utility](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/utilities.cpp)
- [Signal handler join](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/signal_handler.cpp)
- [Node type cache](https://github.com/ros2/rcl/blob/rolling/rcl/include/rcl/node_type_cache.h)

## Validation — 2026-09-24

- The regression fixture holds an on-shutdown callback open for 300 ms and
  fails if its node is destroyed first. Both SIGINT and SIGTERM failed with
  the original entry point (exit 42). Both pass with the fix, five cycles each.
  The fixture uses the same executable template as the installed drivers.
- All 85 C++ tests and four camera launch-test cases passed, including clean
  exits for the real image and pinhole nodes.
- Instrumented replay of `07052026_4_an` produced synchronized CameraInfo,
  reflectivity and depth for four full and four cropped panels. Both nodes
  exited zero, with no overlapping cache unregister calls.
- Eight further uninstrumented bag replays passed: 16 pinhole processes exited
  zero, including eight stopped while packets and simulated clock were still
  streaming. Every run validated at least 20 synchronized triples per panel
  across the four full and four cropped views.

Workspace diagnostic artifacts are under `results/ouster-shutdown/`:
`gdb-7/pinhole-auto.log`, `cache-stack/`, `regression-before.log`,
`tests-fixed.log`, `fixed-traced/`, and `replay-summary.json`. These captures
are not required to run the regression tests.
