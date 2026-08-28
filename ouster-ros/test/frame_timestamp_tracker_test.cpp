/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "frame_timestamp_tracker.h"

namespace ouster_ros {
namespace impl {

TEST(FrameTimestampTrackerTest, CompleteFrameUsesItsOwnFirstPacketTimestamp) {
    FrameTimestampTracker tracker;

    tracker.observe(41, 1'000);
    tracker.observe(41, 1'010);

    EXPECT_EQ(tracker.take(41), 1'000);
    EXPECT_EQ(tracker.take(41), std::nullopt);
}

TEST(FrameTimestampTrackerTest, RolloverCompletionPreservesNextFrameTimestamp) {
    FrameTimestampTracker tracker;

    tracker.observe(41, 1'000);
    tracker.observe(42, 1'100);

    EXPECT_EQ(tracker.take(41), 1'000);
    EXPECT_EQ(tracker.take(42), 1'100);
}

TEST(FrameTimestampTrackerTest, FrameIdWrapDoesNotReusePriorTimestamp) {
    FrameTimestampTracker tracker;
    const auto max_frame_id = std::numeric_limits<uint32_t>::max();

    tracker.observe(max_frame_id, 1'000);
    EXPECT_EQ(tracker.take(max_frame_id), 1'000);

    tracker.observe(0, 1'100);
    EXPECT_EQ(tracker.take(0), 1'100);
}

TEST(FrameTimestampTrackerTest, EvictsStaleFramesAtConfiguredCapacity) {
    FrameTimestampTracker tracker{2};

    tracker.observe(10, 1'000);
    tracker.observe(11, 1'100);
    tracker.observe(12, 1'200);

    EXPECT_EQ(tracker.take(10), std::nullopt);
    EXPECT_EQ(tracker.take(11), 1'100);
    EXPECT_EQ(tracker.take(12), 1'200);
}

}  // namespace impl
}  // namespace ouster_ros
