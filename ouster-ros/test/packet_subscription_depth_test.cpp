/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include "ouster_ros/os_processing_node_base.h"

namespace ouster_ros {
namespace {

class PacketSubscriptionDepthAccess : public OusterProcessingNodeBase {
   public:
    using OusterProcessingNodeBase::lidar_packet_subscription_depth;
};

TEST(PacketSubscriptionDepthTest, BuffersTwoCompleteLidarFrames) {
    ouster::sdk::core::SensorInfo info;
    info.format.columns_per_frame = 1024;
    info.format.columns_per_packet = 16;

    EXPECT_EQ(
        PacketSubscriptionDepthAccess::lidar_packet_subscription_depth(info),
        128u);
}

TEST(PacketSubscriptionDepthTest, RoundsUpPartialPacketAndKeepsMinimum) {
    ouster::sdk::core::SensorInfo info;
    info.format.columns_per_frame = 33;
    info.format.columns_per_packet = 16;
    EXPECT_EQ(
        PacketSubscriptionDepthAccess::lidar_packet_subscription_depth(info),
        6u);

    info.format.columns_per_frame = 0;
    EXPECT_EQ(
        PacketSubscriptionDepthAccess::lidar_packet_subscription_depth(info),
        5u);
}

}  // namespace
}  // namespace ouster_ros
