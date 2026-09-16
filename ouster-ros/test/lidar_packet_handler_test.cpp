/**
 * Copyright (c) 2018-2026, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <limits>

#include "../src/lidar_packet_handler.h"

TEST(LidarPacketHandlerTimestampTest, InterpolatesEpochNanosecondsExactly) {
    constexpr uint64_t base = 1700000000000000123ULL;

    EXPECT_EQ(base + 1021000ULL,
              linear_interpolate(3, base, 1027, base + 1024000ULL, 1024));
}

TEST(LidarPacketHandlerTimestampTest, InterpolatesDecreasingValuesExactly) {
    constexpr uint64_t base = 1700000000001024123ULL;

    EXPECT_EQ(base - 1021000ULL,
              linear_interpolate(3, base, 1027, base - 1024000ULL, 1024));
}

TEST(OusterRosTimestampTest, AppliesSignedOffsetsWithoutOverflow) {
    EXPECT_EQ(0U,
              ouster_ros::impl::ts_safe_offset_add(
                  1U, std::numeric_limits<int64_t>::min()));
    EXPECT_EQ(std::numeric_limits<uint64_t>::max(),
              ouster_ros::impl::ts_safe_offset_add(
                  std::numeric_limits<uint64_t>::max(), 1));
    EXPECT_EQ(42U, ouster_ros::impl::ts_safe_offset_add(79U, -37));
}

TEST(LidarPacketHandlerMetadataTest, AcceptsDataFormatWithoutLidarMode) {
    ::testing::FLAGS_gtest_death_test_style = "threadsafe";
    // Isolate construction so a prematurely started std::thread cannot abort
    // the entire test process if a later initialization step throws.
    EXPECT_EXIT(
        {
            auto info = ouster::sdk::core::default_sensor_info(
                ouster::sdk::core::LidarMode::_512x10);
            info.config.lidar_mode.reset();
            try {
                ouster_ros::LidarPacketHandler handler(info, {}, "", 0, 0.0f);
            } catch (...) {
                std::_Exit(1);
            }
            std::_Exit(0);
        },
        ::testing::ExitedWithCode(0), "");
}

TEST(LidarPacketHandlerMetadataTest, RejectsInvalidTimingWithoutTerminating) {
    ::testing::FLAGS_gtest_death_test_style = "threadsafe";
    EXPECT_EXIT(
        {
            auto info = ouster::sdk::core::default_sensor_info(
                ouster::sdk::core::LidarMode::_512x10);
            info.config.lidar_mode.reset();
            info.format.fps = 0;
            try {
                ouster_ros::LidarPacketHandler handler(info, {}, "", 0, 0.0f);
            } catch (const std::invalid_argument&) {
                std::_Exit(0);
            } catch (...) {
                std::_Exit(2);
            }
            std::_Exit(1);
        },
        ::testing::ExitedWithCode(0), "");
}
