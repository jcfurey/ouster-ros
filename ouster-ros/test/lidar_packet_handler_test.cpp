/**
 * Copyright (c) 2018-2026, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <limits>
#include <map>
#include <mutex>

#include <ouster/impl/packet_writer.h>

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

namespace {

class LidarPacketHandlerRosTimeTest : public ::testing::Test {
   protected:
    void SetUp() override {
        info = ouster::sdk::core::default_sensor_info(
            ouster::sdk::core::LidarMode::_512x10);
        // Two packets per 100 ms frame, with exact 3.125 ms column spacing.
        info.format.columns_per_frame = 32;
        info.format.columns_per_packet = 16;
        info.format.column_window = {0, 31};
        handler = ouster_ros::LidarPacketHandler::create(
            info,
            {[this](const ouster::sdk::core::LidarScan& scan, uint64_t,
                    const rclcpp::Time& stamp) {
                std::lock_guard<std::mutex> lock(mutex);
                stamps[scan.frame_id] = stamp.nanoseconds();
                ready.notify_all();
            }},
            "TIME_FROM_ROS_TIME", 0, 0.0f);
    }

    void send(uint32_t frame, int first_column, uint64_t receive_ns) {
        const ouster::sdk::core::impl::PacketWriter writer{
            ouster::sdk::core::get_format(info)};
        ouster::sdk::core::LidarPacket packet(writer.lidar_packet_size);
        packet.host_timestamp = receive_ns;
        writer.set_frame_id(packet.buf.data(), frame);
        for (int i = 0; i < writer.columns_per_packet; ++i) {
            auto* column = writer.nth_col(i, packet.buf.data());
            writer.set_col_measurement_id(column, first_column + i);
            writer.set_col_status(column, 1);
            writer.set_col_timestamp(
                column, 1'000'000'000ULL + (frame - 41) * 100'000'000ULL +
                            (first_column + i) * 3'125'000ULL);
        }
        handler(packet);
    }

    void expect_stamp(uint32_t frame, int64_t expected_ns) {
        std::unique_lock<std::mutex> lock(mutex);
        ASSERT_TRUE(ready.wait_for(lock, std::chrono::seconds(5), [&] {
            return stamps.count(frame) != 0;
        })) << "No completed scan for frame " << frame;
        EXPECT_EQ(stamps.at(frame), expected_ns);
    }

    ouster::sdk::core::SensorInfo info;
    std::mutex mutex;
    std::condition_variable ready;
    std::map<uint32_t, int64_t> stamps;
    ouster_ros::LidarPacketHandler::HandlerType handler;
};

TEST_F(LidarPacketHandlerRosTimeTest, CompleteFramesUseTheirOwnPacketTimes) {
    send(41, 0, 2'000'000'000);
    send(41, 16, 2'100'000'000);
    send(42, 0, 2'200'000'000);
    send(42, 16, 2'300'000'000);

    expect_stamp(41, 2'000'000'000);
    expect_stamp(42, 2'200'000'000);
}

TEST_F(LidarPacketHandlerRosTimeTest, MissingLeadingPacketExtrapolatesToScanStart) {
    send(41, 16, 2'050'000'000);
    send(42, 0, 2'200'000'000);
    send(42, 16, 2'300'000'000);

    expect_stamp(41, 2'000'000'000);
    expect_stamp(42, 2'200'000'000);
}

TEST_F(LidarPacketHandlerRosTimeTest, RolloverKeepsTheCompletedFramesPacketTime) {
    send(41, 0, 2'000'000'000);
    send(42, 0, 2'200'000'000);
    send(42, 16, 2'300'000'000);

    expect_stamp(41, 2'000'000'000);
    expect_stamp(42, 2'200'000'000);
}

TEST_F(LidarPacketHandlerRosTimeTest, BurstDeliveryPreservesScanSpanCorrection) {
    send(41, 0, 2'100'000'000);
    send(41, 16, 2'100'000'000);

    expect_stamp(41, 2'003'125'000);
}

TEST_F(LidarPacketHandlerRosTimeTest, MissingPacketTimesUseCompletionFallback) {
    send(41, 0, 0);
    send(41, 16, 0);
    send(42, 0, 2'200'000'000);

    expect_stamp(41, 2'100'000'000);
}

TEST_F(LidarPacketHandlerRosTimeTest, BurstNearClockStartClampsToZero) {
    send(41, 0, 1);
    send(41, 16, 1);

    expect_stamp(41, 0);
}

}  // namespace
