/**
 * Copyright (c) 2018-2026, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <vector>

#include <ouster/impl/packet_writer.h>

#include "../src/imu_packet_handler.h"

namespace {

using ouster::sdk::core::ImuPacket;
using ouster::sdk::core::PacketFormat;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::core::UDPProfileIMU;

constexpr int kSamples = 8;
constexpr uint64_t kFirstSampleNs = 1'000'000'000ULL;
constexpr uint64_t kSamplePeriodNs = 1'250'000ULL;

SensorInfo modern_imu_info() {
    auto info = ouster::sdk::core::default_sensor_info(
        ouster::sdk::core::LidarMode::_1024x10);
    // Firmware that emits ACCEL32_GYRO32_NMEA uses standard packet headers
    // and footers; the SDK's 64-bit field accessors rely on that trailing
    // footer when writing the last measurement.
    info.format.udp_profile_lidar =
        ouster::sdk::core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16;
    info.format.header_type = ouster::sdk::core::HeaderType::STANDARD;
    info.format.udp_profile_imu = UDPProfileIMU::ACCEL32_GYRO32_NMEA;
    info.format.imu_measurements_per_packet = kSamples;
    info.format.imu_packets_per_frame = 10;
    info.format.fps = 10;
    return info;
}

ImuPacket modern_packet(const SensorInfo& info, uint64_t host_ns,
                        uint16_t valid_mask) {
    auto format = std::make_shared<PacketFormat>(info);
    EXPECT_EQ(format->imu_packet_size, 32U + 100U + kSamples * 36U + 32U);
    ouster::sdk::core::impl::PacketWriter writer(*format);
    ImuPacket packet(static_cast<int>(format->imu_packet_size));
    packet.format = format;
    packet.host_timestamp = host_ns;
    auto* buf = packet.buf.data();
    writer.set_packet_type(buf, 0x2);
    for (int i = 0; i < kSamples; ++i) {
        auto* sample = writer.imu_nth_measurement(i, buf);
        writer.set_col_timestamp(sample, kFirstSampleNs + i * kSamplePeriodNs);
        writer.set_col_status(sample, (valid_mask >> i) & 1U);
        writer.set_imu_la_z(sample, 9.80665F);
        writer.set_imu_av_x(sample, 0.25F * static_cast<float>(i));
    }
    return packet;
}

uint64_t stamp_ns(const sensor_msgs::msg::Imu& msg) {
    return rclcpp::Time(msg.header.stamp).nanoseconds();
}

}  // namespace

TEST(ImuPacketHandlerTest, RosTimeAnchorsLastSampleToArrival) {
    const auto info = modern_imu_info();
    constexpr uint64_t kArrivalNs = 50'000'000'000ULL;
    const auto handler = ouster_ros::ImuPacketHandler::create(
        info, "os_imu", "TIME_FROM_ROS_TIME", 0);

    const auto msgs = handler(modern_packet(info, kArrivalNs, 0xff));
    ASSERT_EQ(msgs.size(), static_cast<size_t>(kSamples));
    // No sample may be stamped after the packet arrived, and samples keep
    // their sensor-clock spacing.
    EXPECT_EQ(stamp_ns(msgs.back()), kArrivalNs);
    for (int i = 0; i < kSamples; ++i) {
        EXPECT_EQ(stamp_ns(msgs[i]),
                  kArrivalNs - (kSamples - 1 - i) * kSamplePeriodNs);
        EXPECT_FLOAT_EQ(msgs[i].angular_velocity.x, 0.25F * i);
        EXPECT_FLOAT_EQ(msgs[i].linear_acceleration.z, 9.80665F);
    }
}

TEST(ImuPacketHandlerTest, RosTimeAnchorsLastValidSample) {
    const auto info = modern_imu_info();
    constexpr uint64_t kArrivalNs = 50'000'000'000ULL;
    const auto handler = ouster_ros::ImuPacketHandler::create(
        info, "os_imu", "TIME_FROM_ROS_TIME", 0);

    // Samples 6 and 7 invalid: sample 5 is the latest measurement sent.
    const auto msgs = handler(modern_packet(info, kArrivalNs, 0x3f));
    ASSERT_EQ(msgs.size(), 6U);
    EXPECT_EQ(stamp_ns(msgs.back()), kArrivalNs);
    EXPECT_EQ(stamp_ns(msgs.front()), kArrivalNs - 5 * kSamplePeriodNs);
}

TEST(ImuPacketHandlerTest, SensorTimeUsesSampleTimestamps) {
    const auto info = modern_imu_info();
    const auto handler = ouster_ros::ImuPacketHandler::create(
        info, "os_imu", "TIME_FROM_INTERNAL_OSC", 0);

    const auto msgs = handler(modern_packet(info, 77, 0xff));
    ASSERT_EQ(msgs.size(), static_cast<size_t>(kSamples));
    for (int i = 0; i < kSamples; ++i) {
        EXPECT_EQ(stamp_ns(msgs[i]), kFirstSampleNs + i * kSamplePeriodNs);
        EXPECT_EQ(msgs[i].header.frame_id, "os_imu");
        EXPECT_EQ(msgs[i].orientation_covariance[0], -1.0);
    }
}
