/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file imu_packet_handler.h
 * @brief Handles IMU packets from the Ouster sensor and converts them to ROS messages.
 */

#pragma once

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

namespace ouster_ros {

class ImuPacketHandler {
   public:
    using HandlerOutput = std::vector<sensor_msgs::msg::Imu>;
    using HandlerType = std::function<HandlerOutput(const ouster::sdk::core::ImuPacket&)>;

   public:
    static HandlerType create(const ouster::sdk::core::SensorInfo& info,
                              const std::string& frame,
                              const std::string& timestamp_mode,
                              int64_t ptp_utc_tai_offset) {
        const auto& pf = ouster::sdk::core::get_format(info);
        using Timestamper = std::function<rclcpp::Time(const ouster::sdk::core::ImuPacket&)>;
        Timestamper timestamper;
        if (timestamp_mode == "TIME_FROM_ROS_TIME") {
            // A packet leaves the sensor after its last sample, so arrival
            // time anchors the last valid sample. The returned base stamps
            // sample 0, which packet_to_imu_msgs then offsets by sensor-clock
            // spacing; anchoring sample 0 instead would stamp the rest of a
            // multi-sample packet in the future.
            timestamper = Timestamper{
                [info](const ouster::sdk::core::ImuPacket& imu_packet) {
                    const auto status = imu_packet.status();
                    const auto sample_ts =
                        imu_sample_timestamps(imu_packet, info);
                    uint64_t span = 0;
                    for (auto i = sample_ts.size() - 1; i > 0; --i) {
                        if (i < status.size() && (status[i] & 0x1) != 0) {
                            if (sample_ts[i] > sample_ts[0]) {
                                span = sample_ts[i] - sample_ts[0];
                            }
                            break;
                        }
                    }
                    const uint64_t host = imu_packet.host_timestamp;
                    return rclcpp::Time(
                        static_cast<int64_t>(host > span ? host - span : 0));
                }};
        } else if (timestamp_mode == "TIME_FROM_PTP_1588") {
            timestamper = Timestamper{
                [pf, ptp_utc_tai_offset](const ouster::sdk::core::ImuPacket& imu_packet) {
                    if (pf.imu_measurements_per_packet == 0) { // LEGACY
                        auto ts = pf.imu_gyro_ts(imu_packet.buf.data());
                        ts = impl::ts_safe_offset_add(ts, ptp_utc_tai_offset);
                        return rclcpp::Time(ts);
                    } else {
                        auto ts = imu_packet.timestamp()[0];
                        ts = impl::ts_safe_offset_add(ts, ptp_utc_tai_offset);
                        return rclcpp::Time(ts);
                    }
                }};
        } else {
            timestamper = Timestamper{
                [pf](const ouster::sdk::core::ImuPacket& imu_packet) {
                    if (pf.imu_measurements_per_packet == 0) { // LEGACY
                        auto ts = pf.imu_gyro_ts(imu_packet.buf.data());
                        return rclcpp::Time(ts);
                    } else {
                        auto ts = imu_packet.timestamp()[0];
                        return rclcpp::Time(ts);
                    }
                }};
        }

        return [pf, frame, timestamper, info](const ouster::sdk::core::ImuPacket& imu_packet) {
            return packet_to_imu_msgs(imu_packet, frame, timestamper(imu_packet), info);
        };
    }
};

}  // namespace ouster_ros