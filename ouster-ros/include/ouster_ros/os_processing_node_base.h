/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_processing_node_base.h
 * @brief Base class for packet-processing nodes
 *
 */

#pragma once

#include <ouster/types.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace ouster_ros {

class OusterProcessingNodeBase : public rclcpp::Node {
   protected:
    OusterProcessingNodeBase(const std::string& name,
                             const rclcpp::NodeOptions& options)
        : rclcpp::Node(name, options) {}

    void create_metadata_subscriber(
        std::function<void(const std_msgs::msg::String::ConstSharedPtr&)>
            on_sensor_metadata);

    // Protect pipeline replacement from in-flight packet callbacks.
    std::mutex pipeline_mutex;

    bool metadata_is_active(const std::string& metadata) const {
        return has_active_metadata_ && metadata == active_metadata_;
    }

    void mark_metadata_active(const std::string& metadata) {
        active_metadata_ = metadata;
        has_active_metadata_ = true;
    }

    void invalidate_active_metadata() {
        active_metadata_.clear();
        has_active_metadata_ = false;
    }

    uint64_t begin_pipeline_update() { return ++pipeline_generation_; }

    bool pipeline_is_current(uint64_t generation) const {
        return generation == pipeline_generation_;
    }

    static std::size_t lidar_packet_subscription_depth(
        const ouster::sdk::core::SensorInfo& sensor_info) noexcept {
        constexpr std::size_t kMinimumSensorDataDepth = 5;
        constexpr std::size_t kBufferedFrames = 2;
        const auto columns_per_frame =
            static_cast<std::size_t>(sensor_info.format.columns_per_frame);
        const auto columns_per_packet =
            static_cast<std::size_t>(sensor_info.format.columns_per_packet);
        if (columns_per_frame == 0 || columns_per_packet == 0) {
            return kMinimumSensorDataDepth;
        }

        const auto packets_per_frame =
            columns_per_frame / columns_per_packet +
            (columns_per_frame % columns_per_packet != 0 ? 1u : 0u);
        // A simulated frame may arrive as one intentional packet burst. Keep
        // one complete frame queued while the preceding frame's cloud or
        // image products are being assembled.
        return std::max(kMinimumSensorDataDepth,
                        kBufferedFrames * packets_per_frame);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr metadata_sub;
    ouster::sdk::core::SensorInfo info;
    std::shared_ptr<ouster::sdk::core::PacketFormat> packet_format;

   private:
    std::string active_metadata_;
    bool has_active_metadata_ = false;
    uint64_t pipeline_generation_ = 0;
};

}  // namespace ouster_ros
