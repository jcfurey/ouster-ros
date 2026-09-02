/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_processing_node_base.cpp
 * @brief implementation of OusterProcessingNodeBase interface
 */

#include "ouster_ros/os_processing_node_base.h"

#include <memory>
#include <string>

#include "ouster_ros/impl/file_util.h"

namespace ouster_ros {

void OusterProcessingNodeBase::create_metadata_subscriber(
    std::function<void(const std_msgs::msg::String::ConstSharedPtr&)>
        on_sensor_metadata) {
    auto latching_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    latching_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    latching_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    metadata_sub = create_subscription<std_msgs::msg::String>(
        "metadata", latching_qos, on_sensor_metadata);
}

bool OusterProcessingNodeBase::load_metadata_from_file(
    const std::function<void(const std_msgs::msg::String::ConstSharedPtr&)>&
        on_sensor_metadata) {
    const auto metadata_path =
        has_parameter("metadata")
            ? get_parameter("metadata").as_string()
            : declare_parameter<std::string>("metadata", "");
    if (metadata_path.find_first_not_of(" \t\r\n") == std::string::npos) {
        return false;
    }

    const auto metadata = impl::read_text_file(metadata_path);
    if (metadata.empty()) {
        RCLCPP_ERROR_STREAM(get_logger(),
                            "metadata file is empty or unreadable: "
                                << metadata_path);
        return false;
    }

    auto message = std::make_shared<std_msgs::msg::String>();
    message->data = metadata;
    on_sensor_metadata(message);
    RCLCPP_INFO_STREAM(get_logger(),
                       "loaded sensor metadata from " << metadata_path);
    return true;
}

}  // namespace ouster_ros
