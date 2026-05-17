/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file lidar_scan_to_point_cloud.h
 * @brief Converts LidarScan + LidarInfo messages into sensor_msgs::msg::PointCloud2
 */

#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ouster_sensor_msgs/msg/lidar_info.hpp"
#include "ouster_sensor_msgs/msg/lidar_scan.hpp"

namespace ouster_ros {

/**
 * @brief Convert a LidarScan message and LidarInfo message into a PointCloud2.
 *
 * This function projects the range measurements from the LidarScan into 3D
 * Cartesian coordinates using the calibration data in LidarInfo. The resulting
 * PointCloud2 is organized (height x width) with the following fields:
 *   x, y, z      - Cartesian coordinates in meters (float32)
 *   range        - Raw range in millimeters (uint32)
 *   signal       - Signal intensity (uint16)
 *   reflectivity - Calibrated reflectivity (uint16)
 *   near_ir      - Near-infrared / ambient (uint16)
 *   t            - Timestamp offset in nanoseconds from scan start (uint32)
 *   ring         - Beam / channel index (uint16)
 *
 * Points with zero range are marked as NaN (invalid).
 *
 * @param[in]  scan       The LidarScan message containing channel data
 * @param[in]  info       The LidarInfo message containing calibration data
 * @param[out] cloud_msg  The output PointCloud2 message
 * @param[in]  destagger  If true, apply pixel shift correction (default: true)
 */
void lidarScanToPointCloud(
    const ouster_sensor_msgs::msg::LidarScan& scan,
    const ouster_sensor_msgs::msg::LidarInfo& info,
    sensor_msgs::msg::PointCloud2& cloud_msg,
    bool destagger = true);

}  // namespace ouster_ros
