/**
 * Copyright (c) 2018-2026, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <ouster/lidar_scan.h>
#include <ouster/xyzlut.h>

#include "ouster_ros/os_ros.h"

namespace {

using ouster::sdk::core::LidarMode;
using ouster::sdk::core::LidarScan;
using ouster::sdk::core::SensorInfo;

double wrapped(double angle) {
    return std::remainder(angle, 2.0 * M_PI);
}

// Every LaserScan ray must lie at the azimuth of the point the SDK places for
// the same pixel, whatever destagger convention the firmware reports.
void expect_rays_at_point_azimuths(const SensorInfo& info, uint16_t ring) {
    const auto w = info.format.columns_per_frame;
    const auto h = info.format.pixels_per_column;
    LidarScan scan(info);
    auto range = scan.field<uint32_t>(ouster::sdk::core::ChanField::RANGE);
    // Encode each staggered column in its range (mm) to identify the pixel.
    for (uint32_t u = 0; u < h; ++u) {
        for (uint32_t s = 0; s < w; ++s) range(u, s) = 20000 + s;
    }
    const auto lut = ouster::sdk::core::make_xyz_lut(
        w, h, 1.0, info.beam_to_lidar_transform,
        ouster::sdk::core::mat4d::Identity(), info.beam_azimuth_angles,
        info.beam_altitude_angles);

    const auto correction = ouster_ros::laser_scan_azimuth_correction(
        info.format.pixel_shift_by_row, info.beam_azimuth_angles, ring, w);
    const auto msg = ouster_ros::lidar_scan_to_laser_scan_msg(
        scan, rclcpp::Time(0), "os_lidar", LidarMode(w, info.format.fps),
        ring, false, info.format.pixel_shift_by_row, 0, correction);

    ASSERT_EQ(msg.ranges.size(), w);
    EXPECT_NEAR(msg.angle_increment, 2.0 * M_PI / w, 1e-6);
    EXPECT_NEAR(msg.angle_max - msg.angle_min,
                (w - 1) * msg.angle_increment, 1e-5);
    EXPECT_LE(std::abs(msg.angle_min + M_PI), msg.angle_increment / 2 + 1e-6);

    double worst = 0.0;
    std::vector<bool> seen(w, false);
    for (uint32_t i = 0; i < w; ++i) {
        const auto s = static_cast<uint32_t>(
            std::lround(msg.ranges[i] * 1000.0)) - 20000U;
        ASSERT_LT(s, w);
        seen[s] = true;
        const auto row = static_cast<Eigen::Index>(ring) * w + s;
        const double r = 20000.0 + s;
        const double x = lut.direction(row, 0) * r + lut.offset(row, 0);
        const double y = lut.direction(row, 1) * r + lut.offset(row, 1);
        const double ray = msg.angle_min + i * msg.angle_increment;
        worst = std::max(worst, std::abs(wrapped(ray - std::atan2(y, x))));
    }
    EXPECT_TRUE(std::all_of(seen.begin(), seen.end(), [](bool b) { return b; }));
    // Residual: beam-origin parallax at 20 m plus float angle_min.
    EXPECT_LT(worst, 2e-4) << "worst ray azimuth error " << worst << " rad";
}

}  // namespace

TEST(LaserScanTest, RaysLieAtTheirPointAzimuths) {
    auto info = ouster::sdk::core::default_sensor_info(LidarMode::_1024x10);
    for (uint16_t ring : {0, 7, 31, 63}) {
        SCOPED_TRACE(ring);
        expect_rays_at_point_azimuths(info, ring);
    }
}

TEST(LaserScanTest, RaysLieAtTheirPointAzimuthsWithOffsetDestaggerShifts) {
    // Firmware 2.x reports shifts offset so the smallest is zero.
    auto info = ouster::sdk::core::default_sensor_info(LidarMode::_512x10);
    auto& shifts = info.format.pixel_shift_by_row;
    const int smallest = *std::min_element(shifts.begin(), shifts.end());
    for (auto& shift : shifts) shift -= smallest;
    for (uint16_t ring : {0, 1, 2, 3, 40}) {
        SCOPED_TRACE(ring);
        expect_rays_at_point_azimuths(info, ring);
    }
}

TEST(LaserScanTest, LegacyOverloadKeepsUniformFullCircle) {
    auto info = ouster::sdk::core::default_sensor_info(LidarMode::_1024x10);
    LidarScan scan(info);
    const auto msg = ouster_ros::lidar_scan_to_laser_scan_msg(
        scan, rclcpp::Time(0), "os_lidar", LidarMode::_1024x10, 0, false,
        info.format.pixel_shift_by_row, 0);
    EXPECT_FLOAT_EQ(msg.angle_min, -M_PI);
    EXPECT_NEAR(msg.angle_max, M_PI - 2.0 * M_PI / 1024, 1e-5);
    // No returns: NaN per REP-117 unless nan_is_inf.
    EXPECT_TRUE(std::isnan(msg.ranges.front()));
}
