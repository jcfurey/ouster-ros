/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file lidar_scan_to_point_cloud_test.cpp
 * @brief Unit tests for the LidarScan + LidarInfo -> PointCloud2 conversion.
 */

#include <gtest/gtest.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include "ouster_ros/lidar_scan_to_point_cloud.h"
#include "ouster_sensor_msgs/msg/channel_field.hpp"
#include "ouster_sensor_msgs/msg/lidar_info.hpp"
#include "ouster_sensor_msgs/msg/lidar_scan.hpp"

namespace {

using ouster_sensor_msgs::msg::ChannelField;
using ouster_sensor_msgs::msg::LidarInfo;
using ouster_sensor_msgs::msg::LidarScan;

LidarInfo makeSimpleLidarInfo(uint32_t h, uint32_t w) {
    LidarInfo info;
    info.pixels_per_column = h;
    info.columns_per_frame = w;
    info.fps = 10;
    info.range_unit = 0.001;
    info.product_line = "TEST-SENSOR";
    info.serial_number = 12345;
    info.firmware_revision = "v1.0.0";
    info.udp_profile_lidar = "LEGACY";

    info.beam_azimuth_angles.resize(h, 0.0);
    info.beam_altitude_angles.resize(h, 0.0);
    info.pixel_shift_by_row.resize(h, 0);

    info.beam_to_lidar_transform.fill(0.0);
    info.beam_to_lidar_transform[0] = 1.0;
    info.beam_to_lidar_transform[5] = 1.0;
    info.beam_to_lidar_transform[10] = 1.0;
    info.beam_to_lidar_transform[15] = 1.0;

    info.lidar_to_sensor_transform.fill(0.0);
    info.lidar_to_sensor_transform[0] = 1.0;
    info.lidar_to_sensor_transform[5] = 1.0;
    info.lidar_to_sensor_transform[10] = 1.0;
    info.lidar_to_sensor_transform[15] = 1.0;

    info.imu_to_sensor_transform.fill(0.0);
    info.imu_to_sensor_transform[0] = 1.0;
    info.imu_to_sensor_transform[5] = 1.0;
    info.imu_to_sensor_transform[10] = 1.0;
    info.imu_to_sensor_transform[15] = 1.0;

    info.extrinsic.fill(0.0);
    info.extrinsic[0] = 1.0;
    info.extrinsic[5] = 1.0;
    info.extrinsic[10] = 1.0;
    info.extrinsic[15] = 1.0;

    return info;
}

void addUint32Channel(LidarScan& scan, const std::string& name,
                      const std::vector<uint32_t>& values) {
    ChannelField ch;
    ch.name = name;
    ch.datatype = ChannelField::UINT32;
    ch.element_size = 4;
    ch.data.resize(values.size() * sizeof(uint32_t));
    std::memcpy(ch.data.data(), values.data(),
                values.size() * sizeof(uint32_t));
    scan.channels.push_back(ch);
}

void addUint16Channel(LidarScan& scan, const std::string& name,
                      const std::vector<uint16_t>& values) {
    ChannelField ch;
    ch.name = name;
    ch.datatype = ChannelField::UINT16;
    ch.element_size = 2;
    ch.data.resize(values.size() * sizeof(uint16_t));
    std::memcpy(ch.data.data(), values.data(),
                values.size() * sizeof(uint16_t));
    scan.channels.push_back(ch);
}

float readFloat(const sensor_msgs::msg::PointCloud2& cloud, size_t point_idx,
                uint32_t field_offset) {
    float val;
    std::memcpy(&val,
                &cloud.data[point_idx * cloud.point_step + field_offset],
                sizeof(float));
    return val;
}

uint32_t readUint32(const sensor_msgs::msg::PointCloud2& cloud,
                    size_t point_idx, uint32_t field_offset) {
    uint32_t val;
    std::memcpy(&val,
                &cloud.data[point_idx * cloud.point_step + field_offset],
                sizeof(uint32_t));
    return val;
}

uint16_t readUint16(const sensor_msgs::msg::PointCloud2& cloud,
                    size_t point_idx, uint32_t field_offset) {
    uint16_t val;
    std::memcpy(&val,
                &cloud.data[point_idx * cloud.point_step + field_offset],
                sizeof(uint16_t));
    return val;
}

}  // anonymous namespace

class LidarScanToPointCloudTest : public ::testing::Test {
   protected:
    void SetUp() override {
        h = 4;
        w = 8;
        info = makeSimpleLidarInfo(h, w);
    }

    uint32_t h, w;
    LidarInfo info;
};

TEST_F(LidarScanToPointCloudTest, BasicDimensions) {
    LidarScan scan;
    scan.height = h;
    scan.width = w;
    scan.frame_id = 1;
    scan.timestamp.resize(w, 1000000000ULL);

    std::vector<uint32_t> range_data(h * w, 1000);
    addUint32Channel(scan, "range", range_data);

    sensor_msgs::msg::PointCloud2 cloud;
    ouster_ros::lidarScanToPointCloud(scan, info, cloud, false);

    EXPECT_EQ(cloud.height, h);
    EXPECT_EQ(cloud.width, w);
    EXPECT_EQ(cloud.fields.size(), 9u);
    EXPECT_FALSE(cloud.is_bigendian);
}

TEST_F(LidarScanToPointCloudTest, ZeroRangeProducesNaN) {
    LidarScan scan;
    scan.height = h;
    scan.width = w;
    scan.frame_id = 1;
    scan.timestamp.resize(w, 1000000000ULL);

    std::vector<uint32_t> range_data(h * w, 0);
    addUint32Channel(scan, "range", range_data);

    sensor_msgs::msg::PointCloud2 cloud;
    ouster_ros::lidarScanToPointCloud(scan, info, cloud, false);

    for (size_t i = 0; i < static_cast<size_t>(h * w); ++i) {
        float x = readFloat(cloud, i, 0);
        float y = readFloat(cloud, i, 4);
        float z = readFloat(cloud, i, 8);
        EXPECT_TRUE(std::isnan(x)) << "Point " << i << " x should be NaN";
        EXPECT_TRUE(std::isnan(y)) << "Point " << i << " y should be NaN";
        EXPECT_TRUE(std::isnan(z)) << "Point " << i << " z should be NaN";
    }

    EXPECT_FALSE(cloud.is_dense);
}

TEST_F(LidarScanToPointCloudTest, NonZeroRangeProducesValidXYZ) {
    LidarScan scan;
    scan.height = h;
    scan.width = w;
    scan.frame_id = 1;
    scan.timestamp.resize(w, 1000000000ULL);

    std::vector<uint32_t> range_data(h * w, 1000);
    addUint32Channel(scan, "range", range_data);

    sensor_msgs::msg::PointCloud2 cloud;
    ouster_ros::lidarScanToPointCloud(scan, info, cloud, false);

    for (uint32_t row = 0; row < h; ++row) {
        for (uint32_t col = 0; col < w; ++col) {
            size_t idx = row * w + col;
            float x = readFloat(cloud, idx, 0);
            float y = readFloat(cloud, idx, 4);
            float z = readFloat(cloud, idx, 8);

            EXPECT_FALSE(std::isnan(x));
            EXPECT_FALSE(std::isnan(y));

            float dist = std::sqrt(x * x + y * y + z * z);
            EXPECT_NEAR(dist, 1.0f, 0.01f)
                << "Point (" << row << "," << col << ") dist=" << dist;
        }
    }
}

TEST_F(LidarScanToPointCloudTest, ChannelDataIsCopied) {
    LidarScan scan;
    scan.height = h;
    scan.width = w;
    scan.frame_id = 1;
    scan.timestamp.resize(w, 1000000000ULL);

    size_t n = h * w;
    std::vector<uint32_t> range_data(n, 1000);
    std::vector<uint16_t> signal_data(n, 42);
    std::vector<uint16_t> reflectivity_data(n, 100);
    std::vector<uint16_t> near_ir_data(n, 200);

    addUint32Channel(scan, "range", range_data);
    addUint16Channel(scan, "signal", signal_data);
    addUint16Channel(scan, "reflectivity", reflectivity_data);
    addUint16Channel(scan, "near_ir", near_ir_data);

    sensor_msgs::msg::PointCloud2 cloud;
    ouster_ros::lidarScanToPointCloud(scan, info, cloud, false);

    uint32_t range_val = readUint32(cloud, 0, 12);
    uint16_t signal_val = readUint16(cloud, 0, 16);
    uint16_t refl_val = readUint16(cloud, 0, 18);
    uint16_t nir_val = readUint16(cloud, 0, 20);

    EXPECT_EQ(range_val, 1000u);
    EXPECT_EQ(signal_val, 42u);
    EXPECT_EQ(refl_val, 100u);
    EXPECT_EQ(nir_val, 200u);
}

TEST_F(LidarScanToPointCloudTest, RingFieldIsSet) {
    LidarScan scan;
    scan.height = h;
    scan.width = w;
    scan.frame_id = 1;
    scan.timestamp.resize(w, 1000000000ULL);

    std::vector<uint32_t> range_data(h * w, 1000);
    addUint32Channel(scan, "range", range_data);

    sensor_msgs::msg::PointCloud2 cloud;
    ouster_ros::lidarScanToPointCloud(scan, info, cloud, false);

    for (uint32_t row = 0; row < h; ++row) {
        for (uint32_t col = 0; col < w; ++col) {
            size_t idx = row * w + col;
            uint16_t ring = readUint16(cloud, idx, 26);
            EXPECT_EQ(ring, static_cast<uint16_t>(row))
                << "ring mismatch at (" << row << "," << col << ")";
        }
    }
}

TEST_F(LidarScanToPointCloudTest, TimestampOffsets) {
    LidarScan scan;
    scan.height = h;
    scan.width = w;
    scan.frame_id = 1;

    scan.timestamp.resize(w);
    for (uint32_t col = 0; col < w; ++col) {
        scan.timestamp[col] = 1000000000ULL + col * 1000000ULL;
    }

    std::vector<uint32_t> range_data(h * w, 1000);
    addUint32Channel(scan, "range", range_data);

    sensor_msgs::msg::PointCloud2 cloud;
    ouster_ros::lidarScanToPointCloud(scan, info, cloud, false);

    uint32_t t0 = readUint32(cloud, 0, 22);
    EXPECT_EQ(t0, 0u);

    uint32_t t1 = readUint32(cloud, 1, 22);
    EXPECT_EQ(t1, 1000000u);
}

TEST_F(LidarScanToPointCloudTest, MissingOptionalChannels) {
    LidarScan scan;
    scan.height = h;
    scan.width = w;
    scan.frame_id = 1;
    scan.timestamp.resize(w, 1000000000ULL);

    std::vector<uint32_t> range_data(h * w, 1000);
    addUint32Channel(scan, "range", range_data);

    sensor_msgs::msg::PointCloud2 cloud;
    EXPECT_NO_THROW(
        ouster_ros::lidarScanToPointCloud(scan, info, cloud, false));

    uint16_t signal_val = readUint16(cloud, 0, 16);
    uint16_t refl_val = readUint16(cloud, 0, 18);
    uint16_t nir_val = readUint16(cloud, 0, 20);

    EXPECT_EQ(signal_val, 0u);
    EXPECT_EQ(refl_val, 0u);
    EXPECT_EQ(nir_val, 0u);
}

TEST_F(LidarScanToPointCloudTest, MissingRangeChannelThrows) {
    LidarScan scan;
    scan.height = h;
    scan.width = w;
    scan.frame_id = 1;
    scan.timestamp.resize(w, 1000000000ULL);

    sensor_msgs::msg::PointCloud2 cloud;
    EXPECT_THROW(ouster_ros::lidarScanToPointCloud(scan, info, cloud, false),
                 std::invalid_argument);
}

TEST_F(LidarScanToPointCloudTest, DimensionMismatchThrows) {
    LidarScan scan;
    scan.height = h + 1;
    scan.width = w;
    scan.frame_id = 1;
    scan.timestamp.resize(w, 1000000000ULL);

    std::vector<uint32_t> range_data((h + 1) * w, 1000);
    addUint32Channel(scan, "range", range_data);

    sensor_msgs::msg::PointCloud2 cloud;
    EXPECT_THROW(ouster_ros::lidarScanToPointCloud(scan, info, cloud, false),
                 std::invalid_argument);
}

TEST_F(LidarScanToPointCloudTest, HeaderIsCopied) {
    LidarScan scan;
    scan.height = h;
    scan.width = w;
    scan.frame_id = 1;
    scan.header.frame_id = "os_lidar";
    scan.header.stamp.sec = 100;
    scan.header.stamp.nanosec = 500;
    scan.timestamp.resize(w, 1000000000ULL);

    std::vector<uint32_t> range_data(h * w, 1000);
    addUint32Channel(scan, "range", range_data);

    sensor_msgs::msg::PointCloud2 cloud;
    ouster_ros::lidarScanToPointCloud(scan, info, cloud, false);

    EXPECT_EQ(cloud.header.frame_id, "os_lidar");
    EXPECT_EQ(cloud.header.stamp.sec, 100);
    EXPECT_EQ(cloud.header.stamp.nanosec, 500u);
}
