/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file lidar_scan_to_point_cloud.cpp
 * @brief Implementation of the LidarScan+LidarInfo to PointCloud2 conversion
 */

#include "ouster_ros/lidar_scan_to_point_cloud.h"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <Eigen/Core>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace ouster_ros {

namespace {

using ouster_sensor_msgs::msg::ChannelField;
using ouster_sensor_msgs::msg::LidarInfo;
using ouster_sensor_msgs::msg::LidarScan;

Eigen::Matrix4d arrayToMatrix4d(const std::array<double, 16>& arr) {
    Eigen::Matrix4d mat;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            mat(r, c) = arr[static_cast<size_t>(r * 4 + c)];
    return mat;
}

/**
 * @brief Build XYZ lookup tables (direction + offset) from LidarInfo.
 *
 * Replicates the logic from ouster::sdk::core::make_xyz_lut() so that the
 * conversion can work purely from the ROS message data without depending on
 * an ouster::sdk::core::SensorInfo object.
 */
void buildXYZLut(const LidarInfo& info,
                 Eigen::Array<double, Eigen::Dynamic, 3, Eigen::RowMajor>& direction,
                 Eigen::Array<double, Eigen::Dynamic, 3, Eigen::RowMajor>& offset) {
    const size_t w = info.columns_per_frame;
    const size_t h = info.pixels_per_column;
    const size_t n = w * h;
    const double range_unit = info.range_unit;

    if (w == 0 || h == 0) {
        throw std::invalid_argument(
            "LidarInfo dimensions must be greater than zero");
    }

    Eigen::Matrix4d beam_to_lidar = arrayToMatrix4d(info.beam_to_lidar_transform);
    Eigen::Matrix4d lidar_to_sensor = arrayToMatrix4d(info.lidar_to_sensor_transform);

    double beam_to_lidar_euclidean_distance_mm = beam_to_lidar(0, 3);
    if (beam_to_lidar(2, 3) != 0.0) {
        beam_to_lidar_euclidean_distance_mm =
            std::sqrt(beam_to_lidar(0, 3) * beam_to_lidar(0, 3) +
                      beam_to_lidar(2, 3) * beam_to_lidar(2, 3));
    }

    Eigen::ArrayXd encoder(n);
    Eigen::ArrayXd azimuth(n);
    Eigen::ArrayXd altitude(n);

    const double azimuth_radians = 2.0 * M_PI / static_cast<double>(w);

    if (info.beam_azimuth_angles.size() == h &&
        info.beam_altitude_angles.size() == h) {
        // Standard OS-sensor: per-beam angles
        for (size_t col = 0; col < w; ++col) {
            for (size_t row = 0; row < h; ++row) {
                size_t i = row * w + col;
                encoder(static_cast<Eigen::Index>(i)) =
                    2.0 * M_PI - (static_cast<double>(col) * azimuth_radians);
                azimuth(static_cast<Eigen::Index>(i)) =
                    -info.beam_azimuth_angles[row] * M_PI / 180.0;
                altitude(static_cast<Eigen::Index>(i)) =
                    info.beam_altitude_angles[row] * M_PI / 180.0;
            }
        }
    } else if (info.beam_azimuth_angles.size() == n &&
               info.beam_altitude_angles.size() == n) {
        // DF sensor: per-pixel angles
        for (size_t col = 0; col < w; ++col) {
            for (size_t row = 0; row < h; ++row) {
                size_t i = row * w + col;
                encoder(static_cast<Eigen::Index>(i)) = 0.0;
                azimuth(static_cast<Eigen::Index>(i)) =
                    info.beam_azimuth_angles[i] * M_PI / 180.0;
                altitude(static_cast<Eigen::Index>(i)) =
                    info.beam_altitude_angles[i] * M_PI / 180.0;
            }
        }
    } else {
        throw std::invalid_argument(
            "beam angle arrays must have size h or h*w");
    }

    direction.resize(static_cast<Eigen::Index>(n), 3);
    direction.col(0) = (encoder + azimuth).cos() * altitude.cos();
    direction.col(1) = (encoder + azimuth).sin() * altitude.cos();
    direction.col(2) = altitude.sin();

    offset.resize(static_cast<Eigen::Index>(n), 3);
    offset.col(0) =
        encoder.cos() * beam_to_lidar(0, 3) -
        direction.col(0) * beam_to_lidar_euclidean_distance_mm;
    offset.col(1) =
        encoder.sin() * beam_to_lidar(0, 3) -
        direction.col(1) * beam_to_lidar_euclidean_distance_mm;
    offset.col(2) =
        -direction.col(2) * beam_to_lidar_euclidean_distance_mm +
        beam_to_lidar(2, 3);

    auto rot = lidar_to_sensor.topLeftCorner(3, 3).transpose();
    auto trans = lidar_to_sensor.topRightCorner(3, 1).transpose();
    direction.matrix() *= rot;
    offset.matrix() *= rot;
    offset.matrix() += trans.replicate(static_cast<Eigen::Index>(n), 1);

    // Apply range_unit scaling so that:  xyz = range * direction + offset
    // yields coordinates in meters.
    direction *= range_unit;
    offset *= range_unit;
}

const ChannelField* findChannel(const LidarScan& scan,
                                const std::string& name) {
    for (const auto& ch : scan.channels) {
        if (ch.name == name) return &ch;
    }
    return nullptr;
}

uint32_t readUint32(const ChannelField& ch, size_t idx) {
    uint32_t val = 0;
    switch (ch.datatype) {
        case ChannelField::UINT8:
            val = ch.data[idx];
            break;
        case ChannelField::UINT16: {
            size_t byte_idx = idx * 2;
            std::memcpy(&val, &ch.data[byte_idx], 2);
            break;
        }
        case ChannelField::UINT32: {
            size_t byte_idx = idx * 4;
            std::memcpy(&val, &ch.data[byte_idx], 4);
            break;
        }
        default:
            break;
    }
    return val;
}

uint16_t readUint16(const ChannelField& ch, size_t idx) {
    uint16_t val = 0;
    switch (ch.datatype) {
        case ChannelField::UINT8:
            val = ch.data[idx];
            break;
        case ChannelField::UINT16: {
            size_t byte_idx = idx * 2;
            std::memcpy(&val, &ch.data[byte_idx], 2);
            break;
        }
        case ChannelField::UINT32: {
            uint32_t tmp = 0;
            size_t byte_idx = idx * 4;
            std::memcpy(&tmp, &ch.data[byte_idx], 4);
            val = static_cast<uint16_t>(tmp);
            break;
        }
        default:
            break;
    }
    return val;
}

}  // anonymous namespace

void lidarScanToPointCloud(
    const ouster_sensor_msgs::msg::LidarScan& scan,
    const ouster_sensor_msgs::msg::LidarInfo& info,
    sensor_msgs::msg::PointCloud2& cloud_msg,
    bool destagger) {

    const uint32_t h = scan.height;
    const uint32_t w = scan.width;

    if (h == 0 || w == 0) {
        throw std::invalid_argument("LidarScan dimensions must be > 0");
    }

    if (info.pixels_per_column != h || info.columns_per_frame != w) {
        throw std::invalid_argument(
            "LidarInfo dimensions do not match LidarScan dimensions");
    }

    Eigen::Array<double, Eigen::Dynamic, 3, Eigen::RowMajor> direction, offset;
    buildXYZLut(info, direction, offset);

    const auto* range_ch = findChannel(scan, "range");
    if (!range_ch) {
        throw std::invalid_argument("LidarScan missing required 'range' channel");
    }

    const auto* signal_ch = findChannel(scan, "signal");
    const auto* reflectivity_ch = findChannel(scan, "reflectivity");
    const auto* near_ir_ch = findChannel(scan, "near_ir");

    std::vector<int> pixel_shift(h, 0);
    if (destagger && info.pixel_shift_by_row.size() == h) {
        for (uint32_t row = 0; row < h; ++row) {
            pixel_shift[row] = info.pixel_shift_by_row[row];
        }
    }

    // Organized point cloud: height × width.
    cloud_msg.header = scan.header;
    cloud_msg.height = h;
    cloud_msg.width = w;
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = false;

    // Layout (28 bytes per point):
    //   x:            float32  offset  0
    //   y:            float32  offset  4
    //   z:            float32  offset  8
    //   range:        uint32   offset 12
    //   signal:       uint16   offset 16
    //   reflectivity: uint16   offset 18
    //   near_ir:      uint16   offset 20
    //   t:            uint32   offset 22
    //   ring:         uint16   offset 26
    cloud_msg.fields.clear();
    auto addField = [&](const std::string& name, uint32_t off, uint8_t dt) {
        sensor_msgs::msg::PointField f;
        f.name = name;
        f.offset = off;
        f.datatype = dt;
        f.count = 1;
        cloud_msg.fields.push_back(f);
    };
    addField("x",            0,  sensor_msgs::msg::PointField::FLOAT32);
    addField("y",            4,  sensor_msgs::msg::PointField::FLOAT32);
    addField("z",            8,  sensor_msgs::msg::PointField::FLOAT32);
    addField("range",       12,  sensor_msgs::msg::PointField::UINT32);
    addField("signal",      16,  sensor_msgs::msg::PointField::UINT16);
    addField("reflectivity",18,  sensor_msgs::msg::PointField::UINT16);
    addField("near_ir",     20,  sensor_msgs::msg::PointField::UINT16);
    addField("t",           22,  sensor_msgs::msg::PointField::UINT32);
    addField("ring",        26,  sensor_msgs::msg::PointField::UINT16);

    cloud_msg.point_step = 28;
    cloud_msg.row_step = cloud_msg.point_step * w;
    cloud_msg.data.resize(static_cast<size_t>(cloud_msg.row_step) * h, 0);

    uint64_t scan_ts = 0;
    if (!scan.timestamp.empty()) {
        scan_ts = scan.timestamp[0];
        for (size_t i = 1; i < scan.timestamp.size(); ++i) {
            if (scan.timestamp[i] != 0 && scan.timestamp[i] < scan_ts) {
                scan_ts = scan.timestamp[i];
            }
        }
    }

    const auto* dir_data = direction.data();
    const auto* ofs_data = offset.data();

    for (uint32_t row = 0; row < h; ++row) {
        for (uint32_t col = 0; col < w; ++col) {
            const size_t tgt_idx = static_cast<size_t>(row) * w + col;

            uint32_t src_col;
            if (destagger) {
                int shifted = (static_cast<int>(col) - pixel_shift[row])
                              % static_cast<int>(w);
                if (shifted < 0) shifted += static_cast<int>(w);
                src_col = static_cast<uint32_t>(shifted);
            } else {
                src_col = col;
            }
            const size_t src_idx = static_cast<size_t>(row) * w + src_col;

            const uint32_t range = readUint32(*range_ch, src_idx);

            uint8_t* pt_ptr = &cloud_msg.data[tgt_idx * cloud_msg.point_step];

            float x, y, z;
            if (range == 0) {
                x = y = z = std::numeric_limits<float>::quiet_NaN();
            } else {
                const size_t i3x = src_idx * 3;
                const size_t i3y = src_idx * 3 + 1;
                const size_t i3z = src_idx * 3 + 2;
                x = static_cast<float>(
                    static_cast<double>(range) * dir_data[i3x] + ofs_data[i3x]);
                y = static_cast<float>(
                    static_cast<double>(range) * dir_data[i3y] + ofs_data[i3y]);
                z = static_cast<float>(
                    static_cast<double>(range) * dir_data[i3z] + ofs_data[i3z]);
            }

            std::memcpy(pt_ptr + 0, &x, sizeof(float));
            std::memcpy(pt_ptr + 4, &y, sizeof(float));
            std::memcpy(pt_ptr + 8, &z, sizeof(float));
            std::memcpy(pt_ptr + 12, &range, sizeof(uint32_t));

            uint16_t signal = signal_ch ? readUint16(*signal_ch, src_idx) : 0;
            std::memcpy(pt_ptr + 16, &signal, sizeof(uint16_t));

            uint16_t reflectivity =
                reflectivity_ch ? readUint16(*reflectivity_ch, src_idx) : 0;
            std::memcpy(pt_ptr + 18, &reflectivity, sizeof(uint16_t));

            uint16_t near_ir =
                near_ir_ch ? readUint16(*near_ir_ch, src_idx) : 0;
            std::memcpy(pt_ptr + 20, &near_ir, sizeof(uint16_t));

            uint32_t t = 0;
            if (src_col < scan.timestamp.size() &&
                scan.timestamp[src_col] >= scan_ts) {
                uint64_t diff = scan.timestamp[src_col] - scan_ts;
                t = static_cast<uint32_t>(
                    diff > UINT32_MAX ? UINT32_MAX : diff);
            }
            std::memcpy(pt_ptr + 22, &t, sizeof(uint32_t));

            uint16_t ring = static_cast<uint16_t>(row);
            std::memcpy(pt_ptr + 26, &ring, sizeof(uint16_t));
        }
    }
}

}  // namespace ouster_ros
