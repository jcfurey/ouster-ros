/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file frame_timestamp_tracker.h
 * @brief Associate reception-derived timestamps with completed lidar frames.
 */

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <optional>
#include <vector>

namespace ouster_ros {
namespace impl {

/**
 * Select a ROS-time acquisition-start stamp for a completed scan.
 *
 * Normally the first packet is received near acquisition start and its
 * extrapolated timestamp is the best estimate. A buffered simulator may
 * instead deliver the entire completed scan as one burst, making that first
 * receipt occur at acquisition end. In either case, a complete scan must not
 * end after the ROS time at which its final packet was received.
 */
inline int64_t select_ros_time_frame_start(int64_t first_packet_start_ns,
                                           int64_t completion_receive_ns,
                                           uint64_t scan_span_ns) noexcept {
    if (completion_receive_ns <= 0) return 0;

    const auto completion_ns = static_cast<uint64_t>(completion_receive_ns);
    const int64_t completion_based_start_ns =
        scan_span_ns >= completion_ns
            ? 0
            : completion_receive_ns - static_cast<int64_t>(scan_span_ns);
    return std::max<int64_t>(
        0, std::min(first_packet_start_ns, completion_based_start_ns));
}

/**
 * Tracks the first reception-derived timestamp observed for each lidar frame.
 *
 * ScanBatcher can finish either on the final packet of a complete frame or on
 * the first packet of the following frame when the previous frame was
 * incomplete. Keying timestamps by frame id handles both cases without
 * assuming which packet triggers completion.
 */
class FrameTimestampTracker {
   public:
    explicit FrameTimestampTracker(std::size_t capacity = 4)
        : capacity_{capacity} {
        entries_.reserve(capacity_);
    }

    bool contains(uint32_t frame_id) const {
        return std::any_of(
            entries_.begin(), entries_.end(), [frame_id](const Entry& entry) {
                return entry.frame_id == frame_id;
            });
    }

    void observe(uint32_t frame_id, int64_t timestamp_ns) {
        const auto existing = std::find_if(
            entries_.begin(), entries_.end(), [frame_id](const Entry& entry) {
                return entry.frame_id == frame_id;
            });
        if (existing != entries_.end()) return;

        if (capacity_ == 0) return;
        if (entries_.size() == capacity_) entries_.erase(entries_.begin());
        entries_.push_back({frame_id, timestamp_ns});
    }

    std::optional<int64_t> take(uint32_t frame_id) {
        const auto entry = std::find_if(
            entries_.begin(), entries_.end(), [frame_id](const Entry& value) {
                return value.frame_id == frame_id;
            });
        if (entry == entries_.end()) return std::nullopt;

        const auto timestamp_ns = entry->timestamp_ns;
        entries_.erase(entries_.begin(), std::next(entry));
        return timestamp_ns;
    }

   private:
    struct Entry {
        uint32_t frame_id;
        int64_t timestamp_ns;
    };

    std::size_t capacity_;
    std::vector<Entry> entries_;
};

}  // namespace impl
}  // namespace ouster_ros
