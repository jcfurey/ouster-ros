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
