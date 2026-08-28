// Copyright 2026 John Cameron Furey
// SPDX-License-Identifier: BSD-3-Clause

#include <fstream>
#include <iterator>
#include <string>

#include <gtest/gtest.h>

namespace {

TEST(ReplayLaunchTest, DecodedMessagesFollowPublishedBagClockByDefault) {
    const std::string launch_path =
        std::string{OUSTER_ROS_SOURCE_DIR} +
        "/launch/replay.composite.launch.xml";
    std::ifstream launch_file{launch_path};
    ASSERT_TRUE(launch_file.is_open()) << launch_path;
    const std::string contents{
        std::istreambuf_iterator<char>{launch_file},
        std::istreambuf_iterator<char>{}};

    EXPECT_NE(contents.find(
                  "<set_parameter name=\"use_sim_time\" value=\"true\" />"),
              std::string::npos);
    EXPECT_NE(contents.find(
                  "<arg name=\"timestamp_mode\" "
                  "default=\"TIME_FROM_ROS_TIME\""),
              std::string::npos);
    EXPECT_NE(contents.find(
                  "<arg name=\"clock_publish_frequency\" default=\"1000.0\""),
              std::string::npos);
    EXPECT_NE(contents.find(
                  "ros2 bag play $(var bag_file) --clock "
                  "$(var clock_publish_frequency)"),
              std::string::npos);
    EXPECT_NE(contents.find(
                  "<arg name=\"replay_recorded_tf\" default=\"true\""),
              std::string::npos);
    EXPECT_NE(contents.find(
                  "value=\"--exclude-topics /tf /tf_static\""),
              std::string::npos);
    EXPECT_NE(contents.find("$(var _recorded_tf_filter)"),
              std::string::npos);
}

}  // namespace
