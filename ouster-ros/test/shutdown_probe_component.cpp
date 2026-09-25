// SPDX-License-Identifier: BSD-3-Clause

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace ouster_ros {
namespace test {

// Keep the signal-handler thread busy after it wakes the executor. The node
// must remain alive until that thread finishes context/logging cleanup.
class ShutdownProbe : public rclcpp::Node {
   public:
    explicit ShutdownProbe(const rclcpp::NodeOptions& options)
        : Node("shutdown_probe", options),
          shutdown_finished_(std::make_shared<std::atomic<bool>>(false)) {
        shutdown_callback_ = get_node_base_interface()->get_context()->add_on_shutdown_callback(
            [finished = shutdown_finished_] {
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
                finished->store(true);
            });
        ready_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {
            ready_timer_->cancel();
            std::cout << "shutdown probe ready" << std::endl;
        });
    }

    ~ShutdownProbe() override {
        if (!shutdown_finished_->load()) {
            std::cerr << "node destroyed before shutdown finished" << std::endl;
            std::_Exit(42);
        }
        // The context outlives this dynamically loaded fixture. Remove the
        // callback while its code and captured state are still in memory.
        get_node_base_interface()->get_context()->remove_on_shutdown_callback(
            shutdown_callback_);
        std::cout << "node destroyed after shutdown finished" << std::endl;
    }

   private:
    std::shared_ptr<std::atomic<bool>> shutdown_finished_;
    rclcpp::OnShutdownCallbackHandle shutdown_callback_;
    rclcpp::TimerBase::SharedPtr ready_timer_;
};

}  // namespace test
}  // namespace ouster_ros

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::test::ShutdownProbe)
