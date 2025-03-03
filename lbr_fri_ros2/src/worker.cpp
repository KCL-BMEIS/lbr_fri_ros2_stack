#include "lbr_fri_ros2/worker.hpp"

namespace lbr_fri_ros2 {
Worker::Worker() : should_stop_(true), running_(false) {}

Worker::~Worker() {
  this->request_stop();
  if (run_thread_.joinable()) {
    run_thread_.join();
  }
}

void Worker::run_async(int rt_prio) {
  if (running_) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                       ColorScheme::WARNING << "Worker already running" << ColorScheme::ENDC);
    return;
  }
  run_thread_ = std::thread([this, rt_prio]() {
    if (!realtime_tools::configure_sched_fifo(rt_prio)) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                         ColorScheme::WARNING
                             << "Failed to set FIFO realtime scheduling policy. Refer to "
                                "[https://control.ros.org/master/doc/ros2_control/"
                                "controller_manager/doc/userdoc.html]."
                             << ColorScheme::ENDC);
    } else {
      RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                         ColorScheme::OKGREEN
                             << "Realtime scheduling policy set to FIFO with priority '" << rt_prio
                             << "'" << ColorScheme::ENDC);
    }

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME()), "Starting run thread");
    should_stop_ = false;

    // perform work in child-class
    this->perform_work_();
    // perform work end

    running_ = false;
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME()), "Exiting run thread");
  });
}

void Worker::request_stop() {
  if (!running_) {
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME()), "Requesting run thread stop");
  should_stop_ = true;
}
} // namespace lbr_fri_ros2
