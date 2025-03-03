#ifndef LBR_FRI_ROS2__WORKER_HPP_
#define LBR_FRI_ROS2__WORKER_HPP_

#include <atomic>
#include <string>
#include <thread>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "realtime_tools/realtime_helpers.hpp"

#include "lbr_fri_ros2/formatting.hpp"

namespace lbr_fri_ros2 {
class Worker {
public:
  Worker();
  ~Worker();

  virtual void run_async(int rt_prio = 80);
  void request_stop();
  inline virtual std::string LOGGER_NAME() const = 0;

protected:
  virtual void perform_work_() = 0;

  std::atomic_bool should_stop_, running_;
  std::thread run_thread_;
};
} // namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__WORKER_HPP_
