#include "lbr_fri_ros2/app.hpp"

namespace lbr_fri_ros2 {
App::App(const std::shared_ptr<AsyncClient> async_client_ptr)
    : async_client_ptr_(nullptr), connection_ptr_(nullptr), app_ptr_(nullptr) {
  async_client_ptr_ = async_client_ptr;
  connection_ptr_ = std::make_unique<KUKA::FRI::UdpConnection>();
  app_ptr_ = std::make_unique<KUKA::FRI::ClientApplication>(*connection_ptr_, *async_client_ptr_);
}

App::~App() {
  request_stop();
  close_udp_socket();
}

bool App::open_udp_socket(const int &port_id, const char *const remote_host) {
  if (!connection_ptr_) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << "Connection not configured" << ColorScheme::ENDC);
    return false;
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                     ColorScheme::OKBLUE << "Opening UDP socket with port_id '" << ColorScheme::BOLD
                                         << port_id << "'" << ColorScheme::ENDC);
  if (!valid_port_(port_id)) {
    return false;
  }
  if (connection_ptr_->isOpen()) {
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME()), "Socket already open");
    return true;
  }
  if (!connection_ptr_->open(port_id, remote_host)) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << "Failed to open socket" << ColorScheme::ENDC);
    return false;
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                     ColorScheme::OKGREEN << "Socket opened successfully" << ColorScheme::ENDC);
  return true;
}

bool App::close_udp_socket() {
  if (!connection_ptr_) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << "Connection not configured" << ColorScheme::ENDC);
    return false;
  }
  while (running_) {
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME()), "Waiting for run thread termination");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                     ColorScheme::OKBLUE << "Closing UDP socket" << ColorScheme::ENDC);
  if (!connection_ptr_->isOpen()) {
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME()), "Socket already closed");
    return true;
  }
  connection_ptr_->close();
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                     ColorScheme::OKGREEN << "Socket closed successfully" << ColorScheme::ENDC);
  return true;
}

void App::run_async(int rt_prio) {
  if (!async_client_ptr_) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << "AsyncClient not configured" << ColorScheme::ENDC);
    return;
  }
  if (!connection_ptr_) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << "Connection not configured" << ColorScheme::ENDC);
    return;
  }
  if (!connection_ptr_->isOpen()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << "Connection not open" << ColorScheme::ENDC);
    return;
  }
  if (!app_ptr_) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << "App not configured" << ColorScheme::ENDC);
    return;
  }

  // call base class run_async post checks
  Worker::run_async(rt_prio);
  run_thread_.detach();
}

void App::perform_work_() {
  bool success = true;
  while (rclcpp::ok() && success && !should_stop_) {
    success = app_ptr_->step(); // stuck if never connected, we thus detach the run_thread_ as join
                                // may never return
    running_ = true;
    if (async_client_ptr_->robotState().getSessionState() == KUKA::FRI::ESessionState::IDLE) {
      RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME()), "LBR in session state idle, exiting");
      break;
    }
  }
  async_client_ptr_->get_state_interface()->uninitialize();
}

bool App::valid_port_(const int &port_id) {
  if (port_id < 30200 || port_id > 30209) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGER_NAME()),
                        ColorScheme::ERROR << "Expected port_id in [30200, 30209], got '"
                                           << ColorScheme::BOLD << port_id << "'"
                                           << ColorScheme::ENDC);
    return false;
  }
  return true;
}
} // namespace lbr_fri_ros2
