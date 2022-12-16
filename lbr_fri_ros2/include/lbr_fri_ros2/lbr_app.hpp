#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <stdexcept>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include "lbr_fri_msgs/srv/app_connect.hpp"
#include "lbr_fri_msgs/srv/app_disconnect.hpp"
#include "lbr_fri_ros2/lbr_state_client.hpp"

#include "fri/friUdpConnection.h"
#include "fri/friClientApplication.h"

namespace lbr_fri_ros2
{
    class LBRApp : public rclcpp::Node
    {
    public:
        LBRApp(const std::string &node_name, const int &port_id = 30200, const char *const remote_host = NULL);
        ~LBRApp();

    protected:
        void app_connect_cb_(
            const lbr_fri_msgs::srv::AppConnect::Request::SharedPtr request,
            lbr_fri_msgs::srv::AppConnect::Response::SharedPtr response);

        void app_disconnect_cb_(
            const lbr_fri_msgs::srv::AppDisconnect::Request::SharedPtr /*request*/,
            lbr_fri_msgs::srv::AppDisconnect::Response::SharedPtr response);

        bool valid_port_(const int &port_id);
        bool connect_(const int &port_id = 30200, const char *const remote_host = NULL);
        bool disconnect_();

        std::unique_ptr<std::thread> app_step_thread_;

        const char *remote_host_;
        int port_id_;

        std::atomic<bool> connected_;

        rclcpp::Service<lbr_fri_msgs::srv::AppConnect>::SharedPtr app_connect_srv_;
        rclcpp::Service<lbr_fri_msgs::srv::AppDisconnect>::SharedPtr app_disconnect_srv_;

        std::unique_ptr<lbr_fri_ros2::LBRStateClient> lbr_state_client_;
        std::unique_ptr<KUKA::FRI::UdpConnection> connection_;
        std::unique_ptr<KUKA::FRI::ClientApplication> app_;
    };
} // end of namespace lbr_fri_ros2