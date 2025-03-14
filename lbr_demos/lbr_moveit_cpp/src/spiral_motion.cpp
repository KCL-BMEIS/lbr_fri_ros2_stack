#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <fstream>

class SpiralMotionPublisher : public rclcpp::Node
{
public:
    SpiralMotionPublisher()
        : Node("spiral_motion_publisher"), 
        r_(0.012),           // Spiral radius (m)
        v_xy_(0.006),        // Linear velocity in the plane (m/s)
        h_(-0.003),          // Descending distance per revolution (m)
        time_step_(0.01),    // Time step (s)
        t_(0.0),             // Time counter (s)
        H_(0.012),             // Total descending distance (m)
        y_scale(3.6),
        x_scale(3.968),
        z_scale(1.4)
    {
        // Calculate parameters
        double circumference = 2 * M_PI * r_;         // Circumference of the circle (m)
        T_ = circumference / v_xy_;                  // Time per revolution (s)
        omega_ = 2 * M_PI / T_;                       // Angular velocity (rad/s)
        vz_ = h_ / T_;                                // Vertical velocity in y direction (m/s)
        max_time_ = H_ * T_ / std::abs(h_);           // Maximum runtime (s)

        // Create publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/lbr/command/twist", 10);

        // Create timer, callback is executed every 10 ms
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(time_step_ * 1000)),
            std::bind(&SpiralMotionPublisher::publishTwist, this));

        // Open file to log data
        data_file_.open("spiral_motion.txt", std::ios::out);
        if (!data_file_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for logging.");
            rclcpp::shutdown();
        }
        else
        {
            data_file_ << "Time (s), Linear Velocity (x, y, z), Angular Velocity (x, y, z)\n";
        }
    }

    ~SpiralMotionPublisher()
    {
        if (data_file_.is_open())
        {
            data_file_.close();
        }
    }

private:
    void publishTwist()
    {
        // Check if the maximum runtime is exceeded
        if (t_ >= max_time_)
        {
            RCLCPP_INFO(this->get_logger(), "Reached maximum runtime. Shutting down.");
            if (data_file_.is_open())
            {
                data_file_.close(); // Close the file before shutting down
            }
            rclcpp::shutdown(); // Terminate program
            return;
        }

        // Create Twist message
        auto twist_msg = geometry_msgs::msg::Twist();

        // Calculate spiral linear velocity components
        twist_msg.linear.x = r_ * omega_ * std::cos(omega_ * t_) ;// * x_scale;  // Linear velocity in x direction
        
        twist_msg.linear.y = -r_ * omega_ * std::sin(omega_ * t_) ;//* y_scale;   // Linear velocity in z direction
       
        twist_msg.linear.z = vz_ ;//* z_scale;                                  // Linear velocity in y direction

        twist_msg.angular.x = 0.0;  // angular velocity in x direction
        twist_msg.angular.z = 0.0;  // angular velocity in z direction
        twist_msg.angular.y = 0.0;  // angular velocity in y direction

        // Log the current velocities and time
        RCLCPP_INFO(this->get_logger(), "Time: %.2f s, Linear Velocity: [x: %.4f, y: %.4f, z: %.4f], Angular Velocity: [x: %.4f, y: %.4f, z: %.4f]",
                    t_,
                    twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z,
                    twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z);

        // Write data to file
        if (data_file_.is_open())
        {
            data_file_ << t_ << ", "
                       << twist_msg.linear.x << ", " << twist_msg.linear.y << ", " << twist_msg.linear.z << ", "
                       << twist_msg.angular.x << ", " << twist_msg.angular.y << ", " << twist_msg.angular.z << "\n";
        }

        // Publish message
        publisher_->publish(twist_msg);

        // Update time
        t_ += time_step_;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;  // Publisher
    rclcpp::TimerBase::SharedPtr timer_;  // Timer
    std::ofstream data_file_;  // File to log data

    // Spiral parameters
    double r_;         // Spiral radius
    double v_xy_;      // Linear velocity in the plane
    double h_;         // Descending distance per revolution
    double vz_;        // Vertical velocity in y direction
    double T_;         // Time per revolution
    double omega_;     // Angular velocity
    double time_step_; // Time step
    double t_;         // Current time
    double H_;         // Total descending distance
    double max_time_;  // Maximum runtime
    double y_scale;
    double x_scale;
    double z_scale;
};

int main(int argc, char *argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create node
    auto node = std::make_shared<SpiralMotionPublisher>();

    // Run node
    rclcpp::spin(node);

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
