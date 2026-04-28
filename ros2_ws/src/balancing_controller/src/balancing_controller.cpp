#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "balancing_controller/PIDController.hpp"

// --- PASTE YOUR PIDCONTROL CLASS HERE ---

class BalancerNode : public rclcpp::Node {
private:
    std::unique_ptr<PIDControl> pid_;
    double target_angle_ = 0.0;
    rclcpp::Time last_time_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motor_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // 1. Timing
        auto now = this->now();

        if(last_time_.nanoseconds() <= 0 || now.seconds() <= last_time_.seconds())
        {
            RCLCPP_INFO(this->get_logger(), "Synchronizing time!\n");
        }

        double dt = (now - last_time_).seconds();
        RCLCPP_INFO(this->get_logger(), "now %f dt %f\n", now.seconds(), dt);
        if (dt <= 0) return;

        // 2. Physical Orientation (Calculate from Accelerometer raw data)
        double acc_x = msg->linear_acceleration.x;
        double acc_z = msg->linear_acceleration.z;
    
        // atan2 gives us the tilt angle in radians based on gravity
        double pitch_rad = atan2(acc_x, acc_z); 
        float pitch_deg = pitch_rad * 57.2958; // Convert to degrees

        // 3. THE MATH, PIDControl calss
        double output = pid_->calculatePIDOutput(target_angle_, pitch_deg, dt);
        RCLCPP_INFO(this->get_logger(), "Pitch: %f, Output: %f", pitch_deg, output);

        // 4. Act (Scale the output for Gazebo meters/second)
        auto drive = geometry_msgs::msg::Twist();
        drive.linear.x = output * 0.1; // Scale down so it doesn't fly away
        motor_pub_->publish(drive);

        last_time_ = now;
    }

public:
    BalancerNode() : Node("balancer_node") {
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        motor_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", qos, std::bind(&BalancerNode::imu_callback, this, std::placeholders::_1));
        
        // Initialize your class with the Arduino gains
        // Remember: Since simulation is 'perfect', start with lower gains!
        pid_ = std::make_unique<PIDControl>(10.0, 5.0, 0.5); 
        
        last_time_ = this->now();
        std::cout << last_time_.seconds() << "\n";
    }
};

int main(int argc, char ** argv) {
    std::cout << "START\n";
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BalancerNode>());
    rclcpp::shutdown();
    return 0;
}