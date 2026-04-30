#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "balancing_controller/PIDController.hpp"

// --- PASTE YOUR PIDCONTROL CLASS HERE ---

#define Kp 0.1      // Significantly lower the "muscle"
#define Ki 0.0       // Keep at zero
#define Kd 0.0      // Significantly increase the "braking" force
#define SCALE 1       //to scale down the output of PID algorithm 
#define GYRO_Y_PERCENTAGE 0.995
#define ACCEL_PITCH_DEG_PERCENTAGE (1-GYRO_Y_PERCENTAGE)

class BalancerNode : public rclcpp::Node {
private:
    std::unique_ptr<PIDControl> pid_;
    double target_angle_ = 0.0;
    rclcpp::Time last_time_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motor_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    double filtered_pitch = 0;
    bool is_filtered_pitch_first_initialized = false;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // 1. Timing
        auto now = this->now();

        if(this->last_time_.nanoseconds() <= 0 || now.seconds() <= this->last_time_.seconds())
        {
            RCLCPP_INFO(this->get_logger(), "Synchronizing time!\n");
            this->last_time_ = now;
            return;
        }

        double dt = (now - last_time_).seconds();
        RCLCPP_INFO(this->get_logger(), "now %f dt %f\n", now.seconds(), dt);
        if (dt <= 0) return;

        // 2. Physical Orientation (Calculate from Accelerometer raw data)
        double acc_x = msg->linear_acceleration.x;
        double acc_z = msg->linear_acceleration.z;
        RCLCPP_INFO(this->get_logger(), "X: %f, Z: %f", acc_x, acc_z);
    
        double gyro_y = msg->angular_velocity.y; // Rad/sec, get the gyroscope data for instant data
        // atan2 gives us the tilt angle in radians based on gravity
        double accel_pitch_rad = atan2(acc_x, acc_z); // get the accelerometer for long term data
        float accel_pitch_deg = accel_pitch_rad * 57.2958; // Convert to degrees

        // Complementary Filter: 98% Gyro / 2% Accel
        if(this->is_filtered_pitch_first_initialized == false)
        {
            this->filtered_pitch = accel_pitch_deg;
            this->is_filtered_pitch_first_initialized = true;
        }
        else
        {
            this->filtered_pitch =  GYRO_Y_PERCENTAGE * (this->filtered_pitch + (-gyro_y) * dt * 57.2958) + ACCEL_PITCH_DEG_PERCENTAGE * accel_pitch_deg; 
        }

        auto drive = geometry_msgs::msg::Twist();
        double output = 0.0;
        if(std::abs(filtered_pitch) >= 30)
        {
            (this->pid_)->reset();
            drive.linear.x = 0;
            drive.linear.z = 0;
            RCLCPP_INFO(this->get_logger(), "Reseting robot and PID controller");
        }
        else
        {
            // 3. THE MATH, PIDControl calss
            output = pid_->calculatePIDOutput(target_angle_, filtered_pitch, dt);
            
            // 4. Act (Scale the output for Gazebo meters/second)
            output = std::clamp(output*SCALE, -3.0, 3.0); // Scale down so it doesn't fly away
            drive.linear.x = output;
            RCLCPP_INFO(this->get_logger(), "Pitch: %f, Output: %f", filtered_pitch, output);
            PIDComponents PIDvalues = pid_->getPIDvalues();
            RCLCPP_INFO(this->get_logger(), "P: %f, I: %f, D: %f", PIDvalues.p, PIDvalues.i, PIDvalues.d);
        }
        RCLCPP_INFO(this->get_logger(), "accel_pitch_deg: %f, gyro_y: %f", accel_pitch_deg, gyro_y);
        // drive.linear.x = 0;
        this->motor_pub_->publish(drive);

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
        pid_ = std::make_unique<PIDControl>(Kp, Ki, Kd); 
        
        this->last_time_ = this->now(); //assign utc timestamp to this->last_time_
        std::cout << this->last_time_.seconds() << "\n";
    }
};

int main(int argc, char ** argv) {
    std::cout << "START\n";
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BalancerNode>());
    rclcpp::shutdown();
    return 0;
}