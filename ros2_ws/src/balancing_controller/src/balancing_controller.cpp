#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
// #include "geometry_msgs/msg/twist.hpp"   //no longer send velocity to control motors, transfer torque instead
#include "std_msgs/msg/float64_multi_array.hpp"     //transfer torque to control motors
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "balancing_controller/PIDController.hpp"

#define Kp 0.02      
#define Ki 0.005      
#define Kd 0.009     
#define SCALE 1       //to scale down the output of PID algorithm 
#define GYRO_Y_PERCENTAGE 0.95
#define ACCEL_PITCH_DEG_PERCENTAGE (1-GYRO_Y_PERCENTAGE)
#define MOTOR_SPEED_CLAMPING 5.0
#define MAX_LINEAR_VELOCITY 1.131 //Vmax​=34.8×0.0325=1.131 m/s, <limit velocity="34.8"/>, <wheel_radius>0.0325</wheel_radius>)
#define MAX_TORQUE 0.49

class BalancerNode : public rclcpp::Node {
private:
    std::unique_ptr<PIDControl> pid_;
    double target_angle_ = 0.0;
    rclcpp::Time last_time_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;
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

        // SENSOR INTEGRITY CHECK: Wait for Gazebo physics gravity to kick in
        double acc_mag = std::sqrt(acc_x*acc_x + acc_z*acc_z);
        if (acc_mag < 5.0) {
            RCLCPP_INFO(this->get_logger(), "acc_mag: %f, DROPT FRAME", acc_mag);
            return; // Drop the frame, physics engine is still sending zeros
        }
    
        double gyro_y = msg->angular_velocity.y; // Rad/sec, get the gyroscope data for instant data
        gyro_y *= 57.2958; //convert to deg
        // atan2 gives us the tilt angle in radians based on gravity
        double accel_pitch_rad = atan2(acc_x, acc_z); // get the accelerometer for long term data
        float accel_pitch_deg = accel_pitch_rad * 57.2958; // Convert to degrees

        //IF the robot falls forward, the accel is negative but the gyro_y is positive

        // Complementary Filter: 98% Gyro / 2% Accel
        if(this->is_filtered_pitch_first_initialized == false)
        {
            this->filtered_pitch = accel_pitch_deg;
            this->is_filtered_pitch_first_initialized = true;
        }
        else
        {
            this->filtered_pitch =  GYRO_Y_PERCENTAGE * (this->filtered_pitch + (-gyro_y) * dt) + ACCEL_PITCH_DEG_PERCENTAGE * accel_pitch_deg; 
        }

        auto torque_cmd = std_msgs::msg::Float64MultiArray();
        double pid_output = 0.0;
        if(std::abs(filtered_pitch) >= 30)
        {
            (this->pid_)->reset();
            torque_cmd.data.push_back(0);   //left wheel
            torque_cmd.data.push_back(0);   //right wheel
            RCLCPP_INFO(this->get_logger(), "Reseting robot and PID controller");
        }
        else
        {
            if (std::abs(filtered_pitch) < 0.01) {
                torque_cmd.data.push_back(0);   //left wheel
                torque_cmd.data.push_back(0);   //right wheel
            }
            else
            {
                // 3. THE MATH, PIDControl calss
                pid_output = pid_->calculatePIDOutput(target_angle_, filtered_pitch, -gyro_y, dt);
                float torque_output = pid_output * MAX_TORQUE;
                // 4. Act (Scale the output for Gazebo meters/second)
                torque_cmd.data.push_back(torque_output);   //left wheel
                torque_cmd.data.push_back(torque_output);   //right wheel
                RCLCPP_INFO(this->get_logger(), "filtered_pitch: %f, torque_output: %f", filtered_pitch, torque_output);
                PIDComponents PIDvalues = pid_->getPIDvalues();
                RCLCPP_INFO(this->get_logger(), "P: %f, I: %f, D: %f", PIDvalues.p, PIDvalues.i, PIDvalues.d);
            }
        }

        RCLCPP_INFO(this->get_logger(), "accel_pitch_deg: %f, gyro_y: %f", accel_pitch_deg, gyro_y);
        // drive.linear.x = 3;
        this->effort_pub_->publish(torque_cmd);

        last_time_ = now;
    }

public:
    BalancerNode() : Node("balancer_node") {
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);
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