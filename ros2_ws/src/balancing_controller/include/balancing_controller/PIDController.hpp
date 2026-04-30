#include <algorithm>
#include "rclcpp/rclcpp.hpp"

struct PIDComponents {
    float p, i, d;
};

class PIDControl
{
private:
    const float kp_, ki_, kd_;
    //for integral
    double error_integral_;
    //for derivative
    float last_error_;
    bool is_first_calculation = true;
    float output_p_ = 0.0, output_i_ = 0.0, output_d_ = 0.0; 
public:
    PIDControl(const float kp_, const float ki_, const float kd_)
    : kp_(kp_), ki_(ki_), kd_(kd_), error_integral_(0), last_error_(0)
    {
        
    }
    ~PIDControl(){};
    
    double calculatePIDOutput(float target, float current_value, float dt)
    {
        //Derivative
        float error = target - current_value;
        float output_d = 0;
        if(this->is_first_calculation == true)
        {
            this->is_first_calculation = false;
        }
        else
        {
            output_d = (error - this->last_error_) / dt * this->kd_;
        }
        this->last_error_ = error;
        //Integral
        //Deadband
        if(std::abs(error) > 0.2)
        {
            if(std::abs(current_value) <= 10)
            {
                this->error_integral_ += error * dt;
                this->error_integral_ = std::clamp(this->error_integral_, -20.0, 20.0);
            } 
            else
            {
                this->error_integral_ *= 0.99;
            }
        }
        float output_i = this->error_integral_ * this->ki_;
        //Proportional
        float output_p = error * this->kp_;
        this->output_p_ = output_p;
        this->output_i_ = output_i;
        this->output_d_ = output_d;
        return output_p + output_i + output_d;
    }

    PIDComponents getPIDvalues()
    {
        return {this->output_p_, this->output_i_, output_d_};
    }

    void reset()
    {
        this->error_integral_ = 0.0;
        this->is_first_calculation = true;
        this->last_error_ = 0.0;
    }
};