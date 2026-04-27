#include <algorithm>

class PIDControl
{
private:
    const float kp_, ki_, kd_;
    //for integral
    double error_integral_;
    //for derivative
    float last_value_;
    bool is_first_calculation = true;
public:
    PIDControl(const float kp_, const float ki_, const float kd_)
    : kp_(kp_), ki_(ki_), kd_(kd_), error_integral_(0), last_value_(0)
    {
        
    }
    ~PIDControl(){};
    
    double calculatePIDOutput(float target, float current_value, float dt)
    {
        //Derivative
        float output_d = 0;
        if(is_first_calculation == true)
        {
            is_first_calculation = false;
        }
        else
        {
            output_d = (current_value - last_value_) / dt * this->kd_;
        }
        last_value_ = current_value;
        //Integral
        float error = target - current_value;
        if(std::abs(current_value) <= 10)
        {
            this->error_integral_ += error * dt;
        } 
        else
        {
            this->error_integral_ *= 0.9;
        }
        float output_i = this->error_integral_ * this->ki_;
        //Proportional
        float output_p = error * this->kp_;
        return output_p + output_i + output_d;
    }
};