#include <algorithm>

struct PIDComponents {
    float p, i, d, raw_output;
};

class PIDControl
{
private:
    const float kp_, ki_, kd_;
    //for integral
    float error_integral_;
    //for derivative
    float last_error_;
    bool is_first_calculation = true;
    float output_p_ = 0.0, output_i_ = 0.0, output_d_ = 0.0, raw_output_ = 0.0;
public:
    PIDControl(const float kp_, const float ki_, const float kd_)
    : kp_(kp_), ki_(ki_), kd_(kd_), error_integral_(0), last_error_(0)
    {
        
    }
    ~PIDControl(){};
    

    //use when there is no rate sensor
    float calculatePIDOutput(float target, float current_value, float dt)
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
        //Not using Deadband
        float output_i = 0.0;
        if(this->ki_ >= 0.0001f)   //divide by zero guarding
        {
            float clamp_output_i = 0.1f;
            float max_error_intergral = clamp_output_i/this->ki_;
            if(std::abs(current_value) <= 10)
            {
                this->error_integral_ += error * dt;
                this->error_integral_ = std::clamp(this->error_integral_, -max_error_intergral, max_error_intergral);
            } 
            else
            {
                this->error_integral_ *= 0.99;
            }
            output_i = this->error_integral_ * this->ki_;
            output_i = std::clamp(output_i, -clamp_output_i, clamp_output_i);

        }
        //Proportional
        float output_p = error * this->kp_;
        this->output_p_ = output_p;
        this->output_i_ = output_i;
        this->output_d_ = output_d;
        float raw_output = std::clamp(output_p + output_i + output_d, -1.0f, 1.0f);
        this->raw_output_ = raw_output;
        return raw_output;    //return percentage
    }

    //use when there is rate sensor
    double calculatePIDOutput(float target, float current_value, float rate_value, float dt)
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
            output_d = -rate_value * this->kd_;
        }
        this->last_error_ = error;
        //Integral
        //Not using Deadband
        float output_i = 0.0;
        if(this->ki_ >= 0.0001f)   //divide by zero guarding
        {
            float clamp_output_i = 0.1f;
            float max_error_intergral = clamp_output_i/this->ki_;
            if(std::abs(current_value) <= 10)
            {
                this->error_integral_ += error * dt;
                this->error_integral_ = std::clamp(this->error_integral_, -max_error_intergral, max_error_intergral);
            } 
            else
            {
                this->error_integral_ *= 0.99;
            }
            output_i = this->error_integral_ * this->ki_;
            output_i = std::clamp(output_i, -clamp_output_i, clamp_output_i);

        }
        //Proportional
        float output_p = error * this->kp_;
        this->output_p_ = output_p;
        this->output_i_ = output_i;
        this->output_d_ = output_d;
        float raw_output = std::clamp(output_p + output_i + output_d, -1.0f, 1.0f);
        this->raw_output_ = raw_output;
        return raw_output;    //return percentage
    }

    PIDComponents getPIDvalues()
    {
        return {this->output_p_, this->output_i_, this->output_d_, this->raw_output_};
    }

    void reset()
    {
        this->error_integral_ = 0.0;
        this->is_first_calculation = true;
        this->last_error_ = 0.0;
    }
};