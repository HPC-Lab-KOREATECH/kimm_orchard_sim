#ifndef LON_CONTROLLER_HPP
#define LON_CONTROLLER_HPP

#include <iostream>

#include "control/PIDController.hpp"


class LonController
{
private:
    shared_ptr<PIDController> gas_pid;
    shared_ptr<PIDController> brake_pid;

    float current_speed_;
    float target_speed_;

    float gas_scale_;  // gas : 0 ~ 6.9444 m/s
    float brake_scale_;  // brake : 0.f ~ 10

    float pre_calc_speed = 0.0;
    float enable_brake_error = 0.2;


public:
    LonController()
        : gas_scale_(6.9), brake_scale_(200) {
            gas_pid = std::make_shared<PIDController>(-1.0, 1.0);
            brake_pid = std::make_shared<PIDController>(-1.0, 1.0);
        }

    void set_lon_data(float current_speed)
    {
        this->current_speed_ = current_speed;
    }

    void set_lon_target_speed(float target_speed)
    {
        this->target_speed_ = target_speed;
    }

    double get_lon_target_speed()
    {
        return this->target_speed_;
    }

    void set_enable_brake_error(float enable_error)
    {
        this->enable_brake_error = enable_error;
    }


    void set_lon_PD_gain(double ga_kp, double ga_kd, double br_kp, double br_kd)
    {
        // i gain은 편의상 0.0으로 고정.
        gas_pid->set_PID_gain(ga_kp, ga_kd);
        brake_pid->set_PID_gain(br_kp, br_kd);
    }

    GasAndBrake calc_gas_n_brake()
    {
        // float enable_error = 0.2;
        float error = target_speed_ - current_speed_;
        GasAndBrake return_val;
        double gas_val = gas_pid->compute(target_speed_, current_speed_, 0.1);

        if (-this->enable_brake_error < error)
        {
            return_val.gas = target_speed_ + gas_val * gas_scale_;
            return_val.brake = 0;
        }
        else if (-2*this->enable_brake_error < error && error <= -this->enable_brake_error)
        {
            return_val.gas = target_speed_;
            return_val.brake = 0;
        }
        else
        {
            double brake_val = brake_pid->compute(target_speed_, current_speed_, 0.05);
            return_val.gas = 0 * gas_scale_;
            return_val.brake = 30 -brake_val * brake_scale_;
        }

        return_val.gas = clip(return_val.gas, 0.0, 6.9);
        // return_val.gas = low_pass_filter(return_val.gas, this->pre_calc_speed, 0.6);
        // return_val.brake = clip(return_val.brake, 1.0, 199.0);
        return_val.brake = clip(return_val.brake, 0.0, 199.0);

        this->pre_calc_speed = return_val.gas;

        return return_val;
    }

    float get_target_speed()
    {
        return this->target_speed_;
    }
};

#endif // LON_CONTROLLER_HPP
