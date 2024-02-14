#ifndef LAT_CONTROLLERS_HPP
#define LAT_CONTROLLERS_HPP

#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

#include "control/PIDController.hpp"
#include "control/callback_data_manage.hpp"

class Stanley
{
private:
    double kp_;             // Proportional gain
    double ki_;             // Integral gain
    double kd_;             // Derivative gain
    double target_heading_; // Target heading
    double prevCrossTrackError;
    double integral;
    double anti_windup_max_;
    double Lf;
    double Lr;
    double L;
    double width;
    double crossTrackError_;
    float speed_;
    float ego_heading_;
    double curvature_;
    double heading_gain_;
    double heading_term;
        
public:
    Stanley()
        : prevCrossTrackError(0.0), integral(0.0)
    {
        Lf = 0.494/2;
        Lr = 0.494/2;
        L = 0.494;
        width = 0.364;
        
        kp_ = 0.1;
        ki_ = 0.0;
        // kd_ = 1.5;
        target_heading_ = 0.0;
        ego_heading_ = 0.0;

        crossTrackError_ = 0.0;
        speed_ = 0.0;
        


        anti_windup_max_ = 0.0;
        heading_gain_ = 1.0;

        heading_term = 0.0;
    }

    void set_stanly_gain(double kp, double ki, double kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void set_heading_gain(double h_gain)
    {
        this->heading_gain_ = h_gain;
    }

    void set_anti_windup_max(double anti_windup_max)
    {
        this->anti_windup_max_ = anti_windup_max;
    }

    void set_stanly_data(float speed, double target_heading, float ego_heading,
                         double crossTrackError, double curvature = 0.0)
    {
        this->speed_ = clip(speed, 0.9F, 10.0F);
        // this->speed_ = speed;
        this->target_heading_ = nomalize_angle(target_heading);
        this->ego_heading_ = nomalize_angle(ego_heading);
        this->crossTrackError_ = crossTrackError;
        this->curvature_ = curvature; // 튜닝에 사용하는 param 일단은 0.0
    }



    PointFR calc_stanley_steer(double dt = 1)
    {
        // Calculate the heading error


        double tmp_headin_error = nomalize_angle(target_heading_ - ego_heading_);

        // double tmp_headin_error = (target_heading_ - ego_heading_);
        double headingError = this->heading_gain_ * (tmp_headin_error);

        // Normalize the heading error to the range [-pi, pi]
        // headingError = nomalize_angle(headingError);
        heading_term = headingError;
        // Calculate the proportional term
        double proportionalTerm = kp_ * crossTrackError_;

        // Calculate the cross track error derivative
        double crossTrackErrorDerivative = (crossTrackError_ - prevCrossTrackError) / dt;

        // Calculate the derivative term
        double derivativeTerm = kd_ * crossTrackErrorDerivative;

        // Update the previous cross track error for the next iteration
        prevCrossTrackError = crossTrackError_;

        // Calculate the integral term
        integral += crossTrackError_ * dt;
        integral = clip(integral, -anti_windup_max_, anti_windup_max_);

        // if (this->speed_ < 1)
        // {
        //     integral = 0;
        // }

        double integralTerm = ki_ * integral;

        // Calculate the steering angle using Stanley Method
        double PID_steer = proportionalTerm + derivativeTerm + integralTerm;

        double headingErrorTermF = headingError * Lf/(Lf+Lr); //4ws
        double headingErrorTermR = -headingError * Lr/(Lf+Lr); //4ws

        // cout << headingError << endl;
        // cout << headingErrorTermR << endl;
        double steeringAngleF = headingErrorTermF + atan2(PID_steer, (0.1 + this->speed_)); //4ws
        double steeringAngleR = headingErrorTermR + atan2(PID_steer, (0.1 + this->speed_)); //4ws
        PointFR steeringAngle;

        steeringAngle.F = nomalize_angle(steeringAngleF);
        steeringAngle.R = nomalize_angle(steeringAngleR);

        return steeringAngle;
    }

    double get_heading_term()
    {
        return heading_term;
    }

    double get_stanley_P_gain()
    {
        return kp_;
    }

    double get_stanley_integral_val()
    {
        return this->integral;
    }

    float get_speed_in_stanley()
    {
        return this->speed_;
    }

    double set_stanley_integral_val(double inte_init)
    {
        this->integral = inte_init;
    }
};

class FeedForward
{
private:
    double Lf;
    double Lr;
    double curvature_;
public:
    FeedForward()
    {
        Lf = 0.494/2;
        Lr = 0.494/2;
    }

    void set_curvature(double curvature) 
    {
        this->curvature_ = curvature;
    }
    
    double calc_FF_SteerF()
    {
        double FF_SteerF = atan2(curvature_ * Lf, 1.0)*0.5;

        return FF_SteerF;
    }

    double calc_FF_SteerR()
    {
        double FF_SteerR = atan2(curvature_ * Lr, 1.0)*0.5;

        return FF_SteerR;
    }
};



class CombinedSteer : public FeedForward, public Stanley
{
private:
    CallbackClass *cb_data_;

    float FF_weight_, stanly_weight_;
    double deltaMax;

    double pre_com_steerF;
    double pre_com_steerR;
    float com_steer_alpha;
    double width;
    double L;
    double FF_steerF;
    double FF_steerR;
    PointFR stanley_steer;
    

public:
    CombinedSteer(CallbackClass *cb_data)
        : FeedForward(), Stanley(), FF_weight_(1),
          stanly_weight_(1), deltaMax(3.141592/2.0)
    {

        this->cb_data_ = cb_data;
        L = 0.494;
        width = 0.364;
        pre_com_steerF = 0;
        pre_com_steerR = 0;
        com_steer_alpha = 0.9;

    }


    PointFR calc_combined_steer()
    {
        double FeedForwardSteeringF = calc_FF_SteerF();
        double FeedForwardSteeringR = calc_FF_SteerR();

        FF_steerF = clip(FeedForwardSteeringF, -deltaMax, deltaMax);
        FF_steerR = clip(FeedForwardSteeringR, -deltaMax, deltaMax);

        if (cb_data_->get_odom_sub_flag())
        {
            // stanley
            this->stanley_steer = calc_stanley_steer();
            this->stanley_steer.F = clip(this->stanley_steer.F, -deltaMax, deltaMax);
            this->stanley_steer.R = clip(this->stanley_steer.R, -deltaMax, deltaMax);
            cb_data_->set_down_odom_sub_flag();
        }

        // Combine the steering angles
        PointFR combinedSteering;
        combinedSteering.F = FF_weight_ * FF_steerF + stanly_weight_ * stanley_steer.F;
        combinedSteering.R = FF_weight_ * FF_steerR + stanly_weight_ * stanley_steer.R;

        double combined_steerF = clip(combinedSteering.F, -deltaMax, deltaMax);
        double combined_steerR = clip(combinedSteering.R, -deltaMax, deltaMax);

        double com_lpf_steerF = low_pass_filter(combined_steerF, pre_com_steerF, com_steer_alpha);
        double com_lpf_steerR = low_pass_filter(combined_steerR, pre_com_steerR, com_steer_alpha);

        pre_com_steerF = com_lpf_steerF;
        pre_com_steerR = com_lpf_steerR;

        PointFR com_lpf_steer;
        com_lpf_steer.F = com_lpf_steerF;
        com_lpf_steer.R = com_lpf_steerR;


        com_lpf_steer.FL = atan2(tan(com_lpf_steerF) , (1 - (width/(2*L)) * (tan(com_lpf_steerF) - tan(com_lpf_steerR)))); //4ws
        com_lpf_steer.FR = atan2(tan(com_lpf_steerF) , (1 + (width/(2*L)) * (tan(com_lpf_steerF) - tan(com_lpf_steerR)))); //4ws
        com_lpf_steer.RL = atan2(tan(com_lpf_steerR) , (1 - (width/(2*L)) * (tan(com_lpf_steerF) - tan(com_lpf_steerR)))); //4ws
        com_lpf_steer.RR = atan2(tan(com_lpf_steerR) , (1 + (width/(2*L)) * (tan(com_lpf_steerF) - tan(com_lpf_steerR)))); //4ws
        
        
        com_lpf_steer.FL = nomalize_angle(com_lpf_steer.FL);
        com_lpf_steer.FR = nomalize_angle(com_lpf_steer.FR);
        com_lpf_steer.RL = nomalize_angle(com_lpf_steer.RL);
        com_lpf_steer.RR = nomalize_angle(com_lpf_steer.RR);

        com_lpf_steer.FL = clip(com_lpf_steer.FL, -deltaMax, deltaMax);
        com_lpf_steer.FR = clip(com_lpf_steer.FR, -deltaMax, deltaMax);
        com_lpf_steer.RL = clip(com_lpf_steer.RL, -deltaMax, deltaMax);
        com_lpf_steer.RR = clip(com_lpf_steer.RR, -deltaMax, deltaMax);

        return com_lpf_steer;
    }


    double get_FF_steerF()
    {
        return FF_steerF;
    }

    double get_FF_steerR()
    {
        return FF_steerR;
    }

    double get_stanley_steerF()
    {
        return stanley_steer.F;
    }    

    double get_stanley_steerR()
    {
        return stanley_steer.R;
    }    


};


#endif // LAT_CONTROLLERS_HPP
