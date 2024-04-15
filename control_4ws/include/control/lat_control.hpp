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
    double heading_integral_;
    double heading_ki_;
    double anti_windup_max_;
    double h_anti_windup_max_;
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
        : prevCrossTrackError(0.0), integral(0.0), heading_integral_(0.0)
    {
        Lf = 0.494/2;
        Lr = 0.494/2;
        L = 0.494;
        width = 0.364;
        
        kp_ = 0.1;
        ki_ = 0.0;
        heading_ki_ = 0.0;
        // kd_ = 1.5;
        target_heading_ = 0.0;
        ego_heading_ = 0.0;

        crossTrackError_ = 0.0;
        speed_ = 0.0;
        

        h_anti_windup_max_ = 0.0;
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

    void set_heading_gain(double h_gain, double h_i_gian = 0.0)
    {
        this->heading_gain_ = h_gain;
        this->heading_ki_  = h_i_gian;
    }

    void set_anti_windup_max(double anti_windup_max)
    {
        this->anti_windup_max_ = anti_windup_max;
    }

    void set_heading_anti_windup_max(double h_anti_windup_max)
    {
        this->h_anti_windup_max_ = h_anti_windup_max;
    }

    void set_heading_integral_term(double heading_integral)
    {
        this->heading_integral_ = heading_integral;
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



    linear_angular calc_stanley_steer(double dt = 1)
    {
        // Calculate the heading error

        linear_angular l_n_a;
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

        heading_integral_ += tmp_headin_error;
        heading_integral_ = clip(heading_integral_, -h_anti_windup_max_, h_anti_windup_max_);
        // if (this->speed_ < 1)
        // {
        //     integral = 0;
        // }

        double integralTerm = ki_ * integral;
        double heading_integral_term = heading_ki_ * heading_integral_;

        cout << "heading_integral_ : " << heading_integral_ << endl;

        // Calculate the steering angle using Stanley Method
        float PID_steer = proportionalTerm + derivativeTerm + integralTerm;

        double headingErrorTermF = headingError * Lf/(Lf+Lr); //4ws
        double headingErrorTermR = -headingError * Lr/(Lf+Lr); //4ws

        // cout << headingError << endl;
        // cout << headingErrorTermR << endl;
        double steeringAngleF = headingErrorTermF + atan2(PID_steer, (0.1 + this->speed_)); //4ws
        double steeringAngleR = headingErrorTermR + atan2(PID_steer, (0.1 + this->speed_)); //4ws
        PointFR steeringAngle;

        steeringAngle.F = nomalize_angle(steeringAngleF);
        steeringAngle.R = nomalize_angle(steeringAngleR);

        // PID_steer = clip(PID_steer, -5.0F, 5.0F);
        
        
        l_n_a.linear = atan2(PID_steer, (0.1 + this->speed_));
        l_n_a.angular = heading_term + heading_integral_term;
        

        return l_n_a;
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


    double width;
    double L;
    linear_angular combined_steer;
    float preview_dt = 0.0;
    int preview_num = 0;
    float preview_h_e_gain = 1.0;
    vector<float> preview_gain;
    PointFR stanley_steer;
    vector<linear_angular> steers;
    Stanley preview_instance;

public:
    CombinedSteer(CallbackClass *cb_data)
        : FeedForward(), Stanley()
          
    {

        this->cb_data_ = cb_data;
        L = 0.494;
        width = 0.364;
        
    }

    void set_preview_param(float dt, vector<float> gain)
    {
        this->preview_dt = dt;
        this->preview_num = gain.size();
        this->preview_gain = gain;
    }

    void set_preview_heading_error_gain(float h_e_g)
    {
        this->preview_h_e_gain = h_e_g;
    }

    linear_angular calc_combined_steer()
    {
        
        if (cb_data_->get_odom_sub_flag())
        {
            // stanley preview
            linear_angular preview_steer = this->calc_stanly_preview();
            // stanley
            linear_angular now_stanley_steer = calc_stanley_steer();

            float sum_preview_gain = 0.0;
            for (size_t i = 0; i < preview_gain.size(); i++)
            {
                sum_preview_gain += preview_gain[i];
            }

            // sum both steer
            
            this->combined_steer.linear = (1-sum_preview_gain) * now_stanley_steer.linear + sum_preview_gain * preview_steer.linear;
            this->combined_steer.angular = (1-sum_preview_gain) * now_stanley_steer.angular + sum_preview_gain * preview_steer.angular;
            
            cb_data_->set_down_odom_sub_flag();
        }

        return this->combined_steer;
    }

    linear_angular calc_stanly_preview()
    {
        this->steers.clear();

        linear_angular sum_preview_steer;
        sum_preview_steer.angular = 0.0;
        sum_preview_steer.linear = 0.0;
        // sync stanly n preview p gain
        preview_instance.set_stanly_gain(get_stanley_P_gain(), 0.0, 0.0);
        preview_instance.set_heading_gain(this->preview_h_e_gain);
        // preview_instance.set_heading_gain(1.0);

        cb_data_->calc_predict_odometry_for_stanley(preview_dt, preview_num);

        vector<Point> predict_pos = cb_data_->get_stanley_predict_pos();
        vector<float> predict_yaw = cb_data_->get_stanley_predict_yaw();

        for (int i = 0; i < preview_num; i++){
            // calc_n_get_lat_error를 호출하면 closest point를 찾고 그 인덱스는
            // closest_index에 저장된다.
            double lat_error = cb_data_->calc_n_get_lat_error(predict_pos[i]);
            float tmp_speed = get_speed_in_stanley();
            double target_yaw = cb_data_->get_pd_path_yaw();

            preview_instance.set_stanly_data(tmp_speed, target_yaw, predict_yaw[i], lat_error);
            linear_angular steer = preview_instance.calc_stanley_steer();
            

            sum_preview_steer.linear += steer.linear * this->preview_gain[i];
            sum_preview_steer.angular += steer.angular * this->preview_gain[i];

            steers.push_back(steer);
        }

        return sum_preview_steer;
    }


     

    vector<linear_angular> get_preview_steers()
    {
        return this->steers;
    }


};


#endif // LAT_CONTROLLERS_HPP
