#ifndef DATA_MANAGE_HPP
#define DATA_MANAGE_HPP

#include <vector>
#include <iostream>
#include <random>
#include <numeric>
#include <limits>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "control/PIDController.hpp"
#include "control/kalman_filter.hpp"

#include "ranger_msgs/msg/actuator_state_array.hpp"
#include "ranger_msgs/msg/actuator_state.hpp"
#include "ranger_msgs/msg/motor_state.hpp"
using namespace std;

class CallbackClass
{
private:
    KalmanFilter2D *km_filter_;
    
    Point lo_odom;
    Point lo_odom_old;

    Point vel_rel;
    Point vel_abs;
    Point vel_rel_km;
    Point vel_rel_math;

    bool odom_sub_flag;
    bool odom_sub_flag_fix;
    bool odom_filter_flag;
    double lo_yaw;
    double lo_yaw_rate;
    bool control_sw;
    // ERP Data
    double vx_gt;
    double vy_gt;

    double vx_est_ki;
    double vy_est_ki;

    double vx_est_ki_pro;
    double vy_est_ki_pro;

    float speed;

    float wheel_speed_FL;
    float wheel_speed_FR;
    float wheel_speed_RL;
    float wheel_speed_RR;

    float wheel_brake_FL;
    float wheel_brake_FR;
    float wheel_brake_RL;
    float wheel_brake_RR;

    float wheel_steer_FL;
    float wheel_steer_FR;
    float wheel_steer_RL;
    float wheel_steer_RR;

    bool serial_sub_flag;

    int rate_control;
    vector<Point> pl_local_path;     // relative coordinate
    vector<Point> pl_local_path_abs; // abs coordinate
    vector<double> pl_local_path_yaws;
    int pl_control_switch;
    float pl_path_yaw;
    double predict_dist;

    int closest_index;
    double path_curvature = 0.0;
    double pre_path_curvature = 0.0;
    

    double wheel_radius = 0.105;
    double L = 0.494;
    double width = 0.364;
    double car_width = 0.364;

    vector<Point> stanley_predict_pos;
    vector<float> stanley_predict_yaw;
    Point zero_point;

    Point AxisTrans_abs2rel(const Point &p)
    {
        Point rel;
        rel.x = (p.x - lo_odom.x) * cos(lo_yaw) + (p.y - lo_odom.y) * sin(lo_yaw);
        rel.y = -(p.x - lo_odom.x) * sin(lo_yaw) + (p.y - lo_odom.y) * cos(lo_yaw);
        return rel;
    }

public:
    CallbackClass(KalmanFilter2D *km_filter)
    {
        // TODO: 생성자 작성 필요
        this->km_filter_ = km_filter;

        lo_odom.x = 0.0;
        lo_odom.y = 0.0;
        
        lo_odom_old.x = 0.0;
        lo_odom_old.y = 0.0;
        vel_rel.x = 0.0;
        vel_rel.y = 0.0;

        vel_abs.x = 0.0;
        vel_abs.y = 0.0;

        vel_rel_km.x = 0.0;
        vel_rel_km.y = 0.0;

        vel_rel_math.x = 0.0;
        vel_rel_math.y = 0.0;

        vx_gt = 0.0;
        vy_gt = 0.0;

        vx_est_ki = 0.0;
        vy_est_ki = 0.0;

        vx_est_ki_pro = 0.0;
        vy_est_ki_pro = 0.0;

        odom_sub_flag = false;
        odom_sub_flag_fix = false;
        odom_filter_flag = false;
        control_sw =  false;
        speed = 0.0;
        rate_control = 0;

        wheel_speed_FL = 0.0;
        wheel_speed_FR = 0.0;
        wheel_speed_RL = 0.0;
        wheel_speed_RR = 0.0;

        wheel_brake_FL = 0.0;
        wheel_brake_FR = 0.0;
        wheel_brake_RL = 0.0;
        wheel_brake_RR = 0.0;

        wheel_steer_FL = 0.0;
        wheel_steer_FR = 0.0;
        wheel_steer_RL = 0.0;
        wheel_steer_RR = 0.0;
        
        serial_sub_flag = false;

        closest_index = 0;
        zero_point.x = 0.0;
        zero_point.y = 0.0;

        pl_control_switch = 0;
    }

    Point AxisTrans_rel2abs(const Point &p)
    {
        Point abs;
        abs.x = p.x * cos(lo_yaw) - p.y * sin(lo_yaw) + lo_odom.x;
        abs.y = p.x * sin(lo_yaw) + p.y * cos(lo_yaw) + lo_odom.y;
        return abs;
    }

    Point AxisTrans_rel2abs_vel(const Point &p)
    {
        Point abs;
        abs.x = p.x * cos(lo_yaw) - p.y * sin(lo_yaw);
        abs.y = p.x * sin(lo_yaw) + p.y * cos(lo_yaw);
        return abs;
    }

    Point AxisTrans_abs2rel_vel(const Point &p)
    {
        Point rel;
        rel.x = p.x * cos(lo_yaw) + p.y * sin(lo_yaw);
        rel.y = -p.x * sin(lo_yaw) + p.y * cos(lo_yaw);
        return rel;
    }

    double calculateVariance(const std::vector<double>& group) {
        double sum = 0.0;
        double mean = 0.0;
        double variance = 0.0;

        // 평균 계산
        for (double num : group) {
            sum += num;
        }
        mean = sum / group.size();

        // 분산 계산
        for (double num : group) {
            variance += pow(num - mean, 2);
        }
        variance /= group.size();

        return variance;
    }

    

    void ranger_data_cb(const ranger_msgs::msg::ActuatorStateArray::SharedPtr msg)
    {
        
        if (!msg->states.empty())
        {   
            const auto& state1 = msg->states[0];
            const auto& state2 = msg->states[1];
            const auto& state3 = msg->states[2];
            const auto& state4 = msg->states[3];
            
            const auto& state5 = msg->states[4];
            const auto& state6 = msg->states[5];
            const auto& state7 = msg->states[6];
            const auto& state8 = msg->states[7];

            this->wheel_steer_FL = state1.motor.driver_state; 
            this->wheel_steer_FR = state2.motor.driver_state; 
            this->wheel_steer_RL = state3.motor.driver_state; 
            this->wheel_steer_RR = state4.motor.driver_state; 

            this->wheel_speed_FL = state5.motor.driver_state * this->wheel_radius; 
            this->wheel_speed_FR = state6.motor.driver_state * this->wheel_radius; 
            this->wheel_speed_RL = state7.motor.driver_state * this->wheel_radius; 
            this->wheel_speed_RR = state8.motor.driver_state * this->wheel_radius;  

            this->serial_sub_flag = true;


            

            // 속도 추정
            this->rate_control = this->rate_control + 1;
            if(this->rate_control == 15){ //
                
                velocity_estimation();

                if(!this->odom_filter_flag){
                    km_filter_->predictUpdate1(this->vel_abs.x, this->vel_abs.y);
                }
                else{
                    km_filter_->predictUpdate2(this->lo_odom.x, this->lo_odom.y, this->vel_abs.x, this->vel_abs.y);
                    // km_filter_->predictUpdate1(this->vel_abs.x, this->vel_abs.y);

                    double abs_x_speed = (this->lo_odom.x - this->lo_odom_old.x) / 0.125;
                    double abs_y_speed = (this->lo_odom.y - this->lo_odom_old.y) / 0.125;
                    this->lo_odom_old.x = this->lo_odom.x;
                    this->lo_odom_old.y = this->lo_odom.y;

                    Point math_abs_vel_tmp;
                    math_abs_vel_tmp.x = abs_x_speed;
                    math_abs_vel_tmp.y = abs_y_speed;

                    this->vel_rel_math = AxisTrans_abs2rel_vel(math_abs_vel_tmp);
                }

                
                Point km_abs_vel_tmp;
                km_abs_vel_tmp.x = km_filter_->get_kf_vx();
                km_abs_vel_tmp.y = km_filter_->get_kf_vy();

                

                vel_rel_km = AxisTrans_abs2rel_vel(km_abs_vel_tmp);
                

                this->odom_filter_flag = false;
                this->rate_control = 0;
            }
            
        }
 
    }
  

    void lo_odom_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // info : [x, y], UTM coordinate

    
        // cout << "lo_odom.x !!!: " << msg->data[0] << endl;
        // cout << "lo_odom.y !!!: " << msg->data[1] << endl;
        // cout << "lo_odom size !!!: " << msg->data.size() << endl;
        
        this->lo_odom.x = msg->data[0];
        this->lo_odom.y = msg->data[1];

        if (!odom_sub_flag_fix){
            km_filter_->set_x_space(this->lo_odom.x, this->lo_odom.y);
        }

        this->odom_sub_flag_fix = true;
        this->odom_filter_flag = true;
        this-> odom_sub_flag = true;
    }
    

    void lo_yaw_cb(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // info : radian, -pi ~ +pi
        this->lo_yaw = msg->data;
    }

    void lo_imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract yaw rate (z-axis angular velocity)
        this->lo_yaw_rate = msg->angular_velocity.z;
    }

    void pl_control_sw_cb(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // Extract yaw rate (z-axis angular velocity)
        this->control_sw = msg->data;
    }
 
    
    void pl_local_path_cb(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty())
        {
            for (int i = 0; i < 20; i++)
            {
                cout << "경로 xxxxxxxxxx: " << msg->poses.size() << endl;
            }

            // cerr << "Error: The local path sub yet!!!!." << endl;
            // return 404; // Return 404 to indicate an error.
            return;
        }

        vector<Point> points;
        vector<Point> points_abs;

        // Convert the received message to a vector of points
        for (const auto &pose : msg->poses)
        {
            // Assuming that the pose message has position field
            geometry_msgs::msg::Point point = pose.pose.position;
            Point point_obj{point.x, point.y};

            points_abs.push_back(point_obj);

            Point pl_rel = AxisTrans_abs2rel(point_obj);
            points.push_back(pl_rel);

            
        }

        this->pl_local_path_abs = points_abs;
        this->pl_local_path = points;
        
        path_yaws_calc();
    }



    void path_yaws_calc()
    {
        std::vector<double> yaws;
        double min_distance = 0.01;
        bool yawInitialiuzed = false;
        double yaw_;
        double yaw;
        for (size_t i = 0; i < this->pl_local_path_abs.size() - 1; ++i)
        {
            double dx = this->pl_local_path_abs[i + 1].x - this->pl_local_path_abs[i].x;
            double dy = this->pl_local_path_abs[i + 1].y - this->pl_local_path_abs[i].y;

            if ((dx < min_distance || dy < min_distance) && this->pl_local_path_abs.size() - 6 > i)
            {
                for (size_t k = 0; k < 5; ++k)
                {
                    dx = this->pl_local_path_abs[i + k + 2].x - this->pl_local_path_abs[i].x;
                    dy = this->pl_local_path_abs[i + k + 2].y - this->pl_local_path_abs[i].y;

                    if (dx > min_distance && dy > min_distance)
                    {
                        break;
                    }
                }
            }

            yaw = std::atan2(dy, dx);

            if (!yawInitialiuzed)
            {
                yaw_ = yaw;
                yawInitialiuzed = true;
            }

            yaw = yaw * 0.95 + yaw_ * 0.05;

            yaw_ = yaw;

            yaws.push_back(yaw);
        }

        yaws.push_back(yaws.back());
        
        this->pl_local_path_yaws = yaws;
    }

    // void pl_path_yaw_cb(const std_msgs::msg::Float64::SharedPtr msg)
    // {
    //     // info : 0:STOP, 1:LOW SPEED, 2:NORMAL SPEED
    //     this->pl_path_yaw = msg->data;
    // }

    void pl_control_switch_cb(const std_msgs::msg::Int16::SharedPtr msg)
    {
        // info : 0:STOP, 1:LOW SPEED, 2:NORMAL SPEED
        this->pl_control_switch = msg->data;
    }

    void ranger_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        this->vx_gt = msg->linear.x;
        this->vy_gt = msg->linear.y;
    }

    

    double calc_n_get_lat_error()
    {
        // 함수 오버로딩을 이용해 일반적인 상황과 다른 점에서의 Lat_error를 구하기 위한 상황을 구분한다.
        // 인자를 주지 않으면 이 함수가 호출
        // 인자를 주면 아래의 함수를 호출
        return calc_n_get_lat_error(this->zero_point);
    }

    double calc_n_get_lat_error(Point predict_pose)
    {
        if (pl_local_path.empty())
        {
            cout << "pl_local_path size !!!: " << pl_local_path.size() << endl;
            cerr << "Error: The local path sub yet!!!!." << endl;
            return 404; // Return 404 to indicate an error.
        }
        else if (lo_odom.x == 0.0 || lo_odom.y == 0.0)
        {
            cerr << "Error: The odom UTM sub yet!!!!." << endl;
            return 404; // Return 404 to indicate an error. 
        }

        closest_index = 0; // 계산 결과로 나오는 값인데 이 값도 사용된다.
        double min_distance = std::sqrt(std::pow(pl_local_path[0].x - predict_pose.x, 2) + std::pow(pl_local_path[0].y - predict_pose.y, 2));

        for (size_t i = 1; i < pl_local_path.size(); i++)
        {
            double distance = std::sqrt(std::pow(pl_local_path[i].x - predict_pose.x, 2) + std::pow(pl_local_path[i].y - predict_pose.y, 2));
            if (distance < min_distance)
            {
                closest_index = static_cast<int>(i);
                min_distance = distance;
            }
        }

        // lat_error의 좌 우를 구분함.
        double l_or_r = determine_side(pl_local_path[closest_index + 1], predict_pose, pl_local_path[closest_index]);
        if (l_or_r > 0)
        {
            min_distance = min_distance * -1;
        }

        if (min_distance > 50)
        {
            cerr << "Error: lateral error is too large!!!!." << endl;
            cerr << " 경로와의 최소 거리 : " << min_distance << endl;
            return 404; // Return 404 to indicate an error.
        }
        return min_distance;
    }

    double calc_path_curvature(float time_delay = 0.0, float diff_s = 1.5)
    {
        float td = clip(time_delay, 0.0F, 2.0F);
        predict_dist = td * clip(this->speed, 1.0F, 9.0F) - (diff_s / 2.0); // (s)*(m/s)
        double second_predict_dist = predict_dist + diff_s;
        int f_pri_yaw_index = 0;
        int s_pri_yaw_index = 0;

        double s = 0.0;        // 경로의 길이 (Frenet s)
        double min_dist = 1e9; // 아주 큰 값으로 초기화
        double second_min_dist = 1e9;

        for (size_t i = 1; i < pl_local_path.size(); i++)
        {
            double dx = pl_local_path[i].x - pl_local_path[i - 1].x;
            double dy = pl_local_path[i].y - pl_local_path[i - 1].y;
            s += std::sqrt(dx * dx + dy * dy); // 경로의 길이 누적

            double f_dist = abs(s - predict_dist);
            double s_dist = abs(s - second_predict_dist);

            if (f_dist < min_dist)
            {
                f_pri_yaw_index = static_cast<int>(i);
                min_dist = f_dist;
            }
            if (s_dist < second_min_dist)
            {
                s_pri_yaw_index = static_cast<int>(i);
                second_min_dist = s_dist;
            }
        }

        // TODO: 여기 path_yaw의 인덱스와 local_path의 인덱스가 다를 경우 값이 일치하지 않을 수 있음.
        double yaw_diff = abs(nomalize_angle(pl_local_path_yaws[f_pri_yaw_index] - pl_local_path_yaws[s_pri_yaw_index]));

        double tmp_curv = yaw_diff / diff_s;
        this->path_curvature = low_pass_filter(tmp_curv, pre_path_curvature, 1.0);
        this->pre_path_curvature = this->path_curvature;

        return path_curvature;
    }

    double calc_path_curvature_center(float diff_s = 1.5)
    {
        if (pl_local_path.empty())
        {
            cout << "pl_local_path size !!!: " << pl_local_path.size() << endl;
            cerr << "Error: The local path sub yet!!!!." << endl;
            return 404; // Return 404 to indicate an error.
        }

        predict_dist = - (diff_s / 2.0); // (s)*(m/s)
        double second_predict_dist = predict_dist + diff_s;
        int f_pri_yaw_index = 0;
        int s_pri_yaw_index = 0;

        double s_f = 0.0;   
        double s_s = 0.0;         // 경로의 길이 (Frenet s)
        double min_dist = 1e9; // 아주 큰 값으로 초기화
        double second_min_dist = 1e9;



        for (int i = this->closest_index; i >= 0; --i){

            double dx = pl_local_path[i].x - pl_local_path[i - 1].x;
            double dy = pl_local_path[i].y - pl_local_path[i - 1].y;

            s_f += std::sqrt(dx * dx + dy * dy); // 경로의 길이 누적

            double f_dist = abs(s_f - abs(predict_dist));

            if (f_dist < min_dist)
            {
                f_pri_yaw_index = static_cast<int>(i);
                min_dist = f_dist;
            }
        }


        for (int i = this->closest_index + 1; i < pl_local_path.size(); i++)
        {
            double dx = pl_local_path[i].x - pl_local_path[i - 1].x;
            double dy = pl_local_path[i].y - pl_local_path[i - 1].y;
            s_s += std::sqrt(dx * dx + dy * dy); // 경로의 길이 누적

            double s_dist = abs(s_s - second_predict_dist);

            if (s_dist < second_min_dist)
            {
                s_pri_yaw_index = static_cast<int>(i);
                second_min_dist = s_dist;
            }
        }

        // TODO: 여기 path_yaw의 인덱스와 local_path의 인덱스가 다를 경우 값이 일치하지 않을 수 있음.
        double yaw_diff = abs(nomalize_angle(pl_local_path_yaws[f_pri_yaw_index] - pl_local_path_yaws[s_pri_yaw_index]));

        double tmp_curv = yaw_diff / diff_s;
        this->path_curvature = low_pass_filter(tmp_curv, pre_path_curvature, 1.0);
        this->pre_path_curvature = this->path_curvature;

        return path_curvature;
    }

    void velocity_estimation()
    {

        std::random_device rd;
        std::mt19937 gen(rd()); // 메르센 트위스터 엔진 사용
        std::normal_distribution<double> dist(0.0, 0.5); // 평균 0, 표준 편차 1


        // double vel_est_fl = (wheel_speed_FL)  + cos(wheel_steer_FL) * this->lo_yaw_rate * this->width / 2.0 \
        //                 - sin(wheel_steer_FL) * this->lo_yaw_rate * this->L / 2.0;
        // double vel_est_fr = (wheel_speed_FR) - cos(wheel_steer_FR) * this->lo_yaw_rate * this->width / 2.0 \
        //                 - sin(wheel_steer_FR) * this->lo_yaw_rate * this->L / 2.0;
        // double vel_est_rl = (wheel_speed_RL) + cos(wheel_steer_RL) * this->lo_yaw_rate * this->width / 2.0 \
        //                 + sin(wheel_steer_RL) * this->lo_yaw_rate * this->L / 2.0;
        // double vel_est_rr = (wheel_speed_RR) - cos(wheel_steer_RR) * this->lo_yaw_rate * this->width / 2.0 \
        //                 + sin(wheel_steer_RR) * this->lo_yaw_rate * this->L / 2.0;

        double vel_est_fl = (wheel_speed_FL + abs(dist(gen))/3.0)  + cos(wheel_steer_FL) * this->lo_yaw_rate * this->width / 2.0 \
                        - sin(wheel_steer_FL) * this->lo_yaw_rate * this->L / 2.0;
        double vel_est_fr = (wheel_speed_FR + abs(dist(gen))/3.0) - cos(wheel_steer_FR) * this->lo_yaw_rate * this->width / 2.0 \
                        - sin(wheel_steer_FR) * this->lo_yaw_rate * this->L / 2.0;
        double vel_est_rl = (wheel_speed_RL + abs(dist(gen))/3.0) + cos(wheel_steer_RL) * this->lo_yaw_rate * this->width / 2.0 \
                        + sin(wheel_steer_RL) * this->lo_yaw_rate * this->L / 2.0;
        double vel_est_rr = (wheel_speed_RR + abs(dist(gen))/3.0) - cos(wheel_steer_RR) * this->lo_yaw_rate * this->width / 2.0 \
                        + sin(wheel_steer_RR) * this->lo_yaw_rate * this->L / 2.0;


        std::vector<double> vx_values = {cos(wheel_steer_FL) * vel_est_fl, cos(wheel_steer_FR) * vel_est_fr, cos(wheel_steer_RL) * vel_est_rl, cos(wheel_steer_RR) * vel_est_rr};
        std::vector<double> vy_values = {sin(wheel_steer_FL) * vel_est_fl, sin(wheel_steer_FR) * vel_est_fr, sin(wheel_steer_RL) * vel_est_rl, sin(wheel_steer_RR) * vel_est_rr};
        
        double minVariance = std::numeric_limits<double>::max(); 
        std::vector<double> minVarianceGroup; 
        std::vector<int> minVarianceGroupInd; 

        for (size_t i = 0; i < vx_values.size(); ++i) {
            for (size_t j = i + 1; j < vx_values.size(); ++j) {
                for (size_t k = j + 1; k < vx_values.size(); ++k) {
                    std::vector<double> group = {vx_values[i], vx_values[j], vx_values[k]};
                    double variance = calculateVariance(group);
                    
                    // 현재 분산이 최소 분산보다 작으면 갱신
                    if (variance < minVariance) {
                        minVariance = variance;
                        minVarianceGroup = group;
                        minVarianceGroupInd = {i, j, k};
                    }
                }
            }
        }

        double vx_sum = 0.0;
        
        double vy_sum = 0.0;

        for (double tmp_value : minVarianceGroup) {
            vx_sum += tmp_value;
        }

        for (int ind : minVarianceGroupInd) {
            vy_sum += vy_values[minVarianceGroupInd[ind]];
        }


        this->vx_est_ki_pro = vx_sum / minVarianceGroup.size();
        this->vy_est_ki_pro = vy_sum / minVarianceGroup.size();


        this->vx_est_ki = (cos(wheel_steer_FL) * vel_est_fl + cos(wheel_steer_FR) * vel_est_fr + cos(wheel_steer_RL) * vel_est_rl + cos(wheel_steer_RR) * vel_est_rr) / 4.0;
        this->vy_est_ki = (sin(wheel_steer_FL) * vel_est_fl + sin(wheel_steer_FR) * vel_est_fr + sin(wheel_steer_RL) * vel_est_rl + sin(wheel_steer_RR) * vel_est_rr) / 4.0;

        this->vel_rel.x = this->vx_est_ki;
        this->vel_rel.y = this->vy_est_ki;

        vel_abs = AxisTrans_rel2abs_vel(this->vel_rel);
    }
    Point get_odom()
    {
        return this->lo_odom;
    }

    void calc_predict_odometry_for_stanley(float dt, int n)
    {
        vector<Point> predict_pos;

        double dx = this->vx_est_ki * dt;
        double dy = this->vy_est_ki * dt;

        for (int i = 1; i <= n; i++)
        {
            Point tmp_odom;

            tmp_odom.x = dx * i;
            tmp_odom.y = dy * i;
            predict_pos.push_back(tmp_odom);
        }

        vector<float> predict_yaw;

        for (int i = 1; i <= n; i++)
        {
            float tmp_yaw = this->lo_yaw + this->lo_yaw_rate * (dt);
            predict_yaw.push_back(tmp_yaw);
        }

        this->stanley_predict_pos = predict_pos;
        this->stanley_predict_yaw = predict_yaw;
    }

    bool get_control_sw()
    {
        return this->control_sw;
    }

    bool get_odom_sub_flag()
    {
        return this->odom_sub_flag;
    }

    void set_down_odom_sub_flag()
    {
        this->odom_sub_flag = false;
    }

    double get_yaw()
    {
        return this->lo_yaw;
    }

    float get_wheel_speed_FL()
    {
        return this->wheel_speed_FL;
    }   

    float get_wheel_speed_FR()
    {
        return this->wheel_speed_FR;
    }   

    float get_wheel_speed_RL()
    {
        return this->wheel_speed_RL;
    }   

    float get_wheel_speed_RR()
    {
        return this->wheel_speed_RR;
    }   

    float get_wheel_steer_FL()
    {
        return this->wheel_steer_FL;
    }   

    float get_wheel_steer_FR()
    {
        return this->wheel_steer_FR;
    } 

    float get_wheel_steer_RL()
    {
        return this->wheel_steer_RL;
    } 

    float get_wheel_steer_RR()
    {
        return this->wheel_steer_RR;
    } 


    float get_speed()
    {
        return this->speed;
    }   

    double get_vx_gt()
    {
        return this->vx_gt;
    }   

    double get_vy_gt()
    {
        return this->vy_gt;
    }   

    double get_vx_est_ki()
    {
        return this->vx_est_ki;
    }
    double get_vy_est_ki()
    {
        return this->vy_est_ki;
    }

    double get_vx_km()
    {
        return this->vel_rel_km.x;
    }

    double get_vy_km()
    {
        return this->vel_rel_km.y;
    }

    double get_vx_math()
    {
        return this->vel_rel_math.x;
    }

    double get_vy_math()
    {
        return this->vel_rel_math.y;
    }

    double get_vx_est_ki_pro()
    {
        return this->vx_est_ki_pro;
    }

    double get_vy_est_ki_pro()
    {
        return this->vy_est_ki_pro;
    }



    // float get_steer()
    // {
    //     return this->steer;
    // }


    int get_control_switch()
    {
        return this->pl_control_switch;
    }

    float get_path_yaw()
    {
        return this->pl_path_yaw;
    }

    double get_pd_path_yaw()
    {
        if (this->pl_local_path_yaws.empty())
        {
            cout << "pl_local_path_yaw is empty!!!" << endl;
            return 404;
        }
        return this->pl_local_path_yaws[this->closest_index];
    }
    vector<double> get_local_path_yaw()
    {
        return this->pl_local_path_yaws;
    }

    float get_yawrate()
    {
        return this->lo_yaw_rate;
    }

    double get_path_curvature()
    {
        return this->path_curvature;
    }

    bool get_serial_sub_flag()
    {
        return this->serial_sub_flag;
    }

    void set_down_serial_sub_flag()
    {
        this->serial_sub_flag = false;
    }

    vector<Point> get_relative_path()
    {
        return this->pl_local_path;
    }

    vector<Point> get_stanley_predict_pos()
    {
        return this->stanley_predict_pos;
    }

    vector<float> get_stanley_predict_yaw()
    {
        return this->stanley_predict_yaw;
    }

};

#endif // DATA_MANAGE_HPP