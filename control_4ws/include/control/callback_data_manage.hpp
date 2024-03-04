#ifndef DATA_MANAGE_HPP
#define DATA_MANAGE_HPP

#include <vector>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include "control/PIDController.hpp"

#include "ranger_msgs/msg/actuator_state_array.hpp"
#include "ranger_msgs/msg/actuator_state.hpp"
#include "ranger_msgs/msg/motor_state.hpp"
using namespace std;

class CallbackClass
{
private:
    Point lo_odom;
    bool odom_sub_flag;
    double lo_yaw;
    double lo_yaw_rate;
    // ERP Data
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
    vector<Point> pl_local_path;     // relative coordinate
    vector<Point> pl_local_path_abs; // abs coordinate
    vector<double> pl_local_path_yaws;
    int pl_control_switch;
    float pl_path_yaw;
    double predict_dist;

    int closest_index;
    double path_curvature = 0.0;
    double pre_path_curvature = 0.0;
    Point zero_point;

    double car_width = 0.364;

    Point AxisTrans_abs2rel(const Point &p)
    {
        Point rel;
        rel.x = (p.x - lo_odom.x) * cos(lo_yaw) + (p.y - lo_odom.y) * sin(lo_yaw);
        rel.y = -(p.x - lo_odom.x) * sin(lo_yaw) + (p.y - lo_odom.y) * cos(lo_yaw);
        return rel;
    }

public:
    CallbackClass()
    {
        // TODO: 생성자 작성 필요
        lo_odom.x = 0.0;
        lo_odom.y = 0.0;
        odom_sub_flag = false;
        
        speed = 0.0;

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
        Point rel;
        rel.x = p.x * cos(lo_yaw) - p.y * sin(lo_yaw) + lo_odom.x;
        rel.y = p.x * sin(lo_yaw) + p.y * cos(lo_yaw) + lo_odom.y;
        return rel;
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

            this->wheel_speed_FL = state5.motor.driver_state; 
            this->wheel_speed_FR = state6.motor.driver_state; 
            this->wheel_speed_RL = state7.motor.driver_state; 
            this->wheel_speed_RR = state8.motor.driver_state;  

            this->serial_sub_flag = true;

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

        this->odom_sub_flag = true;
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


    Point get_odom()
    {
        return this->lo_odom;
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

};

#endif // DATA_MANAGE_HPP