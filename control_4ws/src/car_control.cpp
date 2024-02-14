#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/path.hpp>
#include <chrono>

#include "control/callback_data_manage.hpp"
#include "control/PIDController.hpp"
#include "control/lat_control.hpp"
#include "control/lon_control.hpp"
#include "control/mission_param.hpp"

#include "ranger_msgs/msg/actuator_state_array.hpp"
#include "ranger_msgs/msg/actuator_state.hpp"
#include "ranger_msgs/msg/motor_state.hpp"

using namespace std;

bool IS_PRINT = true;

class CarControl : public rclcpp::Node
{
private:
    std::shared_ptr<CallbackClass> callback_data_ptr;
    CallbackClass *cb_data;

    std::shared_ptr<CombinedSteer> lat_control_ptr;
    CombinedSteer *lat_control;

    std::shared_ptr<LonController> lon_control_ptr;
    LonController *lon_control;

    std::shared_ptr<ControlGainTuning> param_manage_ptr;
    ControlGainTuning *param_manage;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr lo_odom_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lo_curr_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr lo_imu__sub;
    
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pl_local_sub;
    // rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pl_local_sub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr pl_cont_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pl_pyaw_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pl_pyaws_sub;

    rclcpp::Subscription<ranger_msgs::msg::ActuatorStateArray>::SharedPtr ranger_data_sub;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ranger_data_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tmp_data_pub;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_execution_time_ = now();
public:
    CarControl()
        : Node("car_control")
        // <ROS 노드 선언>---------------------------------------------------------
    // ---------------------------------------------------------</ROS 노드 선언>
    {
        

        // <클레스들 인스턴스화>------------------------------------------------------
        callback_data_ptr = std::make_shared<CallbackClass>();
        cb_data = callback_data_ptr.get();

        lat_control_ptr = std::make_shared<CombinedSteer>(cb_data);
        lat_control = lat_control_ptr.get();
    
        lon_control_ptr = std::make_shared<LonController>();
        lon_control = lon_control_ptr.get();

        param_manage_ptr = std::make_shared<ControlGainTuning>(cb_data, lat_control, lon_control);
        param_manage = param_manage_ptr.get();

        // ------------------------------------------------------</클레스들 인스턴스화>
        // <SUBSCRIBER> -----------------------------------------------------------
        
        // Local
        lo_odom_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/Local/utm", 1, std::bind(&CallbackClass::lo_odom_cb, cb_data, std::placeholders::_1));
        lo_curr_sub = this->create_subscription<std_msgs::msg::Float64>("/Local/heading", 1, std::bind(&CallbackClass::lo_yaw_cb, cb_data, std::placeholders::_1));
        lo_imu__sub = this->create_subscription<sensor_msgs::msg::Imu>("/Local/imu_hpc/out", 1, std::bind(&CallbackClass::lo_imu_cb, cb_data, std::placeholders::_1));

        // Planning
        pl_local_sub = this->create_subscription<nav_msgs::msg::Path>("/Planning/local_path", 1, std::bind(&CallbackClass::pl_local_path_cb, cb_data, std::placeholders::_1));
        
        ranger_data_sub = this->create_subscription<ranger_msgs::msg::ActuatorStateArray>("/ranger_states", 1, std::bind(&CallbackClass::ranger_data_cb, cb_data, std::placeholders::_1));
  
        // -----------------------------------------------------------</SUBSCRIBER>
        // <PUBLISHER> ------------------------------------------------------------

        ranger_data_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/Control/ranger_data", 1);
        tmp_data_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Control/tmp_plot_val", 1);
        
        // ------------------------------------------------------------</PUBLISHER>
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                         std::bind(&CarControl::timer_callback, this));
        
        
    }

    void timer_callback()
    {

        auto start_time = std::chrono::steady_clock::now();
        rclcpp::Time current_time = now();
        
        auto time_diff = current_time - last_execution_time_;
        last_execution_time_ = now();
        double time_diff_ms = time_diff.seconds() * 1000.0;
        last_execution_time_ = current_time;

        double wheel_radius = 0.1;
        double L = 0.494;
        double width = 0.364;

        float curr_speed = cb_data->get_speed();
        float wheel_speed_FL = cb_data->get_wheel_speed_FL() * wheel_radius; // m/s 
        float wheel_speed_FR = cb_data->get_wheel_speed_FR() * wheel_radius; // m/s 
        float wheel_speed_RL = cb_data->get_wheel_speed_RL() * wheel_radius; // m/s 
        float wheel_speed_RR = cb_data->get_wheel_speed_RR() * wheel_radius; // m/s 


        float yaw = cb_data->get_yaw();
        Point odom = cb_data->get_odom();
        double lat_error = cb_data->calc_n_get_lat_error();
        double yaw_rate = cb_data->get_yawrate();
        
        double path_curature = cb_data->calc_path_curvature_center(1.0);
        double pd_path_yaw = cb_data->get_pd_path_yaw();


        vector<Point> rel_local_path = cb_data->get_relative_path();

        // lat_error가 404면 뭔가 잘못됨.
        if (lat_error == 404 || pd_path_yaw == 404)
        {
            cout << "404!!!" << endl;
            // return;
        }

        // <계산> -----------------------------------------------------------
        param_manage->set_normal_param();

        // Control Switch
        int cs = cb_data->get_control_switch();
        GasAndBrake acc_val_FL;
        GasAndBrake acc_val_FR;
        GasAndBrake acc_val_RL;
        GasAndBrake acc_val_RR;

        PointFR steerAngle;

        double target_speed = lon_control->get_target_speed() / 3.6;
        double max_target_yr = target_speed / (L / 2.0);
        
        //Target Speed wheel
        double target_speed_FR;
        double target_speed_FL;
        double target_speed_RR;
        double target_speed_RL;

        // 횡방향 계산

        // 적분 초기화
        // if (abs(lat_error)<0.05)
        // {
        // double tmp_integ = lat_control->get_stanley_integral_val();
        // lat_control->set_stanley_integral_val(tmp_integ/2.0);
        // }

        cout << "integral : " << lat_control->get_stanley_integral_val() << endl;

        lat_control->set_stanly_data(curr_speed, pd_path_yaw, yaw, lat_error);
        lat_control->set_curvature(path_curature);

        steerAngle = lat_control->calc_combined_steer();

        switch (cs)
        {

        case -1: // 정지 모드
            acc_val_FL.gas = 0.0;
            acc_val_FL.brake = 180;

            acc_val_FR.gas = 0.0;
            acc_val_FR.brake = 180;

            acc_val_RL.gas = 0.0;
            acc_val_RL.brake = 180;

            acc_val_RR.gas = 0.0;
            acc_val_RR.brake = 180;
            break;

        case 0: // normal 모드
            
            int yr_sign = sign_determinater(steerAngle.F, steerAngle.R);
            double yr_norm = sqrt(pow((cos(steerAngle.F) - cos(steerAngle.R)) / 2, 2) + pow((sin(steerAngle.F) - sin(steerAngle.R)) / 2, 2));
            double target_yr = max_target_yr * yr_norm * yr_sign;

            double target_speed_x = target_speed * (cos(steerAngle.F) + cos(steerAngle.R)) / 2.0;
            double target_speed_y = target_speed * (sin(steerAngle.F) + sin(steerAngle.R)) / 2.0;


            target_speed_FL = target_speed_x * cos(steerAngle.FL) - target_yr * cos(steerAngle.FL) * width / 2 \
                            + target_yr * sin(steerAngle.FL) * L / 2 + target_speed_y * sin(steerAngle.FL);

            target_speed_FR = target_speed_x * cos(steerAngle.FR) + target_yr * cos(steerAngle.FR) * width / 2 \
                            + target_yr * sin(steerAngle.FR) * L / 2 + target_speed_y * sin(steerAngle.FR);

            target_speed_RL = target_speed_x * cos(steerAngle.RL) - target_yr * cos(steerAngle.RL) * width / 2 \
                            - target_yr * sin(steerAngle.RL) * L / 2 + target_speed_y * sin(steerAngle.RL);

            target_speed_RR = target_speed_x * cos(steerAngle.RR) + target_yr * cos(steerAngle.RR) * width / 2 \
                            + target_yr * sin(steerAngle.RR) * L / 2 + target_speed_y * sin(steerAngle.RR);

            
            lon_control->set_lon_target_speed(target_speed_FR);                                              
            lon_control->set_lon_data(wheel_speed_FR);
            acc_val_FR = lon_control->calc_gas_n_brake();


            lon_control->set_lon_target_speed(target_speed_FL);
            lon_control->set_lon_data(wheel_speed_FL);
            acc_val_FL = lon_control->calc_gas_n_brake();


            lon_control->set_lon_target_speed(target_speed_RR);
            lon_control->set_lon_data(wheel_speed_RR);
            acc_val_RR = lon_control->calc_gas_n_brake();

            lon_control->set_lon_target_speed(target_speed_RL);
            lon_control->set_lon_data(wheel_speed_RL);
            acc_val_RL = lon_control->calc_gas_n_brake();
   
            break;
        }

        // ---------------------------------------------------------- </계산>
        // <ros topic pub> -------------------------------------------------

        // 제어 값 pub

        auto car_data_msg = std::make_shared<std_msgs::msg::Float32MultiArray>();

        car_data_msg->data.push_back(acc_val_FL.gas / wheel_radius);
        car_data_msg->data.push_back(acc_val_FR.gas / wheel_radius);
        car_data_msg->data.push_back(acc_val_RL.gas / wheel_radius);
        car_data_msg->data.push_back(acc_val_RR.gas / wheel_radius);


        car_data_msg->data.push_back(steerAngle.FL);      
        car_data_msg->data.push_back(steerAngle.FR);   
        car_data_msg->data.push_back(steerAngle.RL);    
        car_data_msg->data.push_back(steerAngle.RR);   


        ranger_data_pub->publish(*car_data_msg);  

        // 디버그용 토픽
        auto tmp_plot_val_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();

        tmp_plot_val_msg->data.push_back(pd_path_yaw);  
        tmp_plot_val_msg->data.push_back(lat_error); 
        tmp_plot_val_msg->data.push_back(nomalize_angle(pd_path_yaw - yaw)); 
        tmp_plot_val_msg->data.push_back(path_curature); 
        tmp_plot_val_msg->data.push_back(lat_control->get_heading_term()); 

        auto end_time = std::chrono::steady_clock::now();
        auto duration = end_time - start_time;
        double seconds = std::chrono::duration_cast<std::chrono::duration<double>>(duration).count()* 1000.0;



        tmp_plot_val_msg->data.push_back(seconds); 
        tmp_plot_val_msg->data.push_back(time_diff_ms); 

        for (size_t i = 0; i < rel_local_path.size(); i += 5)
        {
            Point tmp_p = rel_local_path[i];

            tmp_plot_val_msg->data.push_back(tmp_p.x);  
            tmp_plot_val_msg->data.push_back(tmp_p.y);      
            
        }

        
        

        

        tmp_data_pub->publish(*tmp_plot_val_msg);
        
        // if (IS_PRINT)
        // {
        //     // cout << "==================" << endl;
        //     // cout << "changed_param_num : " << changed_param_num << endl;
        //     // cout << "gas : " << acc_val.gas << endl;
        //     // cout << "brake : " << acc_val.brake << endl;
        //     // cout << "steer : " << steer << endl
        //     //      << endl;

        //     // cout << "yawrate : " << yaw_rate << endl;
        //     // cout << "PP_steer : " << lat_control->get_PP_steer() * 180 / M_PI << " deg (" << lat_control->get_PP_steer() << " rad)" << endl;
        //     // cout << "stanly_steer : " << lat_control->get_stanly_steer() * 180 / M_PI << " deg (" << lat_control->get_stanly_steer() << " rad)" << endl;
        //     // cout << "curr_speed : " << curr_speed * 3.6 << "km/h (" << curr_speed << "m/s" << endl;
        //     // cout << "yaw : " << yaw << endl;
        //     // cout << "lat_error : " << lat_error << endl;
        //     // cout << "LD distance : " << lat_control->get_LA_distance() << endl;
        //     // cout << "target speed : " << lon_control->get_target_speed() * 3.6 << "km/h (" << lon_control->get_target_speed() << "m/s" << endl;
        //     // cout << "path_yaw_nearest(rad) : " << pd_path_yaw << endl;
        // }

        // ------------------------------------------------------- </print>
        return;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CarControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

