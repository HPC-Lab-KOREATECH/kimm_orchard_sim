// Edited from https://github.com/JokerJohn/LIO_SAM_6AXIS/blob/d026151c12588821de8b7dd240b3ca7012da007d/LIO-SAM-6AXIS/src/simpleGpsOdom.cpp
// Mark Jin Edited 20230523

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>


#include <deque>
#include <mutex>
#include <queue>
#include <iostream>

#include "robot_localization/srv/set_datum.hpp"  
#include "robot_localization/srv/set_pose.hpp"  
#include "robot_localization/srv/get_datum.hpp"  

#include "gpsTools.hpp"
#include <rclcpp/rclcpp.hpp>
#include "utility.hpp"


using namespace std;

class GNSSOdom : public ParamServer {
 public:
  GNSSOdom(const rclcpp::NodeOptions & options) : ParamServer("lio_sam_gps_converter", options) {
    sleep(3); // wait starting robot_localization

    if(isLocalizationMode){
      gpsSub = create_subscription<sensor_msgs::msg::NavSatFix>(gpsNaiveTopic, qos, std::bind(&GNSSOdom::GNSSCB, this, std::placeholders::_1));
      imuSub = create_subscription<sensor_msgs::msg::Imu>(imuTopic, 1, std::bind(&GNSSOdom::imuHandler, this, std::placeholders::_1));
      setDatumClient = this->create_client<robot_localization::srv::SetDatum>("datum"); 
      initialPosePub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
      readFile();
      isInit = true;
    }else{
      getDatumClient = this->create_client<robot_localization::srv::GetDatum>("getDatum"); 
    }
    
    gpsToOdomSub = create_subscription<sensor_msgs::msg::NavSatFix>(gpsToOdomSubTopic, qos, std::bind(&GNSSOdom::gpsToOdomCallback, this, std::placeholders::_1));
    gpsToOdomPub = create_publisher<geometry_msgs::msg::Pose>(gpsToOdomPubTopic, 10 /* QoS depth */);

    odomToGpsSub = create_subscription<geometry_msgs::msg::Pose>(odomToGpsSubTopic, qos, std::bind(&GNSSOdom::odomToGpsCallback, this, std::placeholders::_1));
    odomToGpsPub = create_publisher<sensor_msgs::msg::NavSatFix>(odomToGpsPubTopic, 10 /* QoS depth */);
      
  }

 private:

    void getDatum(){
      auto request = std::make_shared<robot_localization::srv::GetDatum::Request>();
      getDatumClient->async_send_request(request, [this](rclcpp::Client<robot_localization::srv::GetDatum>::SharedFuture future) {
        auto result = future.get();
        auto geo_point = result->geo_pose.position;
        auto geo_orientation = result->geo_pose.orientation;
;

        odom2UTM = tf2::Quaternion(geo_orientation.x, geo_orientation.y, geo_orientation.z, geo_orientation.w);
        gtools.lla_origin_ = Eigen::Vector3d(geo_point.latitude, geo_point.longitude, geo_point.altitude);
        
        isInit = true;
      });
    }

    void gpsToOdomCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
      // if(!isInit && !isLocalizationMode)
      //   getDatum();
      // if(!isInit)
      //   return;
      if(!isLocalizationMode)
        getDatum();
      if(!isInit)
        return;
      Eigen::Vector3d lla(msg->latitude, msg->longitude, msg->altitude);

      Eigen::Vector3d ecef = gtools.LLA2ECEF(lla);
      Eigen::Vector3d enu = gtools.ECEF2ENU(ecef);
      
      // tf2::Vector3 position(enu(0), enu(1), 0); // sim utm
      // tf2::Matrix3x3 odom2utm(odom2UTM);
      // position = odom2utm.transpose() * position;
      
      tf2::Vector3 position(-enu(1), enu(0), 0); // seu

      auto poseMsg = geometry_msgs::msg::Pose();
      poseMsg.position.x = position.getX(); 
      poseMsg.position.y = position.getY(); 
      poseMsg.position.z = position.getZ(); 
      gpsToOdomPub->publish(poseMsg);
    }

    void odomToGpsCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
      // if(!isInit && !isLocalizationMode)
      //   getDatum();
      // if(!isInit)
      //   return;
      if(!isLocalizationMode)
        getDatum();
      if(!isInit)
        return;
      
      tf2::Vector3 position = {msg->position.x, msg->position.y, 0};
      tf2::Matrix3x3 odom2utm(odom2UTM);
      position = odom2utm * position;

      auto ecef = gtools.ENU2ECEF(Eigen::Vector3d{position.getX(),position.getY(),position.getZ()});
      auto lla = gtools.ECEF2LLA(ecef);

      auto navSatFixMsg = sensor_msgs::msg::NavSatFix();
      navSatFixMsg.latitude = lla.x();
      navSatFixMsg.longitude = lla.y();
      navSatFixMsg.altitude = lla.z();
      odomToGpsPub->publish(navSatFixMsg);
    }



  void readFile(){
    float initial_lla[3];
    std::ifstream initial_info(map_dir + "initial_LLA_orientation");
    if (!initial_info.is_open()) {
        std::cerr << "Failed to open file." << std::endl;
        exit(1);
    }

    double value;
    for (int i = 0; i < 3; ++i) {
        if (initial_info >> value)
            initial_lla[i] = value;
        else {
            std::cerr << "Failed to read float value from file." << std::endl;
            exit(1);
        }
    }
    float tmp[4];
    for (int i = 0; i < 4; ++i) {
      if (initial_info >> value)
        tmp[i] = value;
      else {
          std::cerr << "Failed to read float value from file." << std::endl;
          exit(1);
      }
    }
    odom2UTM = tf2::Quaternion(tmp[0],tmp[1],tmp[2],tmp[3]);


    initial_info.close();

    // sleep(2); // wait for starting robot_localization 
    auto request = std::make_shared<robot_localization::srv::SetDatum::Request>();
    // geographic_msgs/GeoPose geo_pose
    request->geo_pose.position.latitude = initial_lla[0];
    request->geo_pose.position.longitude = initial_lla[1];
    request->geo_pose.position.altitude = initial_lla[2];
    request->geo_pose.orientation.x = odom2UTM.getX();
    request->geo_pose.orientation.y = odom2UTM.getY();
    request->geo_pose.orientation.z = odom2UTM.getZ();
    request->geo_pose.orientation.w = odom2UTM.getW();

    setDatumClient->async_send_request(request, [this](rclcpp::Client<robot_localization::srv::SetDatum>::SharedFuture future) {});


    gtools.lla_origin_ = Eigen::Vector3d(initial_lla[0],initial_lla[1],initial_lla[2]);
  }


  void imuHandler(const sensor_msgs::msg::Imu::ConstSharedPtr& msg){
    initOrientation = tf2::Quaternion(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    isInitOrientation = true;
  }

  void GNSSCB(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg) {
    if (std::isnan(msg->latitude + msg->longitude + msg->altitude)) {
      RCLCPP_ERROR(this->get_logger(), "POS LLA NAN...");
      return;
    }

    if(!isInitOrientation)
      return;
      
    Eigen::Vector3d lla(msg->latitude, msg->longitude, msg->altitude);

    Eigen::Vector3d ecef = gtools.LLA2ECEF(lla);
    Eigen::Vector3d enu = gtools.ECEF2ENU(ecef);
    
     tf2::Vector3 position(-enu(1), enu(0), 0); // seu
    
    // tf2::Vector3 position(enu(0), enu(1), 0); // sim utm
    // tf2::Matrix3x3 odom2utm(odom2UTM);
    // position = odom2utm.transpose() * position;

    //tf2::Vector3 position(-enu(1), enu(0), 0); // seu

    auto message = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    message->header.frame_id = "base_link";
    message->header.stamp = this->now();

    message->pose.pose.position.x = position.x();
    message->pose.pose.position.y = position.y();
    message->pose.pose.position.z = position.z();

    message->pose.pose.orientation.x = initOrientation.x();
    message->pose.pose.orientation.y = initOrientation.y();
    message->pose.pose.orientation.z = initOrientation.z();
    message->pose.pose.orientation.w = initOrientation.w();

    initialPosePub->publish(*message);

  }

  void ResetOrigin(Eigen::Vector3d &_lla) { gtools.lla_origin_ = _lla; }

  GpsTools gtools;

  
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialPosePub;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsSub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;
  rclcpp::Client<robot_localization::srv::GetDatum>::SharedPtr getDatumClient; // To convert GPS -> SLAM in mapping mode

  bool isInitOrientation = false;
  tf2::Quaternion initOrientation;
  tf2::Quaternion odom2UTM;

  rclcpp::Client<robot_localization::srv::SetDatum>::SharedPtr setDatumClient; // create for localization


  // Convert : GPS - SLAM coordinate
  bool isInit = false;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsToOdomSub;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr odomToGpsSub;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr odomToGpsPub;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr gpsToOdomPub;

  
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  rclcpp::executors::SingleThreadedExecutor exec;

  auto GO = std::make_shared<GNSSOdom>(options);
  exec.add_node(GO);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> GPS converter Started.\033[0m");

  exec.spin();

  return 0;
}