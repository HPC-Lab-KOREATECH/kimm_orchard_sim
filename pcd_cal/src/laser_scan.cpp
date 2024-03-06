#include <limits>
#include <memory>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>

// auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_reliability_policy_e));

class PCDPublisher : public rclcpp::Node {
public:
  PCDPublisher() : Node("pcd_publisher") {
    publisher_pcd = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan_123", 10);
    // publisher_pcd_filterd = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_filtered", 10);
    // publisher_pcd_clipped = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_clipped", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points", rclcpp::SensorDataQoS(), std::bind(&PCDPublisher::publishPointCloud, this, std::placeholders::_1));
  }

private:
  void publishPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(minZ, maxZ);  // minZ와 maxZ는 필터링할 Z값의 범위입니다.
    pass.filter(*cloud_filtered);

    sensor_msgs::msg::LaserScan laser_scan = convertPointCloudToLaserScan(cloud_filtered);

    publisher_pcd->publish(laser_scan);
  }

  sensor_msgs::msg::LaserScan convertPointCloudToLaserScan(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    sensor_msgs::msg::LaserScan scan;

    // LaserScan 메시지의 기본 설정
    scan.header.frame_id = "laser_data_frame";  // 프레임 ID 설정
    scan.header.stamp = rclcpp::Clock().now();  // 현재 시각
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = M_PI / 180;  // 1도 간격 M_PI / 180.0
    scan.range_min = 0.5;
    scan.range_max = 200.0;  // 최대 거리 설정

    // 각도 범위에 따른 배열 크기 계산
    uint32_t ranges_size = static_cast<uint32_t>(std::round((scan.angle_max - scan.angle_min) / scan.angle_increment));
    scan.ranges.assign(ranges_size, std::numeric_limits<float>::infinity());

    for (const auto& point : cloud->points) {
        float range = std::sqrt(point.x * point.x + point.y * point.y);
        if (range < scan.range_min || range > scan.range_max) {
            continue;  // 거리가 범위를 벗어나면 무시
        }

        float angle = std::atan2(point.y, point.x);
        if (angle < scan.angle_min || angle > scan.angle_max) {
            continue;  // 각도가 범위를 벗어나면 무시
        }

        // 각도를 인덱스로 변환
        int index = static_cast<int>((angle - scan.angle_min) / scan.angle_increment);
        if (index >= 0 && index < static_cast<int>(ranges_size)) {
            scan.ranges[index] = std::min(range, scan.ranges[index]);  // 가장 가까운 거리 저장
        }
    }

    return scan;
  }
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_pcd;
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcd_filterd;
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcd_clipped;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  // 클리핑 값
  float minZ = -0.2;
  float maxZ = 0.4;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PCDPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
