#include <memory>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Core>
#include <pcl/console/print.h>
#include <vector>
#include <limits>
#include <unordered_set>

class PCDPublisher : public rclcpp::Node {
public:
  PCDPublisher() : Node("pcd_publisher") {
    this->declare_parameter<std::string>("pcd_path", "/root/map_file/real_world_map/resolution0.8/GlobalMap.pcd");
    this->declare_parameter<float>("clipping_minz", -0.4); 
    this->declare_parameter<float>("clipping_maxz", 0.4);
    this->declare_parameter<int>("minNeighborsInRadius", 80);
    this->declare_parameter<float>("clustering_radius", 0.15);
    this->declare_parameter<float>("center_search_radius", 0.7);
    this->declare_parameter<std::string>("tree_pcd_path", "/root/map_file/tree_pcd/tree.pcd");
    
    pcd_path = this->get_parameter("pcd_path").as_string();
    clipping_minz = this->get_parameter("clipping_minz").as_double();
    clipping_maxz = this->get_parameter("clipping_maxz").as_double();
    minNeighborsInRadius = this->get_parameter("minNeighborsInRadius").as_int();
    clustering_radius = this->get_parameter("clustering_radius").as_double();
    center_search_radius = this->get_parameter("center_search_radius").as_double();
    tree_pcd_path = this->get_parameter("tree_pcd_path").as_string();

    publisher_pcd = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
    publisher_pcd_filterd = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_filtered", 10);
    publisher_pcd_clipped = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_clipped", 10);
    publisher_pcd_coordinate = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_axis", 10);
    publisher_pcd_tree_center = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_tree_center", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&PCDPublisher::publishPointCloud, this));
  }

private:
  void publishPointCloud() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load the PCD file.");
      return;
    }
    std::cout << "Loaded "
        << cloud->width * cloud->height
        << " data points from test_pcd.pcd with the following fields: "
        << std::endl;

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(clipping_minz, clipping_maxz);  // minZ와 maxZ는 필터링할 Z값의 범위입니다.
    pass.filter(*cloud_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_radius = kdtree_search_radius(cloud_filtered);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_axis = find_point_cloud_coordinate();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tree_center = find_point_cloud_center(cloud_radius);
    save_pcd(cloud_tree_center);

    sensor_msgs::msg::PointCloud2 output1;
    pcl::toROSMsg(*cloud, output1);
    output1.header.frame_id = "point_cloud_frame";
    output1.header.stamp = this->get_clock()->now();
    publisher_pcd->publish(output1);

    sensor_msgs::msg::PointCloud2 output2;
    pcl::toROSMsg(*cloud_radius, output2);
    output2.header.frame_id = "point_cloud_frame";
    output2.header.stamp = this->get_clock()->now();
    publisher_pcd_filterd->publish(output2);

    sensor_msgs::msg::PointCloud2 output3;
    pcl::toROSMsg(*cloud_filtered, output3);
    output3.header.frame_id = "point_cloud_frame";
    output3.header.stamp = this->get_clock()->now();
    publisher_pcd_clipped->publish(output3);

    sensor_msgs::msg::PointCloud2 output4;
    pcl::toROSMsg(*cloud_axis, output4);
    output4.header.frame_id = "point_cloud_frame";
    output4.header.stamp = this->get_clock()->now();
    publisher_pcd_coordinate->publish(output4);

    sensor_msgs::msg::PointCloud2 output5;
    pcl::toROSMsg(*cloud_tree_center, output5);
    output5.header.frame_id = "point_cloud_frame";
    output5.header.stamp = this->get_clock()->now();
    publisher_pcd_tree_center->publish(output5);

    // 포인트 수를 출력합니다.
    // std::cout << "Number of points: " << cloud->size() << std::endl;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr kdtree_search_radius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // k-d 트리 객체를 생성합니다.
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // 필터링된 인덱스를 저장할 객체입니다.
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // 모든 포인트에 대해 주변 이웃을 검사합니다.
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        // 현재 포인트 주변의 이웃을 탐색합니다.
        if (kdtree.radiusSearch(cloud->points[i], clustering_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > minNeighborsInRadius) {
            // 최소 이웃 수 이상이면 결과에 포함시킵니다.
            inliers->indices.push_back(i);
        }
    }
    // 필터링된 포인트 클라우드를 추출합니다.
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>());
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.filter(*filteredCloud);

    return filteredCloud;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr find_point_cloud_coordinate() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->push_back(pcl::PointXYZ(1, 0, 0));
    cloud->push_back(pcl::PointXYZ(0, 1, 0));
    cloud->push_back(pcl::PointXYZ(0, 0, 1));

    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr find_point_cloud_center(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // k-d 트리 객체를 생성합니다.
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // 필터링된 인덱스를 저장할 객체입니다.
    pcl::PointCloud<pcl::PointXYZ>::Ptr central_points(new pcl::PointCloud<pcl::PointXYZ>);
    std::unordered_set<int> processed_indices;

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        // 이미 처리된 점은 건너뜁니다.
        if (processed_indices.find(i) != processed_indices.end()) {
            continue;
        }

        std::vector<int> point_indices;
        std::vector<float> point_squared_distance;

        // 반지름 내의 모든 점 검색
        if (kdtree.radiusSearch(cloud->points[i], center_search_radius, point_indices, point_squared_distance) > 0) {
            float sum_x = 0, sum_y = 0, sum_z = 0;
            int count = 0;

            // 원점으로부터 가장 가까운 점 찾기
            for (int idx : point_indices) {
                if (processed_indices.find(idx) != processed_indices.end()) {
                    continue; // 이미 처리된 점은 건너뜁니다.
                }
                sum_x += cloud->points[idx].x;
                sum_y += cloud->points[idx].y;
                sum_z += cloud->points[idx].z;
                count++;
                processed_indices.insert(idx);
            }
            if (count > 0) {
                pcl::PointXYZ central_point(sum_x / count, sum_y / count, sum_z / count);
                central_points->points.push_back(central_point);
            }
        }
    }
    return central_points;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr save_pcd(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::cout<<"d";
    cloud->height = 1;
    std::cout<<"c";
    cloud->width = cloud->points.size();
    std::cout<<"a";
    cloud->is_dense = false;
    std::cout<<"b";
    pcl::io::savePCDFile(tree_pcd_path, *cloud);
  }

  float squaredDistance(const pcl::PointXYZ& point1, const pcl::PointXYZ& point2) {
    float diff_x = point2.x - point1.x;
    float diff_y = point2.y - point1.y;
    float diff_z = point2.z - point1.z;
    return diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcd;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcd_filterd;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcd_clipped;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcd_coordinate;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcd_tree_center;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string pcd_path;
  std::string tree_pcd_path;
  float clipping_minz;
  float clipping_maxz;
  int minNeighborsInRadius;
  float clustering_radius;
  float center_search_radius;

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PCDPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
