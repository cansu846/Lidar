#include <memory>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber")
  {
    // Veriyi yayınlamak için bir publisher oluşturun
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_lidar_data", 10);
    //Rosbag den yayınlan topic adı kullanıldı
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points2", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z"); // Z ekseninde filtreleme yapılacak
    pass.setFilterLimits(-1, 4); // Min ve max z değerleri arasındaki noktaları koru
    pass.filter(*cloud_filtered);

    /*
    //isteğe bağlı z değerleri bastırılabilir
    for (int p = 0; p < cloud_filtered->points.size(); ++p)
      {
         RCLCPP_INFO(this->get_logger(), "z=%f", cloud_filtered->points[p].z );
      }
    */

    // Convert to ROS data type
    sensor_msgs::msg::PointCloud2 filtered_lidar_data;
    pcl::toROSMsg(*cloud_filtered, filtered_lidar_data);

    // Yayıncıya yeni mesajı gönderin
    auto msg_test = std::make_shared<sensor_msgs::msg::PointCloud2>();
    publisher_->publish(filtered_lidar_data);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}