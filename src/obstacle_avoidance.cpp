#include "geometry_msgs/msg/twist.hpp"    // Twist
#include "rclcpp/rclcpp.hpp"              // ROS Core Libraries
#include "sensor_msgs/msg/laser_scan.hpp" // Laser Scan
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

using std::placeholders::_1;

class ObstacleAvoidance : public rclcpp::Node
{
public:
  ObstacleAvoidance() : Node("ObstacleAvoidance")
  {

    // auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/filtered_lidar_data", 10,
                                                                             std::bind(&ObstacleAvoidance::topic_callback, this, _1));

    publisher_ = create_publisher<geometry_msgs::msg::Twist>("control_cmd", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr _msg)
  {

    const uint32_t num_points = _msg->width * _msg->height;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*_msg, *cloud);

    float min = 10;
    float current = 0;

    for (int p = 0; p < cloud->points.size(); ++p)
    {
      // orijinden olan karesel mesafeyi bul
      float pointDepth2 = (cloud->points[p].x * cloud->points[p].x) +
                          (cloud->points[p].y * cloud->points[p].y) +
                          (cloud->points[p].z * cloud->points[p].z);

      current = sqrt(pointDepth2);
      if (current < min)
      {
        min = current;
      }
    }

    auto message = this->calculateVelMsg(min);
    publisher_->publish(message);
  }

  geometry_msgs::msg::Twist calculateVelMsg(float distance)
  {
    auto msg = geometry_msgs::msg::Twist();
    // logic
    RCLCPP_INFO(this->get_logger(), "Distance is: '%f'", distance);

    if (distance < 1)
    {
      // turn around
      msg.linear.x = 0;
      msg.angular.z = 0.3;
    }
    else
    {
      // go straight ahead
      msg.linear.x = 0.3;
      msg.angular.z = 0;
    }
    return msg;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoidance>());
  rclcpp::shutdown();
  return 0;
}
