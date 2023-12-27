#include "geometry_msgs/msg/twist.hpp"    // Twist
#include "rclcpp/rclcpp.hpp"              // ROS Core Libraries
#include "sensor_msgs/msg/laser_scan.hpp" // Laser Scan
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <numbers> // std::numbers
#define _USE_MATH_DEFINES
using std::placeholders::_1;

class LaserDataPlaning : public rclcpp::Node
{
  public:
  LaserDataPlaning() : Node("LaserDataPlaning")
  {

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/filtered_lidar_data", 10,
                                                                             std::bind(&LaserDataPlaning::topic_callback, this, _1));

    publisher_ = create_publisher<geometry_msgs::msg::Twist>("control_cmd", 10);
  }


  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr _msg)
  {

    const uint32_t num_points = _msg->width * _msg->height;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*_msg, *cloud);

    // float min = 10;
    float current_distance = 0;
    float x, y, z;
    // lidar kordinatı gerekli
    float x_origin = 0;
    float y_origin = 0;
    float relative_angle;
    float degree;
    double pi = M_PI;
    int count_area1 = 0, count_area2 = 0, count_area3 = 0, count_area4 = 0, count_area5 = 0, count_area6 = 0;

    for (int p = 0; p < cloud->points.size(); ++p)
    {
      x = cloud->points[p].x;
      y = cloud->points[p].y;
      z = cloud->points[p].z;
      // orijinden olan karesel mesafeyi bul
      float pointDepth2 = (cloud->points[p].x * cloud->points[p].x) +
                          (cloud->points[p].y * cloud->points[p].y);
      // RCLCPP_INFO(this->get_logger(), "her şey ok..");;

      RCLCPP_INFO(this->get_logger(), "x=%f,y=%f", x, y);
      current_distance = sqrt(pointDepth2);
      double temp_x = x_origin - x;
      double temp_y = y_origin - y;

      relative_angle = std::atan(temp_y / temp_x);



if (x>=0) & (y>=0)
 
relative_angle =  atan((temp_y )/(temp_x));
 
 
if (x<=0) & (y>=0)
        
relative_angle =  2*pi + atan((temp_y)/(temp_x));


if (x<=0) & (y<=0)
 
relative_angle = -pi + atan((temp_y )/(temp_x));

 
if (x>=0) & (y<=0)
 
relative_angle = pi + atan((temp_y )/(temp_x));

 



      degree = relative_angle * (180.0 / pi);
              //RCLCPP_INFO(this->get_logger(), "degree");


      RCLCPP_INFO(this->get_logger(), "degree:%f", degree);

      if (current_distance > 4 && current_distance <= 5) //güvenlik çemberi
      {
        RCLCPP_INFO(this->get_logger(), "Araç yavaşlatılıyor...");
      }

      else if (current_distance <= 4)
      {
        if ((degree > 0 && degree <60) || (degree < 0 &&  degree >= -60))
        {
          RCLCPP_INFO(this->get_logger(), "area1");
          count_area1 += 1;  
        }
        else{
          count_area2 +=1;
          RCLCPP_INFO(this->get_logger(), "###area2####");
        }
        /*
        else if ((degree >= 60 && degree < 90))
        {
          RCLCPP_INFO(this->get_logger(), "area2");
          count_area2 += 1;
          // RCLCPP_INFO(this->get_logger(), "degree:%f",degree);
        }
        else if ((degree >= 90 && degree < 150))
        {

          RCLCPP_INFO(this->get_logger(), "area3");
          count_area3 += 1;

          // RCLCPP_INFO(this->get_logger(), "degree:%f",degree);
        }
        else if ((degree >= 150 && degree < 180) || (degree >= -180 && degree < -150))
        {

          RCLCPP_INFO(this->get_logger(), "area4");
          count_area4 += 1;

          // RCLCPP_INFO(this->get_logger(), "degree:%f",degree);
        }
        else if ((degree >= -150 && degree < -90))
        {

          RCLCPP_INFO(this->get_logger(), "area5");
          count_area5 += 1;

          // RCLCPP_INFO(this->get_logger(), "degree:%f",degree);
        }
        else if ((degree >= -90 && degree < -60))
        {

          RCLCPP_INFO(this->get_logger(), "area6");
          count_area6 += 1;

          // RCLCPP_INFO(this->get_logger(), "degree:%f",degree);
        }

        else {

        }
        */

       



        if (count_area1 >= 100 || count_area2 >= 100 || count_area3 >= 100 || count_area4 >= 100 || count_area5 >= 100 || count_area6 >= 100)
        {
          RCLCPP_INFO(this->get_logger(), "Uygun aksiyomlar gönderiliyor...");
        }
      }



    }

    // auto message = this->calculateVelMsg(current_distance);
    // publisher_->publish(message);
  }

  geometry_msgs::msg::Twist calculateVelMsg(float distance)
  {
    auto msg = geometry_msgs::msg::Twist();
    // logic
    // RCLCPP_INFO(this->get_logger(), "Distance is: '%f'", distance);

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
  rclcpp::spin(std::make_shared<LaserDataPlaning>());
  rclcpp::shutdown();
  return 0;
}
