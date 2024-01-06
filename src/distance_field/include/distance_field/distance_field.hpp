#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <list>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

class DistanceField : public rclcpp::Node {
 public:
  DistanceField(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "Distance_Field_Node已启动");
    // costmap2d_subscriber_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    //   shared_from_this(), "global_costmap");
    
    // costmap = costmap2d_subscriber_->getCostmap();
    // nav2_costmap_2d::calculate_distance_field(*costmap);
    // distanceField = costmap->getDistanceField();
    
    clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", 100, 
      std::bind(&DistanceField::PointSubCallback, this, std::placeholders::_1));
    

  }


 private:
  void PointSubCallback(geometry_msgs::msg::PointStamped::SharedPtr clicked_point) {
    // if (clicked_point != nullptr) {
    //   RCLCPP_INFO(this->get_logger(), "Received a point: (%f, %f, %f)", 
    //     clicked_point->point.x, clicked_point->point.y, clicked_point->point.z);
    //   // 在这里处理接收到的点
      
    //   RCLCPP_INFO(this->get_logger(), "Obstacle distance: %f", obstacle_distance);
    // }
  }

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  // std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap2d_subscriber_;
  // std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  // std::vector<std::vector<double>> distanceField;
};




