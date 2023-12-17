#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <list>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


class Bresenham : public rclcpp::Node {
public:
  Bresenham(std::string name) : Node(name) {
    goal_position_ = nullptr;
    start_position_ = nullptr;
    map_data_ = nullptr;
    RCLCPP_INFO(this->get_logger(), "Bresenham已启动");
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10, 
      std::bind(&Bresenham::GoalPoseSubCallback, this, std::placeholders::_1));
    init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10, 
      std::bind(&Bresenham::StartPoseSubCallback, this, std::placeholders::_1));
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(rclcpp::KeepLast(5)).reliable().transient_local(),
      std::bind(&Bresenham::MapSubCallback, this, std::placeholders::_1));
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
      "path_pub", rclcpp::QoS(rclcpp::KeepLast(5)));
    path_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
      std::bind(&Bresenham::run, this));
  }

  void run() {
    bool collision = false;
    // std::cout << "宽：" << map_data_->info.width << "高：" << map_data_->info.height << std::endl;
    if (goal_position_ != nullptr && start_position_ != nullptr && map_data_ != nullptr) {
      // std::cout << "地图、起始位置、目标位置信息" << std::endl;
      collision = bresenham();
      if (collision) {
        RCLCPP_INFO(this->get_logger(), "碰撞");
      } else {
        RCLCPP_INFO(this->get_logger(), "无碰撞");
      }
      goal_position_ = nullptr;
      start_position_ = nullptr;
    }
  }

  bool bresenham() {
    float x0, y0, x1, y1;
    y0 = start_position_->pose.pose.position.y;
    x0 = start_position_->pose.pose.position.x;
    y1 = goal_position_->pose.position.y;
    x1 = goal_position_->pose.position.x;
    float steep = (abs(y1 - y0) > abs(x1 - x0));
    float step = map_data_->info.resolution;
    if (steep) {
      std::swap(x0, y0);
      std::swap(x1, y1);
    }
    if (x0 > x1) {
      std::swap(x0, x1);
      std::swap(y0, y1);
    }
    float delta_x = x1 - x0;
    float delta_y = abs(y1 - y0);
    float error = 0;
    float delta_error = delta_y / delta_x;
    float yk = y0;

    if (y0 > y1) {
      step = -step;
    }
    for (float xk = x0; xk < x1; xk += abs(step)) {
      bool check_result = false;
      if (steep) {
        check_result = CheckNodeIsCollision(yk, xk);
      } else {
        check_result = CheckNodeIsCollision(xk, yk);
      }
      if(check_result) {
        return true;
      }
      error = error + delta_error;
      if(error >= map_data_->info.resolution / 2) {
        yk += step;
        error -= map_data_->info.resolution;
      }
    }
    return false;
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr map_data_;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_position_; 
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr start_position_;  

private:
  void GoalPoseSubCallback(geometry_msgs::msg::PoseStamped::SharedPtr goal_pose) {
    if (goal_pose != nullptr) {
      if (!CheckNodeIsCollision(goal_pose->pose.position.x, goal_pose->pose.position.y)) {
        RCLCPP_INFO(this->get_logger(), "目标位置已收到");
        goal_position_ = goal_pose;
      }
    }
  }

  void StartPoseSubCallback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr start_pose) {
    if (start_pose != nullptr) {
      if (!CheckNodeIsCollision(start_pose->pose.pose.position.x, start_pose->pose.pose.position.y)) {
        RCLCPP_INFO(this->get_logger(), "起始位置已收到");
        start_position_ = start_pose;
      }
    }
  }

  void MapSubCallback(nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    if (map != nullptr) {
      std::cout << "地图信息已收到" << std::endl;
      map_data_ = map;
    }
  }

  bool CheckNodeIsCollision(float x, float y) {
    int index_x = (x - map_data_->info.origin.position.x) / map_data_->info.resolution;
    int index_y = (y - map_data_->info.origin.position.y) / map_data_->info.resolution;
    int index = index_x + index_y * map_data_->info.width;
    //std::cout << "index_x: " << index_x << " index_y: " << index_y << " index: " << index << std::endl;
    if (map_data_->data[index] == 100 || map_data_->data[index] == -1)
      return true;
    else
      return false;
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::TimerBase::SharedPtr path_timer_;

};



int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Bresenham>("Bresenham");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}