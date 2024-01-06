#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"


class CostmapClickSubscriber : public rclcpp::Node {
public:
  CostmapClickSubscriber()
    : Node("costmap_click_subscriber")
  {
    RCLCPP_INFO(this->get_logger(), "Starting costmap click subscriber");
    costmap_subscriber_ = this->create_subscription<nav2_msgs::msg::Costmap>(
      "/costmap/costmap_raw", 10, std::bind(&CostmapClickSubscriber::costmapCallback, this, std::placeholders::_1));

    pose_click_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/clicked_point", 10, std::bind(&CostmapClickSubscriber::poseClickCallback, this, std::placeholders::_1));
  }

private:
  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "Received costmap");
    index += 200;
    current_costmap_ = *msg;
    std::cout << "代价值：" << current_costmap_.data[index] << std::endl;
  }

  void poseClickCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Check if a costmap has been received
    if (current_costmap_.data.empty())
    {
      RCLCPP_WARN(get_logger(), "No costmap received yet");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Received pose click");
    // Convert the clicked pose to costmap coordinates
    unsigned int mx, my;
    mx = msg->pose.position.x / current_costmap_.metadata.resolution;
    my = msg->pose.position.y / current_costmap_.metadata.resolution;
    // Get the cost at the clicked point
    unsigned char cost = current_costmap_.data[current_costmap_.metadata.size_x * my + mx];
    RCLCPP_INFO(get_logger(), "Cost at clicked point: %d", cost);
  }

  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_click_subscriber_;
  nav2_msgs::msg::Costmap current_costmap_;
  int index = 100;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  rclcpp::spin(std::make_shared<CostmapClickSubscriber>());
  rclcpp::shutdown();
  return 0;
}
