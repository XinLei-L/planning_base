#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

std::string my_ros_pub_value = " ";

class ros2_node : public rclcpp::Node {
 public:
  ros2_node(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "ros2_node constructor");
    chatter_pub_ = this->create_publisher<std_msgs::msg::String>("ros_pub", 10);
    chatter_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ros2_node::timerCallback, this));
    chatter_sub_ = this->create_subscription<std_msgs::msg::String>("ros_pub",
                                                                     10,
                                                                     std::bind(&ros2_node::chatterCallback, this, std::placeholders::_1));
  }

  void chatterCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
  }

  void timerCallback() {
    RCLCPP_INFO(this->get_logger(), "timerCallback");
    std_msgs::msg::String msg;
    msg.data = my_ros_pub_value;
    chatter_pub_->publish(msg);
  }


 private:
  int i = 0;
  // 定义一个名为"chatter"的发布者
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chatter_pub_;
  rclcpp::TimerBase::SharedPtr chatter_timer_;
  // 定义一个名为"chatter"的订阅者
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr chatter_sub_;

};

class action_node_1  : public BT::SyncActionNode {
public:
  action_node_1(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override {
    //端口映射
    std::string HP_write_value;
    std::string ros_pub_value;
    HP_write_value = "1" + std::to_string(i++);
    ros_pub_value = "hello ros:" + std::to_string(i++);
    setOutput("HP_write", HP_write_value);
    setOutput("ros_pub", ros_pub_value);
    // getInput("HP", HP_write_value);
    std::cout << "action1：HP_write_value: " << HP_write_value << std::endl;
    // std::cout << "action1：ros_pub_value: " << ros_pub_value << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts(){
    return {
      //端口注册
      BT::OutputPort<std::string>("HP_write"),
      BT::OutputPort<std::string>("ros_pub"),
    };
  }

private:
  int i = 0;
};

class action_node_2  : public BT::SyncActionNode {
public:
  action_node_2(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override {
    //端口映射
    std::string HP_read_value;
    getInput("HP_read", HP_read_value);
    std::cout << "action2：HP_read_value: " << HP_read_value << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts(){
    return {
      //端口注册
      BT::InputPort<std::string>("HP_read"),
    };
  }
};

//创建ros对行为树的修饰器
class ROS2DecoratorNode  : public BT::SyncActionNode {
public:
  ROS2DecoratorNode(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus tick() override {
    //端口映射
    getInput("ros_pub", my_ros_pub_value);
    std::cout << "ros_pub：ros_pub_value: " << my_ros_pub_value << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts() {
    return {
      //端口注册
    BT::InputPort<std::string>("ros_pub"),
    };
  }
};


