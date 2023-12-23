#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include "behavior/bahavior_node.h"

int main(int argc, char **argv) {
  // 创建行为树加工厂
  BT::BehaviorTreeFactory factory;
  // 初始化 ROS2
  rclcpp::init(argc,argv);
  auto node = std::make_shared<ros2_node>("node_start");

  //注册树节点
  factory.registerNodeType<action_node_1>("action_node_1");
  factory.registerNodeType<action_node_2>("action_node_2");
  // 注册ros发布的树节点
  factory.registerNodeType<ROS2DecoratorNode>("ROS2DecoratorNode");

  // 从文件创建一个行为树
  auto tree = factory.createTreeFromFile("./src/behavior/bt_tree.xml");
  // 启动监视
  BT::Groot2Publisher publisher(tree);
  while (rclcpp::ok()) {
    tree.tickWhileRunning();
    rclcpp::spin_some(node);
    std::cout << "----------------------------" << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}