#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

using namespace std::chrono_literals;

static constexpr char const *lifecycle_node = "my_lcnode";

// 每一个lifecycle node都有很多服务与它相关
// 按惯例都用 <node name>/<service name> 的形式
// 在这个demo中我们使用 get_state 和 change_state
// 因此这两个service为:
// lc_talker/get_state
// lc_talker/change_state
static constexpr char const *node_get_state_topic = "lc_talker/get_state";
static constexpr char const *node_change_state_topic = "lc_talker/change_state";

template <typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT &future, WaitTimeT time_to_wait) {
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {
      break;
    }
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

class LifecycleServiceClient : public rclcpp::Node {
public:
  explicit LifecycleServiceClient(const std::string &node_name) : Node(node_name)
  { }

  void init() {
    // 每个 lifecycle node 都会自动产生几个service来允许外部交互
    // The two main important ones are GetState and ChangeState.
    client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(node_get_state_topic);
    client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(node_change_state_topic);
  }

  /// Requests the current state of the node
  /** 在这个函数中，我们发送一个service请求来询问lc_talker节点的当前状态
   * 如果在给定的 time_out 时间内return了，那就返回当前状态，如果没有就返回一个unknown状态
   * \param time_out 超时的time (秒)
   */
  unsigned int get_state(std::chrono::seconds time_out = 3s) {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    if (!client_get_state_->wait_for_service(time_out)) {
      RCLCPP_ERROR(get_logger(), "Service %s is not available.", client_get_state_->get_service_name());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // We send the service request for asking the current
    // state of the lc_talker node.
    auto future_result = client_get_state_->async_send_request(request);

    // Let's wait until we have the answer from the node.

    auto future_status = wait_for_result(future_result, time_out);

    // If the request times out, we return an unknown state.
    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Server time out while getting current state for node %s", lifecycle_node);
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // We have an succesful answer. So let's print the current state.
    if (future_result.get()) {
      RCLCPP_INFO(get_logger(), "Node %s has current state %s.", lifecycle_node,
                  future_result.get()->current_state.label.c_str());
      return future_result.get()->current_state.id;
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to get current state for node %s", lifecycle_node);
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
  }

  /// Invokes a transition
  /**
   * 我们发送一个service请求来表示我们希望根据id "transition" 来进行切换
   * 默认的切换有：
   * - configure
   * - activate
   * - cleanup
   * - shutdown
   * @param transition 区别切换的id
   * @param time_out 超时的time (秒)
   */
  bool change_state(std::uint8_t transition, std::chrono::seconds time_out = 3s) {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    if (!client_change_state_->wait_for_service(time_out)) {
      RCLCPP_ERROR(get_logger(), "Service %s is not available.", client_change_state_->get_service_name());
      return false;
    }
    // We send the request with the transition we want to invoke.
    auto future_result = client_change_state_->async_send_request(request);
    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Server time out while getting current state for node %s", lifecycle_node);
      return false;
    }

    // We have an answer, let's print our success.
    if (future_result.get()->success) {
      RCLCPP_INFO(get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
      return true;
    }
    else {
      RCLCPP_WARN(get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
      return false;
    }
  }

private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
};

/**
 * 这是一个简单的脚本来依次触发各个服务
 * It starts with configure, activate,
 * deactivate, activate, deactivate,
 * cleanup and finally shutdown
 */
void callee_script(std::shared_ptr<LifecycleServiceClient> lc_client) {
  rclcpp::WallRate time_between_state_changes(0.25); // 4s
  // configure
  {
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // activate
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return;
    }
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // deactivate
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return;
    }
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // activate it again
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return;
    }
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // and deactivate it again
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return;
    }
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // we cleanup
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return;
    }
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // and finally shutdown
  // Note: We have to be precise here on which shutdown transition id to call
  // We are currently in the unconfigured state and thus have to call
  // TRANSITION_UNCONFIGURED_SHUTDOWN
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return;
    }
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto lc_client = std::make_shared<LifecycleServiceClient>("lc_client");
  lc_client->init();

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(lc_client);

  std::shared_future<void> script = std::async(std::launch::async, std::bind(callee_script, lc_client));
  exe.spin_until_future_complete(script);

  rclcpp::shutdown();

  return 0;
}
