#include <mutex>
#include <random>

#include "rclcpp/rclcpp.hpp"

#include "vtr_logging/logging.hpp"

namespace vtr {
namespace exps {

std::string random_string(std::size_t length) {
  const std::string CHARACTERS = "abcdefghijklmnopqrstuvwxyz";
  std::random_device random_device;
  std::mt19937 generator(random_device());
  std::uniform_int_distribution<> distribution(0, CHARACTERS.size() - 1);
  std::string result;
  for (std::size_t i = 0; i < length; ++i) result += CHARACTERS[distribution(generator)];
  return result;
}

struct TestControl {
  TestControl(const rclcpp::Node::SharedPtr &node) {
    // parameters to control the playback
    // clang-format off
    play_ = node->declare_parameter<bool>("control_test.play", play_);
    terminate_ = node->declare_parameter<bool>("control_test.terminate", terminate_);
    // clang-format on
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    rcl_interfaces::msg::IntegerRange delay_range;
    delay_range.from_value = 0;
    delay_range.to_value = 1000;
    delay_range.step = 1;
    param_desc.integer_range.emplace_back(delay_range);
    delay_ = node->declare_parameter<int>("control_test.delay_millisec", delay_, param_desc);
    parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        node->get_node_base_interface(), node->get_node_topics_interface(), node->get_node_graph_interface(),
        node->get_node_services_interface());
    parameter_event_sub_ =
        parameter_client_->on_parameter_event([&](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
          std::lock_guard<std::mutex> lock(mutex_);
          for (auto &changed_parameter : event->changed_parameters) {
            const auto &type = changed_parameter.value.type;
            const auto &name = changed_parameter.name;
            const auto &value = changed_parameter.value;
            CLOG(WARNING, "test") << "Received parameter change event with name: " << name;

            if (type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
              if (name == "control_test.delay_millisec") {
                delay_ = value.integer_value;
              }
            } else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
              if (name == "control_test.play") {
                play_ = value.bool_value;
              } else if (name == "control_test.terminate") {
                terminate_ = value.bool_value;
              }
            }
          }
        });
  }

  bool play() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return play_;
  }

  bool terminate() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return terminate_;
  }

  int delay() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return delay_;
  }

 private:
  mutable std::mutex mutex_;

  bool play_ = true;
  bool terminate_ = false;
  int delay_ = 0;  // milliseconds

  rclcpp::AsyncParametersClient::SharedPtr parameter_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
};

}  // namespace exps
}  // namespace vtr
