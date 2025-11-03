#include "v2i_interface/v2i_interface_node.hpp"

#include <chrono>
#include <functional>
#include <set>
#include <string>

using namespace std::chrono_literals;

namespace v2i_interface
{

V2IInterfaceNode::V2IInterfaceNode(const rclcpp::NodeOptions & options)
: Node("v2i_interface_node", options)
{
  // Declare and get parameters
  input_topic_ = this->declare_parameter<std::string>("input_topic", "/signal_info");
  output_topic_ = this->declare_parameter<std::string>("output_topic", "/awapi/tmp/virtual_traffic_light_states");
  virtual_traffic_light_type_ = this->declare_parameter<std::string>("virtual_traffic_light_type", "TRAFFIC_LIGHT");
  signal_timeout_sec_ = this->declare_parameter<double>("signal_timeout_sec", 5.0);
  default_to_stop_ = this->declare_parameter<bool>("default_to_stop", true);
  approval_event_states_ = this->declare_parameter<std::vector<int64_t>>("approval_event_states", {6});
  finalized_event_states_ = this->declare_parameter<std::vector<int64_t>>("finalized_event_states", {6});

  // Manually create the mapping
  RCLCPP_INFO(this->get_logger(), "Hardcoding signal group mapping as a workaround.");
  signal_group_to_autoware_vtl_map_[9] = {"8347", "8508"};
  signal_group_to_autoware_vtl_map_[1] = {"8613"};
  signal_group_to_autoware_vtl_map_[5] = {"8628"};
  signal_group_to_autoware_vtl_map_[13] = {"8861", "8874"};
  RCLCPP_INFO(this->get_logger(), "Finished loading signal group mapping. Map size: %zu", signal_group_to_autoware_vtl_map_.size());

  // Initialize state converter with the mapping and event state configurations
  converter_ = std::make_unique<StateConverter>(signal_group_to_autoware_vtl_map_, approval_event_states_, finalized_event_states_);

  // Create subscriber for SPaT messages
  signal_sub_ = this->create_subscription<its_ros_msgs::msg::SPaTMsgs>(
    input_topic_, 10,
    std::bind(&V2IInterfaceNode::signalInfoCallback, this, std::placeholders::_1));

  // Create publisher for VTL state array
  vtl_pub_ = this->create_publisher<tier4_v2x_msgs::msg::VirtualTrafficLightStateArray>(
    output_topic_, 10);

  // Create timer for periodic publishing (10Hz)
  publish_timer_ = this->create_wall_timer(
    100ms,
    std::bind(&V2IInterfaceNode::publishTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "V2I Interface Node initialized");
  RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  VTL type: %s", virtual_traffic_light_type_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Signal timeout: %.1f sec", signal_timeout_sec_);
}

void V2IInterfaceNode::signalInfoCallback(const its_ros_msgs::msg::SPaTMsgs::SharedPtr msg)
{
  // Validate event state
  if (!converter_->isValidEventState(msg->event_state)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid event_state: %ld for signal_group_id: %ld",
      msg->event_state, msg->signal_group_id);
    return;
  }

  // Convert SPaT to VTL states (now returns a vector)
  auto new_vtl_states = converter_->convertSpaTToVTL(*msg, virtual_traffic_light_type_, this->now());

  if (new_vtl_states.empty()) {
      RCLCPP_WARN(this->get_logger(), "No Autoware VTL IDs found for signal_group_id: %ld. No VTL states updated.", msg->signal_group_id);
      return;
  }

  // Update state cache for each new VTL state
  for (const auto& vtl_state : new_vtl_states) {
      vtl_states_[vtl_state.id] = vtl_state; // Key by Autoware VTL ID
      last_update_times_[vtl_state.id] = this->now();
      RCLCPP_DEBUG(
          this->get_logger(),
          "Received signal: ID=%s, state=%ld, approval=%s",
          vtl_state.id.c_str(), msg->event_state,
          vtl_state.approval ? "true" : "false");
  }
}

void V2IInterfaceNode::publishTimerCallback()
{
  // Remove stale signals
  removeStaleSignals();

  // If no active signals, publish empty array
  if (vtl_states_.empty()) {
    tier4_v2x_msgs::msg::VirtualTrafficLightStateArray array_msg;
    array_msg.stamp = this->now();
    vtl_pub_->publish(array_msg);
    return;
  }

  // Aggregate all active VTL states
  tier4_v2x_msgs::msg::VirtualTrafficLightStateArray array_msg;
  array_msg.stamp = this->now();

  for (const auto & [vtl_id, vtl_state] : vtl_states_) { // Iterate by VTL ID
    array_msg.states.push_back(vtl_state);
  }

  // Publish VTL state array
  vtl_pub_->publish(array_msg);

  RCLCPP_DEBUG(
    this->get_logger(),
    "Published %zu virtual traffic light states", array_msg.states.size());
}

bool V2IInterfaceNode::isSignalValid(const rclcpp::Time & last_signal_time) const
{
  auto elapsed = (this->now() - last_signal_time).seconds();
  return elapsed < signal_timeout_sec_;
}

void V2IInterfaceNode::removeStaleSignals()
{
  std::vector<std::string> stale_vtl_ids;

  // Find stale signals
  for (const auto & [vtl_id, last_time] : last_update_times_) { // Iterate by VTL ID
    if (!isSignalValid(last_time)) {
      stale_vtl_ids.push_back(vtl_id);
      RCLCPP_WARN(
        this->get_logger(),
        "Signal timeout: VTL ID=%s (%.1f sec since last update)",
        vtl_id.c_str(), (this->now() - last_time).seconds());
    }
  }

  // Remove stale signals from cache
  for (const auto & vtl_id : stale_vtl_ids) {
    vtl_states_.erase(vtl_id);
    last_update_times_.erase(vtl_id);
  }
}

}  // namespace v2i_interface

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(v2i_interface::V2IInterfaceNode)
