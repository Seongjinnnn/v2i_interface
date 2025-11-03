#ifndef V2I_INTERFACE__V2I_INTERFACE_NODE_HPP_
#define V2I_INTERFACE__V2I_INTERFACE_NODE_HPP_

#include <memory>
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <its_ros_msgs/msg/s_pa_t_msgs.hpp>
#include <tier4_v2x_msgs/msg/virtual_traffic_light_state.hpp>
#include <tier4_v2x_msgs/msg/virtual_traffic_light_state_array.hpp>

#include "v2i_interface/state_converter.hpp"

namespace v2i_interface
{

class V2IInterfaceNode : public rclcpp::Node
{
public:
  explicit V2IInterfaceNode(const rclcpp::NodeOptions & options);
  ~V2IInterfaceNode() = default;

private:
  // Callback for SPaT messages
  void signalInfoCallback(const its_ros_msgs::msg::SPaTMsgs::SharedPtr msg);

  // Timer callback for periodic publishing
  void publishTimerCallback();

  // Check if signal is still valid (not timed out)
  bool isSignalValid(const rclcpp::Time & last_signal_time) const;

  // Remove stale signals from cache
  void removeStaleSignals();

  // Parameters
  std::string input_topic_;
  std::string output_topic_;
  std::string virtual_traffic_light_type_;
  double signal_timeout_sec_;
  bool default_to_stop_;

  // State management
  std::map<std::string, tier4_v2x_msgs::msg::VirtualTrafficLightState> vtl_states_;
  std::map<std::string, rclcpp::Time> last_update_times_;

  // Mapping from signal_group_id to Autoware VTL IDs
  std::map<int64_t, std::vector<std::string>> signal_group_to_autoware_vtl_map_;

  // Configurable event states for approval and finalization
  std::vector<int64_t> approval_event_states_;
  std::vector<int64_t> finalized_event_states_;

  // ROS interfaces
  rclcpp::Subscription<its_ros_msgs::msg::SPaTMsgs>::SharedPtr signal_sub_;
  rclcpp::Publisher<tier4_v2x_msgs::msg::VirtualTrafficLightStateArray>::SharedPtr vtl_pub_;

  // Timer for periodic publishing
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // State converter
  std::unique_ptr<StateConverter> converter_;
};

}  // namespace v2i_interface

#endif  // V2I_INTERFACE__V2I_INTERFACE_NODE_HPP_
