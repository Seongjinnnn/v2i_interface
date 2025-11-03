#ifndef V2I_INTERFACE__STATE_CONVERTER_HPP_
#define V2I_INTERFACE__STATE_CONVERTER_HPP_

#include <string>
#include <its_ros_msgs/msg/s_pa_t_msgs.hpp>
#include <tier4_v2x_msgs/msg/virtual_traffic_light_state.hpp>
#include <map>
#include <vector>


#include <rclcpp/time.hpp>

namespace v2i_interface
{

class StateConverter
{
public:
  explicit StateConverter(
    const std::map<int64_t, std::vector<std::string>>& mapping,
    const std::vector<int64_t>& approval_event_states,
    const std::vector<int64_t>& finalized_event_states);
  ~StateConverter() = default;

  // Convert SPaT message to VirtualTrafficLightState
  std::vector<tier4_v2x_msgs::msg::VirtualTrafficLightState> convertSpaTToVTL(
    const its_ros_msgs::msg::SPaTMsgs & spat_msg,
    const std::string & vtl_type,
    const rclcpp::Time& now);

  // Determine approval based on event_state
  bool getApprovalFromEventState(int64_t event_state) const;

  // Check if signal state is valid
  bool isValidEventState(int64_t event_state) const;

private:
  // Valid event state range (0-9 according to J2735)
  static constexpr int64_t MIN_EVENT_STATE = 0;
  static constexpr int64_t MAX_EVENT_STATE = 9;

  const std::map<int64_t, std::vector<std::string>>& signal_group_to_autoware_vtl_map_;
  const std::vector<int64_t>& approval_event_states_;
  const std::vector<int64_t>& finalized_event_states_;
};

}  // namespace v2i_interface

#endif  // V2I_INTERFACE__STATE_CONVERTER_HPP_
