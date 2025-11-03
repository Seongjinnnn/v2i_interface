#include "v2i_interface/state_converter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>


#include <sstream>
#include <iomanip>

namespace v2i_interface
{

StateConverter::StateConverter(
  const std::map<int64_t, std::vector<std::string>>& mapping,
  const std::vector<int64_t>& approval_event_states,
  const std::vector<int64_t>& finalized_event_states)
: signal_group_to_autoware_vtl_map_(mapping),
  approval_event_states_(approval_event_states),
  finalized_event_states_(finalized_event_states)
{
}

std::vector<tier4_v2x_msgs::msg::VirtualTrafficLightState> StateConverter::convertSpaTToVTL(
  const its_ros_msgs::msg::SPaTMsgs & spat_msg,
  const std::string & vtl_type,
  const rclcpp::Time& now)
{
  std::vector<tier4_v2x_msgs::msg::VirtualTrafficLightState> vtl_states;

  auto it = signal_group_to_autoware_vtl_map_.find(spat_msg.signal_group_id);
  if (it != signal_group_to_autoware_vtl_map_.end()) {
    bool approval = getApprovalFromEventState(spat_msg.event_state);
    // Check if event_state is in finalized_event_states_
    bool is_finalized = std::find(finalized_event_states_.begin(), finalized_event_states_.end(), spat_msg.event_state) != finalized_event_states_.end();

    for (const std::string& autoware_vtl_id : it->second) {
      tier4_v2x_msgs::msg::VirtualTrafficLightState vtl_state;
      if (spat_msg.header.stamp.sec == 0 && spat_msg.header.stamp.nanosec == 0) {
        vtl_state.stamp = now;
      } else {
        vtl_state.stamp = spat_msg.header.stamp;
      }
      vtl_state.type = vtl_type;
      vtl_state.id = autoware_vtl_id;
      vtl_state.approval = approval;
      vtl_state.is_finalized = is_finalized;
      vtl_states.push_back(vtl_state);
    }
  } else {
    RCLCPP_WARN(rclcpp::get_logger("StateConverter"), "signal_group_id %ld not found in mapping table. No VTL states generated.", spat_msg.signal_group_id);
  }
  return vtl_states;
}

bool StateConverter::getApprovalFromEventState(int64_t event_state) const
{
  // Check if event_state is in approval_event_states_
  return std::find(approval_event_states_.begin(), approval_event_states_.end(), event_state) != approval_event_states_.end();
}

bool StateConverter::isValidEventState(int64_t event_state) const
{
  return (event_state >= MIN_EVENT_STATE && event_state <= MAX_EVENT_STATE);
}

}  // namespace v2i_interface
