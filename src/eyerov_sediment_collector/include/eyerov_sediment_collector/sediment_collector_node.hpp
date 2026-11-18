#ifndef EYEROV_SEDIMENT_COLLECTOR__SEDIMENT_COLLECTOR_NODE_HPP_
#define EYEROV_SEDIMENT_COLLECTOR__SEDIMENT_COLLECTOR_NODE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include "eyerov_sediment_interfaces/action/collect_sediment.hpp"
#include "eyerov_sediment_interfaces/msg/sampler_command.hpp"
#include "eyerov_sediment_interfaces/msg/sampler_telemetry.hpp"

namespace eyerov
{

using CollectSediment = eyerov_sediment_interfaces::action::CollectSediment;
using GoalHandleCollect = rclcpp_action::ServerGoalHandle<CollectSediment>;
using SamplerCommand = eyerov_sediment_interfaces::msg::SamplerCommand;
using SamplerTelemetry = eyerov_sediment_interfaces::msg::SamplerTelemetry;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

enum class SequencePhase
{
  IDLE,
  PRE_CHECK,
  DEPLOYING_ARM,
  PENETRATING_SEABED,
  CLOSING_SCOOP,
  RETRACTING_ARM,
  STOWING_SAMPLE,
  SAFE_ABORT_RETRACT
};

class SedimentCollectorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SedimentCollectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SedimentCollectorNode() override;

  // ROS 2 Lifecycle callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  // Action server callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CollectSediment::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleCollect> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleCollect> goal_handle);

  // Main action execution loop (runs in dedicated worker thread)
  void execute_sequence(const std::shared_ptr<GoalHandleCollect> goal_handle);

  // 20 Hz safety watchdog timer callback
  void watchdog_callback();

  // Subscriber callbacks
  void telemetry_callback(const SamplerTelemetry::SharedPtr msg);
  void station_keeping_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void altitude_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void estop_callback(const std_msgs::msg::Bool::SharedPtr msg);

  // Helper methods
  void send_sampler_command(uint8_t cmd, float target_value = 0.0f);
  void trigger_fault(uint8_t error_code, const std::string & reason);
  uint8_t evaluate_safety_conditions(float max_torque_limit, std::string & out_reason);
  bool perform_safe_retraction(
    const std::shared_ptr<GoalHandleCollect> & goal_handle,
    std::shared_ptr<CollectSediment::Feedback> & feedback);
  std::string phase_to_string(SequencePhase phase) const;

  // ROS 2 Interfaces
  rclcpp_action::Server<CollectSediment>::SharedPtr action_server_;
  rclcpp_lifecycle::LifecyclePublisher<SamplerCommand>::SharedPtr cmd_pub_;

  rclcpp::Subscription<SamplerTelemetry>::SharedPtr telemetry_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr station_keeping_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr altitude_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;

  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Configured parameters
  double min_altitude_m_{0.3};
  double max_altitude_m_{1.0};
  double max_motor_current_amp_{12.0};
  double sensor_timeout_sec_{0.5};
  double default_max_torque_nm_{25.0};
  double max_allowed_penetration_cm_{30.0};
  double target_arm_extension_cm_{50.0};
  double safe_retract_timeout_sec_{10.0};

  // Thread-safe state & sensor data
  mutable std::mutex state_mutex_;
  SamplerTelemetry latest_telemetry_{};
  rclcpp::Time last_telemetry_time_{0, 0, RCL_ROS_TIME};
  bool telemetry_received_{false};
  bool station_keeping_ok_{false};
  float current_altitude_m_{0.0f};
  bool emergency_stop_{false};

  SequencePhase current_phase_{SequencePhase::IDLE};
  uint8_t latched_error_code_{CollectSediment::Result::ERR_NONE};
  std::string latched_error_msg_{};

  // Lifecycle & execution flags
  std::atomic<bool> is_active_{false};
  std::atomic<bool> goal_in_progress_{false};
  std::atomic<bool> fault_triggered_{false};
  std::atomic<bool> cancel_requested_{false};

  std::thread execution_thread_;
};

}  // namespace eyerov

#endif  // EYEROV_SEDIMENT_COLLECTOR__SEDIMENT_COLLECTOR_NODE_HPP_
