#include "eyerov_sediment_collector/sediment_collector_node.hpp"

#include <cmath>
#include <functional>

namespace eyerov
{

using namespace std::chrono_literals;

SedimentCollectorNode::SedimentCollectorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("sediment_collector_node", options)
{
  // Declare configurable safety & operational parameters
  this->declare_parameter<double>("min_altitude_m", 0.30);
  this->declare_parameter<double>("max_altitude_m", 1.00);
  this->declare_parameter<double>("max_motor_current_amp", 12.0);
  this->declare_parameter<double>("sensor_timeout_sec", 0.50);
  this->declare_parameter<double>("default_max_torque_nm", 25.0);
  this->declare_parameter<double>("max_allowed_penetration_cm", 30.0);
  this->declare_parameter<double>("target_arm_extension_cm", 50.0);
  this->declare_parameter<double>("safe_retract_timeout_sec", 10.0);

  RCLCPP_INFO(get_logger(), "SedimentCollectorNode constructed (Unconfigured).");
}

SedimentCollectorNode::~SedimentCollectorNode()
{
  is_active_.store(false);
  fault_triggered_.store(true);
  if (execution_thread_.joinable()) {
    execution_thread_.join();
  }
}

CallbackReturn SedimentCollectorNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring SedimentCollectorNode...");

  min_altitude_m_ = this->get_parameter("min_altitude_m").as_double();
  max_altitude_m_ = this->get_parameter("max_altitude_m").as_double();
  max_motor_current_amp_ = this->get_parameter("max_motor_current_amp").as_double();
  sensor_timeout_sec_ = this->get_parameter("sensor_timeout_sec").as_double();
  default_max_torque_nm_ = this->get_parameter("default_max_torque_nm").as_double();
  max_allowed_penetration_cm_ = this->get_parameter("max_allowed_penetration_cm").as_double();
  target_arm_extension_cm_ = this->get_parameter("target_arm_extension_cm").as_double();
  safe_retract_timeout_sec_ = this->get_parameter("safe_retract_timeout_sec").as_double();

  // Lifecycle publisher for hardware actuator commands
  cmd_pub_ = this->create_publisher<SamplerCommand>("/sampler/cmd", rclcpp::QoS(10));

  // Subscriptions to sampler feedback and ROV state
  telemetry_sub_ = this->create_subscription<SamplerTelemetry>(
    "/sampler/telemetry", rclcpp::QoS(10),
    std::bind(&SedimentCollectorNode::telemetry_callback, this, std::placeholders::_1));

  station_keeping_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/rov/station_keeping_ok", rclcpp::QoS(10),
    std::bind(&SedimentCollectorNode::station_keeping_callback, this, std::placeholders::_1));

  altitude_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/rov/altitude_m", rclcpp::QoS(10),
    std::bind(&SedimentCollectorNode::altitude_callback, this, std::placeholders::_1));

  estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/rov/emergency_stop", rclcpp::QoS(10),
    std::bind(&SedimentCollectorNode::estop_callback, this, std::placeholders::_1));

  // Create ROS 2 Action Server
  action_server_ = rclcpp_action::create_server<CollectSediment>(
    this,
    "/collect_sediment",
    std::bind(&SedimentCollectorNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&SedimentCollectorNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&SedimentCollectorNode::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Configuration complete. Node is now INACTIVE.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SedimentCollectorNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating SedimentCollectorNode...");

  cmd_pub_->on_activate();
  is_active_.store(true);
  fault_triggered_.store(false);
  cancel_requested_.store(false);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_phase_ = SequencePhase::IDLE;
    latched_error_code_ = CollectSediment::Result::ERR_NONE;
    latched_error_msg_.clear();
  }

  // Send initial IDLE command to ensure actuators are stationary
  send_sampler_command(SamplerCommand::CMD_IDLE, 0.0f);

  // Start 20 Hz safety watchdog timer
  watchdog_timer_ = this->create_wall_timer(
    50ms, std::bind(&SedimentCollectorNode::watchdog_callback, this));

  RCLCPP_INFO(get_logger(), "Node is ACTIVE and ready to accept /collect_sediment goals.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SedimentCollectorNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_WARN(get_logger(), "Deactivating SedimentCollectorNode!");

  is_active_.store(false);

  if (watchdog_timer_) {
    watchdog_timer_->cancel();
    watchdog_timer_.reset();
  }

  // If a goal is actively running, trigger fail-safe abort immediately
  if (goal_in_progress_.load()) {
    trigger_fault(
      CollectSediment::Result::ERR_NODE_DEACTIVATED,
      "Lifecycle node deactivated during active sediment collection.");
  }

  if (execution_thread_.joinable()) {
    execution_thread_.join();
  }

  // Command safe halt before deactivating publisher
  if (cmd_pub_ && cmd_pub_->is_activated()) {
    send_sampler_command(SamplerCommand::CMD_HALT, 0.0f);
    cmd_pub_->on_deactivate();
  }

  RCLCPP_INFO(get_logger(), "Node is now INACTIVE.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SedimentCollectorNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up SedimentCollectorNode resources...");

  if (execution_thread_.joinable()) {
    execution_thread_.join();
  }

  watchdog_timer_.reset();
  action_server_.reset();
  cmd_pub_.reset();
  telemetry_sub_.reset();
  station_keeping_sub_.reset();
  altitude_sub_.reset();
  estop_sub_.reset();

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    telemetry_received_ = false;
  }

  RCLCPP_INFO(get_logger(), "Cleanup complete. Node is UNCONFIGURED.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SedimentCollectorNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down SedimentCollectorNode...");
  is_active_.store(false);
  fault_triggered_.store(true);

  if (watchdog_timer_) {
    watchdog_timer_->cancel();
  }
  if (execution_thread_.joinable()) {
    execution_thread_.join();
  }
  return CallbackReturn::SUCCESS;
}

// ============================================================================
// Subscriber Callbacks
// ============================================================================

void SedimentCollectorNode::telemetry_callback(const SamplerTelemetry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_telemetry_ = *msg;
  last_telemetry_time_ = this->now();
  telemetry_received_ = true;
}

void SedimentCollectorNode::station_keeping_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  station_keeping_ok_ = msg->data;
}

void SedimentCollectorNode::altitude_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_altitude_m_ = msg->data;
}

void SedimentCollectorNode::estop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  bool prev_estop = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    prev_estop = emergency_stop_;
    emergency_stop_ = msg->data;
  }

  if (msg->data && !prev_estop) {
    RCLCPP_ERROR(get_logger(), "EMERGENCY STOP signal received!");
    if (goal_in_progress_.load()) {
      trigger_fault(
        CollectSediment::Result::ERR_ESTOP,
        "Hardware/Topside Emergency Stop triggered.");
    }
  }
}

// ============================================================================
// 20 Hz Watchdog Timer
// ============================================================================

void SedimentCollectorNode::watchdog_callback()
{
  if (!is_active_.load() || !goal_in_progress_.load() || fault_triggered_.load()) {
    return;
  }

  std::string reason;
  uint8_t err = evaluate_safety_conditions(static_cast<float>(default_max_torque_nm_), reason);
  if (err != CollectSediment::Result::ERR_NONE) {
    trigger_fault(err, reason);
  }
}

// ============================================================================
// Action Server Callbacks
// ============================================================================

rclcpp_action::GoalResponse SedimentCollectorNode::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const CollectSediment::Goal> goal)
{
  if (!is_active_.load()) {
    RCLCPP_WARN(get_logger(), "Goal rejected: Node is not in ACTIVE lifecycle state.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal_in_progress_.load()) {
    RCLCPP_WARN(get_logger(), "Goal rejected: Another sediment collection sequence is in progress.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->target_penetration_cm <= 0.0f ||
      goal->target_penetration_cm > static_cast<float>(max_allowed_penetration_cm_))
  {
    RCLCPP_WARN(
      get_logger(),
      "Goal rejected: target_penetration_cm (%.2f) out of valid range (0, %.2f].",
      goal->target_penetration_cm, max_allowed_penetration_cm_);
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->max_torque_nm <= 0.0f || goal->timeout_sec <= 1.0f) {
    RCLCPP_WARN(get_logger(), "Goal rejected: Invalid torque limit or timeout parameter.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(
    get_logger(),
    "Accepted CollectSediment goal: depth=%.1f cm, max_torque=%.1f Nm, timeout=%.1f s",
    goal->target_penetration_cm, goal->max_torque_nm, goal->timeout_sec);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SedimentCollectorNode::handle_cancel(
  const std::shared_ptr<GoalHandleCollect>)
{
  RCLCPP_WARN(get_logger(), "Received cancellation request from client. Initiating safe abort...");
  cancel_requested_.store(true);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SedimentCollectorNode::handle_accepted(const std::shared_ptr<GoalHandleCollect> goal_handle)
{
  if (execution_thread_.joinable()) {
    execution_thread_.join();
  }

  goal_in_progress_.store(true);
  fault_triggered_.store(false);
  cancel_requested_.store(false);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latched_error_code_ = CollectSediment::Result::ERR_NONE;
    latched_error_msg_.clear();
  }

  execution_thread_ = std::thread(
    &SedimentCollectorNode::execute_sequence, this, goal_handle);
}

// ============================================================================
// Core Sediment Collection Sequence & Fail-Safe Execution
// ============================================================================
void SedimentCollectorNode::execute_sequence(const std::shared_ptr<GoalHandleCollect> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<CollectSediment::Feedback>();
  auto result = std::make_shared<CollectSediment::Result>();

  const rclcpp::Time start_time = this->now();
  const float max_torque = goal->max_torque_nm;
  const float target_depth = goal->target_penetration_cm;
  const double total_timeout = static_cast<double>(goal->timeout_sec);

  float max_achieved_depth = 0.0f;
  rclcpp::Rate loop_rate(20.0);  // 20 Hz control loop
  auto publish_feedback = [&](SequencePhase phase, float progress) {
    SamplerTelemetry telem;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_phase_ = phase;
      telem = latest_telemetry_;
    }
    feedback->current_phase = phase_to_string(phase);
    feedback->progress_pct = progress;
    feedback->current_penetration_cm = telem.penetration_depth_cm;
    feedback->current_torque_nm = telem.axial_torque_nm;
    goal_handle->publish_feedback(feedback);
  };

  auto check_all_guards = [&]() -> bool {
    // 1. Check elapsed total goal timeout
    if ((this->now() - start_time).seconds() > total_timeout) {
      trigger_fault(
        CollectSediment::Result::ERR_EXECUTION_TIMEOUT,
        "Sequence exceeded total allowed goal timeout.");
    }

    // 2. Check live sensor, torque, station-keeping, and E-Stop conditions
    if (!fault_triggered_.load()) {
      std::string reason;
      uint8_t err = evaluate_safety_conditions(max_torque, reason);
      if (err != CollectSediment::Result::ERR_NONE) {
        trigger_fault(err, reason);
      }
    }

    return !fault_triggered_.load() && !cancel_requested_.load() && is_active_.load();
  };

  // --------------------------------------------------------------------------
  // PHASE 1: PRE_CHECK
  // --------------------------------------------------------------------------
  RCLCPP_INFO(get_logger(), "[Phase 1/6] Running PRE_CHECK...");
  publish_feedback(SequencePhase::PRE_CHECK, 5.0f);

  {
    std::string precheck_reason;
    uint8_t err = evaluate_safety_conditions(max_torque, precheck_reason);
    if (err != CollectSediment::Result::ERR_NONE) {
      RCLCPP_ERROR(get_logger(), "PRE_CHECK failed: %s", precheck_reason.c_str());
      result->success = false;
      result->error_code = CollectSediment::Result::ERR_PRECHECK_FAILED;
      result->collected_depth_cm = 0.0f;
      result->status_message = "Pre-check failed: " + precheck_reason;
      goal_handle->abort(result);

      std::lock_guard<std::mutex> lock(state_mutex_);
      current_phase_ = SequencePhase::IDLE;
      goal_in_progress_.store(false);
      return;
    }
  }

  // --------------------------------------------------------------------------
  // PHASE 2: DEPLOYING_ARM
  // --------------------------------------------------------------------------
  RCLCPP_INFO(get_logger(), "[Phase 2/6] Deploying manipulator arm to seabed approach...");
  send_sampler_command(SamplerCommand::CMD_DEPLOY_ARM, static_cast<float>(target_arm_extension_cm_));

  while (rclcpp::ok()) {
    if (!check_all_guards()) {
      break;
    }

    float ext = 0.0f;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      ext = latest_telemetry_.arm_extension_cm;
    }

    float phase_ratio = std::min(1.0f, ext / static_cast<float>(target_arm_extension_cm_));
    publish_feedback(SequencePhase::DEPLOYING_ARM, 10.0f + 15.0f * phase_ratio);

    if (ext >= static_cast<float>(target_arm_extension_cm_) - 0.5f) {
      RCLCPP_INFO(get_logger(), "Arm deployed (%.1f cm).", ext);
      break;
    }
    loop_rate.sleep();
  }
  // --------------------------------------------------------------------------
  // PHASE 3: PENETRATING_SEABED
  // --------------------------------------------------------------------------
  if (!fault_triggered_.load() && !cancel_requested_.load()) {
    RCLCPP_INFO(get_logger(), "[Phase 3/6] Penetrating seabed to %.1f cm...", target_depth);
    send_sampler_command(SamplerCommand::CMD_PENETRATE, target_depth);

    while (rclcpp::ok()) {
      if (!check_all_guards()) {
        break;
      }

      float depth = 0.0f;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        depth = latest_telemetry_.penetration_depth_cm;
      }
      max_achieved_depth = std::max(max_achieved_depth, depth);

      float phase_ratio = std::min(1.0f, depth / target_depth);
      publish_feedback(SequencePhase::PENETRATING_SEABED, 25.0f + 25.0f * phase_ratio);

      if (depth >= target_depth - 0.2f) {
        RCLCPP_INFO(get_logger(), "Target seabed penetration reached (%.1f cm).", depth);
        break;
      }
      loop_rate.sleep();
    }
  }
  // --------------------------------------------------------------------------
  // PHASE 4: CLOSING_SCOOP
  // --------------------------------------------------------------------------
  if (!fault_triggered_.load() && !cancel_requested_.load()) {
    RCLCPP_INFO(get_logger(), "[Phase 4/6] Closing clamshell scoop / core shutter...");
    send_sampler_command(SamplerCommand::CMD_CLOSE_SCOOP, 1.0f);

    while (rclcpp::ok()) {
      if (!check_all_guards()) {
        break;
      }

      bool closed = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        closed = latest_telemetry_.scoop_closed;
      }

      publish_feedback(SequencePhase::CLOSING_SCOOP, 60.0f);

      if (closed) {
        RCLCPP_INFO(get_logger(), "Sediment scoop sealed.");
        break;
      }
      loop_rate.sleep();
    }
  }
  // --------------------------------------------------------------------------
  // PHASE 5: RETRACTING_ARM
  // --------------------------------------------------------------------------
  if (!fault_triggered_.load() && !cancel_requested_.load()) {
    RCLCPP_INFO(get_logger(), "[Phase 5/6] Retracting arm clear of seabed...");
    send_sampler_command(SamplerCommand::CMD_RETRACT_ARM, 0.0f);

    while (rclcpp::ok()) {
      if (!check_all_guards()) {
        break;
      }

      float ext = 0.0f;
      float depth = 0.0f;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ext = latest_telemetry_.arm_extension_cm;
        depth = latest_telemetry_.penetration_depth_cm;
      }

      float retract_progress = 1.0f - std::min(1.0f, ext / static_cast<float>(target_arm_extension_cm_));
      publish_feedback(SequencePhase::RETRACTING_ARM, 65.0f + 20.0f * retract_progress);

      if (depth <= 0.1f && ext <= 1.0f) {
        RCLCPP_INFO(get_logger(), "Arm retracted to clear position.");
        break;
      }
      loop_rate.sleep();
    }
  }
  // --------------------------------------------------------------------------
  // PHASE 6: STOWING_SAMPLE
  // --------------------------------------------------------------------------
  if (!fault_triggered_.load() && !cancel_requested_.load()) {
    RCLCPP_INFO(get_logger(), "[Phase 6/6] Stowing sample into sealed canister...");
    send_sampler_command(SamplerCommand::CMD_STOW_SAMPLE, 1.0f);

    while (rclcpp::ok()) {
      if (!check_all_guards()) {
        break;
      }

      bool locked = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        locked = latest_telemetry_.canister_locked;
      }

      publish_feedback(SequencePhase::STOWING_SAMPLE, 95.0f);

      if (locked) {
        RCLCPP_INFO(get_logger(), "Sample canister locked.");
        break;
      }
      loop_rate.sleep();
    }
  }

  // ==========================================================================
  // FAIL-SAFE / CANCELLATION HANDLING (ALWAYS RETRACT SAFELY!)
  // ==========================================================================
  if (fault_triggered_.load() || cancel_requested_.load()) {
    bool retracted_ok = perform_safe_retraction(goal_handle, feedback);

    uint8_t final_code = CollectSediment::Result::ERR_USER_CANCELED;
    std::string final_msg = "Sequence canceled by operator; arm safely retracted.";

    if (fault_triggered_.load()) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      final_code = latched_error_code_;
      final_msg = latched_error_msg_;
    }
    if (!retracted_ok) {
      final_msg += " [WARNING: Safe retraction timed out - verify visual camera!]";
    } else {
      final_msg += " [Arm safely retracted and stowed.]";
    }

    result->success = false;
    result->error_code = final_code;
    result->collected_depth_cm = max_achieved_depth;
    result->status_message = final_msg;

    if (cancel_requested_.load() && !fault_triggered_.load()) {
      RCLCPP_WARN(get_logger(), "Goal CANCELED: %s", final_msg.c_str());
      goal_handle->canceled(result);
    } else {
      RCLCPP_ERROR(get_logger(), "Goal ABORTED (code %u): %s", final_code, final_msg.c_str());
      goal_handle->abort(result);
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_phase_ = SequencePhase::IDLE;
    }
    goal_in_progress_.store(false);
    return;
  }


  // ==========================================================================
  // NORMAL COMPLETION
  // ==========================================================================
  send_sampler_command(SamplerCommand::CMD_IDLE, 0.0f);
  publish_feedback(SequencePhase::IDLE, 100.0f);

  result->success = true;
  result->error_code = CollectSediment::Result::ERR_NONE;
  result->collected_depth_cm = max_achieved_depth;
  result->status_message = "Sediment sample successfully collected and stowed in sealed canister.";

  RCLCPP_INFO(get_logger(), "Goal SUCCEEDED! Collected core depth: %.1f cm.", max_achieved_depth);
  goal_handle->succeed(result);
  goal_in_progress_.store(false);
}


// ============================================================================
// Safety Evaluation & Emergency Retraction Helpers
// ============================================================================
uint8_t SedimentCollectorNode::evaluate_safety_conditions(
  float max_torque_limit, std::string & out_reason)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (emergency_stop_) {
    out_reason = "Emergency stop is active.";
    return CollectSediment::Result::ERR_ESTOP;
  }

  if (!telemetry_received_) {
    out_reason = "No sampler telemetry received yet.";
    return CollectSediment::Result::ERR_SENSOR_TIMEOUT;
  }

  const double age_sec = (this->now() - last_telemetry_time_).seconds();
  if (age_sec > sensor_timeout_sec_) {
    out_reason = "Sampler telemetry watchdog timeout (" + std::to_string(age_sec) + "s > " +
      std::to_string(sensor_timeout_sec_) + "s).";
    return CollectSediment::Result::ERR_SENSOR_TIMEOUT;
  }

  if (!station_keeping_ok_) {
    out_reason = "ROV station-keeping lost (vehicle drifting).";
    return CollectSediment::Result::ERR_STATION_KEEPING_LOST;
  }

  if (current_altitude_m_ < static_cast<float>(min_altitude_m_) ||
      current_altitude_m_ > static_cast<float>(max_altitude_m_))
  {
    out_reason = "ROV altitude (" + std::to_string(current_altitude_m_) +
      "m) outside safe bounds [" + std::to_string(min_altitude_m_) +
      "m, " + std::to_string(max_altitude_m_) + "m].";
    return CollectSediment::Result::ERR_STATION_KEEPING_LOST;
  }

  if (latest_telemetry_.hardware_fault) {
    out_reason = "Sampler hardware driver reported an internal fault.";
    return CollectSediment::Result::ERR_HARDWARE_FAULT;
  }
  if (latest_telemetry_.axial_torque_nm > max_torque_limit ||
      latest_telemetry_.motor_current_amp > static_cast<float>(max_motor_current_amp_))
  {
    out_reason = "Over-torque/obstruction detected (torque=" +
    std::to_string(latest_telemetry_.axial_torque_nm) + " Nm, limit=" +
    std::to_string(max_torque_limit) + " Nm).";
    return CollectSediment::Result::ERR_OVER_TORQUE;
  }
  return CollectSediment::Result::ERR_NONE;
}

void SedimentCollectorNode::trigger_fault(uint8_t error_code, const std::string & reason)
{
  bool expected = false;
  if (fault_triggered_.compare_exchange_strong(expected, true)) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latched_error_code_ = error_code;
      latched_error_msg_ = reason;
    }
    RCLCPP_ERROR(get_logger(), "FAIL-SAFE TRIGGERED [Code %u]: %s", error_code, reason.c_str());
    send_sampler_command(SamplerCommand::CMD_EMERGENCY_RETRACT, 0.0f);
  }
}

bool SedimentCollectorNode::perform_safe_retraction(
  const std::shared_ptr<GoalHandleCollect> & goal_handle,
  std::shared_ptr<CollectSediment::Feedback> & feedback)
{
  RCLCPP_WARN(get_logger(), "Executing SAFE_ABORT_RETRACT sequence...");
  send_sampler_command(SamplerCommand::CMD_EMERGENCY_RETRACT, 0.0f);

  const rclcpp::Time retract_start = this->now();
  rclcpp::Rate rate(20.0);

  while (rclcpp::ok()) {
    SamplerTelemetry telem;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_phase_ = SequencePhase::SAFE_ABORT_RETRACT;
      telem = latest_telemetry_;
    }
    feedback->current_phase = phase_to_string(SequencePhase::SAFE_ABORT_RETRACT);
    feedback->progress_pct = 0.0f;
    feedback->current_penetration_cm = telem.penetration_depth_cm;
    feedback->current_torque_nm = telem.axial_torque_nm;
    goal_handle->publish_feedback(feedback);

    if (telem.penetration_depth_cm <= 0.1f && telem.arm_extension_cm <= 1.0f) {
      RCLCPP_INFO(get_logger(), "SAFE_ABORT_RETRACT complete: Manipulator is stowed safely.");
      send_sampler_command(SamplerCommand::CMD_IDLE, 0.0f);
      return true;
    }

    if ((this->now() - retract_start).seconds() > safe_retract_timeout_sec_) {
      RCLCPP_ERROR(get_logger(), "SAFE_ABORT_RETRACT timed out! Commanding actuator HALT.");
      send_sampler_command(SamplerCommand::CMD_HALT, 0.0f);
      return false;
    }

    rate.sleep();
  }

  return false;
}

void SedimentCollectorNode::send_sampler_command(uint8_t cmd, float target_value)
{
  if (cmd_pub_ && cmd_pub_->is_activated()) {
    SamplerCommand msg;
    msg.command = cmd;
    msg.target_value = target_value;
    cmd_pub_->publish(msg);
  }
}

std::string SedimentCollectorNode::phase_to_string(SequencePhase phase) const
{
  switch (phase) {
    case SequencePhase::IDLE: return "IDLE";
    case SequencePhase::PRE_CHECK: return "PRE_CHECK";
    case SequencePhase::DEPLOYING_ARM: return "DEPLOYING_ARM";
    case SequencePhase::PENETRATING_SEABED: return "PENETRATING_SEABED";
    case SequencePhase::CLOSING_SCOOP: return "CLOSING_SCOOP";
    case SequencePhase::RETRACTING_ARM: return "RETRACTING_ARM";
    case SequencePhase::STOWING_SAMPLE: return "STOWING_SAMPLE";
    case SequencePhase::SAFE_ABORT_RETRACT: return "SAFE_ABORT_RETRACT";
    default: return "UNKNOWN";
  }
}

}  // namespace eyerov