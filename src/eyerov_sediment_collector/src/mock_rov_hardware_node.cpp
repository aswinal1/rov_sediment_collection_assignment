#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include "eyerov_sediment_interfaces/msg/sampler_command.hpp"
#include "eyerov_sediment_interfaces/msg/sampler_telemetry.hpp"

using namespace std::chrono_literals;
using SamplerCommand = eyerov_sediment_interfaces::msg::SamplerCommand;
using SamplerTelemetry = eyerov_sediment_interfaces::msg::SamplerTelemetry;

class MockRovHardwareNode : public rclcpp::Node
{
public:
  MockRovHardwareNode()
  : Node("mock_rov_hardware_node")
  {
    // Fault-injection parameters (can be toggled at startup or via `ros2 param set`)
    this->declare_parameter<bool>("inject_rock_obstruction", false);
    this->declare_parameter<bool>("inject_drift_fault", false);
    this->declare_parameter<bool>("inject_sensor_freeze", false);
    this->declare_parameter<bool>("inject_estop", false);
    this->declare_parameter<bool>("inject_hardware_fault", false);
    this->declare_parameter<double>("simulated_altitude_m", 0.50);

    telemetry_pub_ = this->create_publisher<SamplerTelemetry>("/sampler/telemetry", 10);
    station_keeping_pub_ = this->create_publisher<std_msgs::msg::Bool>("/rov/station_keeping_ok", 10);
    altitude_pub_ = this->create_publisher<std_msgs::msg::Float32>("/rov/altitude_m", 10);
    estop_pub_ = this->create_publisher<std_msgs::msg::Bool>("/rov/emergency_stop", 10);

    cmd_sub_ = this->create_subscription<SamplerCommand>(
      "/sampler/cmd", 10,
      std::bind(&MockRovHardwareNode::command_callback, this, std::placeholders::_1));

    // 20 Hz physics & telemetry simulation loop
    sim_timer_ = this->create_wall_timer(
      50ms, std::bind(&MockRovHardwareNode::simulation_step, this));

    RCLCPP_INFO(get_logger(), "MockRovHardwareNode started (20 Hz telemetry active).");
  }

private:
  void command_callback(const SamplerCommand::SharedPtr msg)
  {
    active_cmd_ = msg->command;
    target_val_ = msg->target_value;
  }

  void simulation_step()
  {
    const bool rock_fault = this->get_parameter("inject_rock_obstruction").as_bool();
    const bool drift_fault = this->get_parameter("inject_drift_fault").as_bool();
    const bool sensor_freeze = this->get_parameter("inject_sensor_freeze").as_bool();
    const bool estop_fault = this->get_parameter("inject_estop").as_bool();
    const bool hw_fault = this->get_parameter("inject_hardware_fault").as_bool();
    const float sim_alt = static_cast<float>(this->get_parameter("simulated_altitude_m").as_double());

    // Publish ROV vehicle health & station-keeping state
    std_msgs::msg::Bool sk_msg;
    sk_msg.data = !drift_fault;
    station_keeping_pub_->publish(sk_msg);

    std_msgs::msg::Float32 alt_msg;
    alt_msg.data = drift_fault ? 1.85f : sim_alt;
    altitude_pub_->publish(alt_msg);

    std_msgs::msg::Bool estop_msg;
    estop_msg.data = estop_fault;
    estop_pub_->publish(estop_msg);

    // Simulate actuator kinematics (dt = 0.05s)
    const float dt = 0.05f;
    switch (active_cmd_) {
      case SamplerCommand::CMD_IDLE:
      case SamplerCommand::CMD_HALT:
        motor_current_ = 0.4f;
        break;

      case SamplerCommand::CMD_DEPLOY_ARM:
        scoop_closed_ = false;
        canister_locked_ = false;
        arm_extension_ = std::min(target_val_, arm_extension_ + 25.0f * dt);
        axial_torque_ = 2.0f;
        motor_current_ = 3.2f;
        break;

      case SamplerCommand::CMD_PENETRATE:
        penetration_depth_ = std::min(target_val_, penetration_depth_ + 6.0f * dt);
        if (rock_fault && penetration_depth_ >= 5.0f) {
          // Simulate hitting solid bedrock at 5 cm depth
          axial_torque_ = 38.5f;
          motor_current_ = 15.8f;
        } else {
          // Soft seabed sediment resistance increases gradually with depth
          axial_torque_ = 4.0f + 0.65f * penetration_depth_;
          motor_current_ = 4.5f + 0.25f * penetration_depth_;
        }
        break;

      case SamplerCommand::CMD_CLOSE_SCOOP:
        scoop_timer_ += dt;
        axial_torque_ = 8.5f;
        motor_current_ = 5.5f;
        if (scoop_timer_ >= 1.2f) {
          scoop_closed_ = true;
          scoop_timer_ = 0.0f;
        }
        break;

      case SamplerCommand::CMD_RETRACT_ARM:
        if (penetration_depth_ > 0.0f) {
          penetration_depth_ = std::max(0.0f, penetration_depth_ - 10.0f * dt);
          axial_torque_ = 6.0f;
        } else {
          arm_extension_ = std::max(0.0f, arm_extension_ - 30.0f * dt);
          axial_torque_ = 2.5f;
        }
        motor_current_ = 4.2f;
        break;

      case SamplerCommand::CMD_STOW_SAMPLE:
        stow_timer_ += dt;
        motor_current_ = 3.0f;
        if (stow_timer_ >= 1.0f) {
          canister_locked_ = true;
          stow_timer_ = 0.0f;
        }
        break;

      case SamplerCommand::CMD_EMERGENCY_RETRACT:
        // High-speed safe vertical extraction
        if (penetration_depth_ > 0.0f) {
          penetration_depth_ = std::max(0.0f, penetration_depth_ - 20.0f * dt);
        } else {
          arm_extension_ = std::max(0.0f, arm_extension_ - 45.0f * dt);
        }
        axial_torque_ = 3.0f;
        motor_current_ = 6.5f;
        break;
    }

    // If sensor freeze fault is injected, stop publishing sampler telemetry
    if (sensor_freeze) {
      return;
    }

    SamplerTelemetry telem;
    telem.header.stamp = this->now();
    telem.header.frame_id = "sampler_link";
    telem.arm_extension_cm = arm_extension_;
    telem.penetration_depth_cm = penetration_depth_;
    telem.scoop_closed = scoop_closed_;
    telem.canister_locked = canister_locked_;
    telem.axial_torque_nm = axial_torque_;
    telem.motor_current_amp = motor_current_;
    telem.hardware_fault = hw_fault;

    telemetry_pub_->publish(telem);
  }

  rclcpp::Publisher<SamplerTelemetry>::SharedPtr telemetry_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr station_keeping_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr altitude_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Subscription<SamplerCommand>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr sim_timer_;

  uint8_t active_cmd_{SamplerCommand::CMD_IDLE};
  float target_val_{0.0f};
  float arm_extension_{0.0f};
  float penetration_depth_{0.0f};
  bool scoop_closed_{false};
  bool canister_locked_{false};
  float axial_torque_{0.0f};
  float motor_current_{0.4f};
  float scoop_timer_{0.0f};
  float stow_timer_{0.0f};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockRovHardwareNode>());
  rclcpp::shutdown();
  return 0;
}

