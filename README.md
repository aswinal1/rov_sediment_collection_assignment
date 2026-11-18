# EyeROV Seabed Sediment Collection System (ROS 2 Humble / C++)

## Overview
This repository contains the complete system architecture and ROS 2 C++ implementation for an ROV seabed sediment collection subsystem:
* **System Architecture (`ARCHITECTURE.md`):** Complete ROV subsystem design (Topside, GNC/Station-Keeping, Perception, Vehicle Health, and Sediment Collection), Mermaid block/state diagrams, and fail-safe matrix.
* **`eyerov_sediment_interfaces`:** Custom ROS 2 Action (`CollectSediment.action`) and telemetry/command messages (`SamplerTelemetry.msg`, `SamplerCommand.msg`).
* **`eyerov_sediment_collector`:** Thread-safe C++17 `LifecycleNode` Action Server implementing a 6-phase deterministic state machine with 9 explicit fail-safes, paired with a `MockRovHardwareNode` for fault-injected hardware-in-the-loop demonstration.

---

## Repository Structure
```text
rov_sediment_collection_assignment/
├── ARCHITECTURE.md                     # Complete ROV system architecture & fail-safe design
├── README.md                           # Build & verification guide
├── scripts/
│   └── verify_all.sh                   # Automated end-to-end nominal & fail-safe test runner
└── src/
    ├── eyerov_sediment_interfaces/     # Custom .action and .msg definitions
    │   ├── action/CollectSediment.action
    │   └── msg/{SamplerCommand.msg, SamplerTelemetry.msg}
    └── eyerov_sediment_collector/      # C++ Lifecycle Action Server & Mock Simulator
        ├── include/eyerov_sediment_collector/sediment_collector_node.hpp
        ├── src/{main.cpp, sediment_collector_node.cpp, mock_rov_hardware_node.cpp}
        └── launch/sediment_demo.launch.py
```

---

## Prerequisites & Build Instructions
* **OS / Middleware:** Ubuntu 22.04 with **ROS 2 Humble** (`rclcpp`, `rclcpp_lifecycle`, `rclcpp_action`, `lifecycle_msgs`, `ros2launch`).

```bash
cd rov_sediment_collection_assignment
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## Demonstration & Test Scenarios

### Scenario 1: Nominal End-to-End Sediment Collection
**Terminal 1** — Launch the mock ROV hardware and auto-activate the `sediment_collector_node` lifecycle node:
```bash
source install/setup.bash
ros2 launch eyerov_sediment_collector sediment_demo.launch.py
```

**Terminal 2** — Send a `CollectSediment` action goal with live feedback:
```bash
source install/setup.bash
ros2 action send_goal /collect_sediment eyerov_sediment_interfaces/action/CollectSediment \
  "{target_penetration_cm: 15.0, max_torque_nm: 25.0, timeout_sec: 30.0}" --feedback
```
**Expected Outcome:** Executes `PRE_CHECK` -> `DEPLOYING_ARM` -> `PENETRATING_SEABED` -> `CLOSING_SCOOP` -> `RETRACTING_ARM` -> `STOWING_SAMPLE`, returning `success: true` and `error_code: 0 (ERR_NONE)`.

---

### Scenario 2: Fail-Safe — Rock / Hard Obstruction (`ERR_OVER_TORQUE = 2`)
Simulates hitting bedrock at `5.0 cm` depth (`38.5 Nm > 25.0 Nm` threshold):
```bash
# Terminal 1
ros2 launch eyerov_sediment_collector sediment_demo.launch.py inject_rock_obstruction:=true

# Terminal 2 (send same goal)
ros2 action send_goal /collect_sediment eyerov_sediment_interfaces/action/CollectSediment \
  "{target_penetration_cm: 15.0, max_torque_nm: 25.0, timeout_sec: 30.0}" --feedback
```
**Expected Outcome:** Halts downward drive immediately, transitions to `SAFE_ABORT_RETRACT` to pull the manipulator clear of the seabed, and aborts with `error_code: 2`.

---

### Scenario 3: Fail-Safe — Mid-Operation ROV Drift (`ERR_STATION_KEEPING_LOST = 3`)
Launch normally, start a goal, and inject a station-keeping drift fault mid-sequence from a third terminal:
```bash
ros2 param set /mock_rov_hardware_node inject_drift_fault true
```
**Expected Outcome:** Immediately halts arm motion to prevent structural damage against the seabed, executes `SAFE_ABORT_RETRACT`, and aborts with `error_code: 3`.

---

### Scenario 4: Fail-Safe — Telemetry Watchdog Timeout (`ERR_SENSOR_TIMEOUT = 4`)
Simulate a sensor/tether telemetry freeze (`> 500 ms` silence):
```bash
ros2 param set /mock_rov_hardware_node inject_sensor_freeze true
```
**Expected Outcome:** 20 Hz watchdog timer detects stale telemetry, commands safe retraction, and aborts with `error_code: 4`.

---

### Scenario 5: Operator Cancellation (`ERR_USER_CANCELED = 6`) & E-Stop (`ERR_ESTOP = 7`)
* **Cancel:** Press `Ctrl + C` in Terminal 2 while running `--feedback` (or send a cancel request); the node enters `SAFE_ABORT_RETRACT` before transitioning to `CANCELED`.
* **E-Stop:** Run `ros2 param set /mock_rov_hardware_node inject_estop true` mid-run; triggers immediate `SAFE_ABORT_RETRACT` and aborts with `error_code: 7`.

---

### Scenario 6: Lifecycle State Management (`ERR_NODE_DEACTIVATED = 8`)
Verify ROS 2 managed lifecycle behavior via CLI:
```bash
ros2 lifecycle get /sediment_collector_node
ros2 lifecycle set /sediment_collector_node deactivate
# Attempting to send an action goal while INACTIVE will be cleanly rejected.
ros2 lifecycle set /sediment_collector_node activate
```

---

### Automated Verification Script
To automatically run both the nominal collection test and the rock-obstruction fail-safe test back-to-back:
```bash
chmod +x scripts/verify_all.sh
./scripts/verify_all.sh
```