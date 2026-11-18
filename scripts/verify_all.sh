#!/usr/bin/env bash
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

cleanup() {
  pkill -f "sediment_collector_node" 2>/dev/null || true
  pkill -f "mock_rov_hardware_node" 2>/dev/null || true
  sleep 1
}
trap cleanup EXIT

cleanup

echo "=== 1. Testing Normal Sediment Collection (Expected: SUCCEEDED, error_code=0) ==="
ros2 launch eyerov_sediment_collector sediment_demo.launch.py > /tmp/eyerov_test1.log 2>&1 &
sleep 3

ros2 action send_goal /collect_sediment eyerov_sediment_interfaces/action/CollectSediment \
  "{target_penetration_cm: 12.0, max_torque_nm: 25.0, timeout_sec: 30.0}"

cleanup

echo ""
echo "=== 2. Testing Fail-Safe Rock Obstruction (Expected: ABORTED, error_code=2) ==="
ros2 launch eyerov_sediment_collector sediment_demo.launch.py inject_rock_obstruction:=true > /tmp/eyerov_test2.log 2>&1 &
sleep 3

ros2 action send_goal /collect_sediment eyerov_sediment_interfaces/action/CollectSediment \
  "{target_penetration_cm: 12.0, max_torque_nm: 25.0, timeout_sec: 30.0}" || true

cleanup

echo ""
echo "=== Verification Complete ==="
