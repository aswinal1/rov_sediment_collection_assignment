import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    rock_arg = DeclareLaunchArgument("inject_rock_obstruction", default_value="false")
    drift_arg = DeclareLaunchArgument("inject_drift_fault", default_value="false")
    freeze_arg = DeclareLaunchArgument("inject_sensor_freeze", default_value="false")
    estop_arg = DeclareLaunchArgument("inject_estop", default_value="false")
    hw_fault_arg = DeclareLaunchArgument("inject_hardware_fault", default_value="false")
    alt_arg = DeclareLaunchArgument("simulated_altitude_m", default_value="0.50")

    mock_hw_node = Node(
        package="eyerov_sediment_collector",
        executable="mock_rov_hardware_node",
        name="mock_rov_hardware_node",
        output="screen",
        parameters=[{
            "inject_rock_obstruction": LaunchConfiguration("inject_rock_obstruction"),
            "inject_drift_fault": LaunchConfiguration("inject_drift_fault"),
            "inject_sensor_freeze": LaunchConfiguration("inject_sensor_freeze"),
            "inject_estop": LaunchConfiguration("inject_estop"),
            "inject_hardware_fault": LaunchConfiguration("inject_hardware_fault"),
            "simulated_altitude_m": LaunchConfiguration("simulated_altitude_m"),
        }],
    )

    collector_node = LifecycleNode(
        package="eyerov_sediment_collector",
        executable="sediment_collector_node",
        name="sediment_collector_node",
        namespace="",
        output="screen",
    )

    # Automatically trigger CONFIGURE when node starts
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(collector_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # Automatically trigger ACTIVATE once node reaches INACTIVE state
    activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=collector_node,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(collector_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription([
        rock_arg,
        drift_arg,
        freeze_arg,
        estop_arg,
        hw_fault_arg,
        alt_arg,
        mock_hw_node,
        collector_node,
        configure_event,
        activate_handler,
    ])
