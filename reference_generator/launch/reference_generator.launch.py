from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


def _topic_prefix(instance: str) -> str:
    inst = str(instance).strip()
    if not inst or inst == '0':
        return '/fmu'
    return f'/px4_{inst}/fmu'


def _load_ros_params(yaml_path: str) -> dict:
    if not yaml_path or not os.path.isfile(yaml_path):
        return {}
    if yaml is None:
        return {}
    with open(yaml_path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    if 'reference_generator' in data:
        return dict(data['reference_generator'].get('ros__parameters', {}) or {})
    if '/**' in data:
        return dict(data['/**'].get('ros__parameters', {}) or {})
    if 'ros__parameters' in data:
        return dict(data['ros__parameters'] or {})
    return {}


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('px4_reference_generator')
    fair_overlay = os.path.join(pkg_share, 'config', 'fair_compare_mission.yaml')
    spawn_overlay = os.path.join(pkg_share, 'config', 'spawn_relative_mission.yaml')

    overlay_env = os.environ.get('SITL_MISSION_OVERLAY', '').strip()
    if not overlay_env and os.environ.get('SITL_FAIR_COMPARE_MISSION', '') == '1':
        overlay_env = fair_overlay
    if not overlay_env and os.environ.get('SITL_SPAWN_RELATIVE_MISSION', '') == '1':
        overlay_env = spawn_overlay

    mission_overlay_cfg = LaunchConfiguration('mission_overlay').perform(context).strip()
    if mission_overlay_cfg:
        overlay_env = mission_overlay_cfg

    mission_config = LaunchConfiguration('mission_config').perform(context)
    params = _load_ros_params(mission_config)
    if overlay_env:
        params.update(_load_ros_params(overlay_env))

    inst = LaunchConfiguration('px4_instance').perform(context)
    prefix = _topic_prefix(inst)
    ros_ns = '' if inst in ('', '0') else f'ref_px4_{inst}'
    try:
        target_system_id = int(inst) + 1 if inst not in ('', '0') else 1
    except ValueError:
        target_system_id = 1

    params.update({
        'target_system_id': target_system_id,
        'topic.offboard_control_mode': f'{prefix}/in/offboard_control_mode',
        'topic.trajectory_setpoint': f'{prefix}/in/trajectory_setpoint',
        'topic.vehicle_command': f'{prefix}/in/vehicle_command',
        'topic.vehicle_status': f'{prefix}/out/vehicle_status',
        'topic.vehicle_local_position': f'{prefix}/out/vehicle_local_position',
    })

    node = Node(
        package='px4_reference_generator',
        executable='reference_generator_node',
        name='reference_generator',
        output='screen',
        parameters=[params],
    )
    if ros_ns:
        return [GroupAction([PushRosNamespace(ros_ns), node])]
    return [node]


def generate_launch_description():
    pkg_share = get_package_share_directory('px4_reference_generator')
    default_config = os.path.join(pkg_share, 'config', 'default_mission.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('mission_config', default_value=default_config),
        DeclareLaunchArgument('mission_overlay', default_value=''),
        DeclareLaunchArgument('px4_instance', default_value=os.environ.get('PX4_INSTANCE', '0')),
        OpaqueFunction(function=_launch_setup),
    ])
