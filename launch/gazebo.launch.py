import os
from pathlib import Path
import json
import uuid

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.events import Shutdown
import xacro

def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    model = Path(value('model_file')).expanduser().resolve()
    world = Path(value('world_file')).expanduser().resolve()
    if not model.is_file() or not world.is_file():
        raise ValueError('model_file and world_file must point to existing files')

    robot_xml = xacro.process_file(str(model)).toxml()
    paths = [value('resource_paths'), os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    environment = {'GZ_SIM_RESOURCE_PATH': os.pathsep.join(p for p in paths if p)}
    plugin_paths = [str(Path(get_package_prefix('advrf_ros2_gazebo')) / 'lib'),
                    os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')]
    environment['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.pathsep.join(p for p in plugin_paths if p)
    # Keep this server and its spawn request separate from other Gazebo sessions.
    environment['GZ_PARTITION'] = 'advrf-' + uuid.uuid4().hex
    request = f'sdf: {json.dumps(robot_xml)}, name: {json.dumps(value("robot_name"))}'
    return [
        ExecuteProcess(
            cmd=['gz', 'sim', '-s', '-v', '3', str(world)],
            additional_env=environment, output='screen',
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo server exited'))]),
        ExecuteProcess(
            cmd=['gz', 'sim', '-g', '-v', '3'],
            additional_env=environment, output='screen',
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo GUI closed'))]),
        ExecuteProcess(
            cmd=['gz', 'service', '-s', f'/world/{value("world_name")}/create',
                 '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '60000', '--req', request],
            additional_env=environment, output='screen'
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('advrf_ros2_gazebo'))
    return LaunchDescription([
        DeclareLaunchArgument('model_file', description='Robot URDF or Xacro file'),
        DeclareLaunchArgument('world_file', default_value=str(share / 'worlds/empty.sdf')),
        DeclareLaunchArgument('world_name', default_value='empty', description='World name declared inside the SDF'),
        DeclareLaunchArgument('robot_name', default_value='robot'),
        DeclareLaunchArgument('resource_paths', default_value='', description='Colon-separated Gazebo resource directories'),
        OpaqueFunction(function=setup),
    ])
