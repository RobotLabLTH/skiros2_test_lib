import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    skiros_config = {
        "libraries_list": "['skiros2_test_lib']",
        "skill_list": "['test_skill_parallel', 'test_skill_sequence', 'test_skill_sequence_of_parallels', 'test_primitive', 'test_action_server']"
    }

    action_test_server = LaunchDescription([
        Node(
            package="skiros2_test_lib",
            name="test_action",
            executable="action_test_server"
        )
    ])

    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('skiros2'),
                'skiros2.launch.py')),
        launch_arguments=skiros_config.items(),
    )
     
    return LaunchDescription([launch_include, action_test_server])
