from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
import launch_testing
from ament_index_python.packages import get_package_share_directory

simple_launch_examples = get_package_share_directory('simple_launch') + '/example'


def generate_test_description():

    return LaunchDescription([
        IncludeLaunchDescription(simple_launch_examples + '/condition_launch.py'),
        launch_testing.actions.ReadyToTest()])
