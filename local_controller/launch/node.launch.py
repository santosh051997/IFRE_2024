# launch/robot_control_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([ 
		Node(
            package='local_controller',
            executable='lane1',
            name='lane1'
        ),
        
        # Node(
        #     package='local_controller',
        #     executable='orientation1',
        #     name='orientation1'
        # ),
        
        Node(
            package='local_controller',
            executable='sequence1',
            name='sequence1'
        ),

        # Node(
        #     package='local_controller',
        #     executable='ransac1',
        #     name='ransac1'
        # ),

        # Node(
        #     package='local_controller',
        #     executable='orientation2',
        #     name='orientation2'
        # ),

        Node(
            package='local_controller',
            executable='sequence2',
            name='sequence2'
        ),
    ])
