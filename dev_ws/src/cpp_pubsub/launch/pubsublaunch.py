from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cpp_pubsub",
            executable="minimal_publisher",
            name="publisher"
            #output="screen",
            #emulate_tty=True,
            #parameters=[
            #    {"my_parameter": "earth"}
            #]
        ), 
        Node (
            package="cpp_pubsub",
            executable="minimal_subscriber",
            name="subscriber"
        )
    ])
