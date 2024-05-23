from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                namespace= "ros_service" , package= "ros_package" ,
                executable= "Robot_Service" , output = "screen"),
            Node(
                namespace= "ros_topic" , package= "ros_package" ,
                executable= "Robot_Topic" , output = "screen"),
            Node(
                namespace= "ros_driver" , package= "ros_package" ,
                executable= "Robot_Driving" , output = "screen"),
        ]
    )
