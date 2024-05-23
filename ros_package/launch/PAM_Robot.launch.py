from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                namespace= "" , package= "ros_package" ,
                executable= "Robot_Service" , output = "screen"),
            Node(
                namespace= "" , package= "ros_package" ,
                executable= "Robot_Topic" , output = "screen"),
            Node(
                namespace= "" , package= "ros_package" ,
                executable= "Robot_Driving" , output = "screen"),
            Node(
                namespace= "" , package= "ros_package" ,
                executable= "camera" , output = "screen"),
            Node(
                namespace= "voice" , package= "ros_package" ,
                executable= "voice_recognize" , output = "screen"),
        ]
    )
