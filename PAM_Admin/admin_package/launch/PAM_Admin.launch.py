from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                namespace= "" , package= "admin_package" ,
                executable= "Admin_manager" , output = "screen"),
            Node(
                namespace= "" , package= "admin_package" ,
                executable= "gui" , output = "screen"),
            Node(
                namespace= "" , package= "admin_package" ,
                executable= "msg" , output = "screen"),
        ]
    )
