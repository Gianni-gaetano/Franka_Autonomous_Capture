#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
import time

def move_franka_x(distance):
    # Initialize the ROS node
    rospy.init_node('franka_move_x', anonymous=True)
    
    # Create a publisher to the franka state or pose command
    pub = rospy.Publisher('/franka/pose_command', PoseStamped, queue_size=10)
    
    # Wait for the publisher to be ready
    rospy.sleep(1)
    
    # Define the current pose (assuming you already know the current position)
    current_pose = PoseStamped()
    
    # Populate the current pose with known values (example)
    # You can subscribe to /franka_state_controller/pose or get it from another source
    current_pose.pose.position.x = 0.0  # Replace with actual x-coordinate
    current_pose.pose.position.y = 0.0  # Replace with actual y-coordinate
    current_pose.pose.position.z = 0.0  # Replace with actual z-coordinate
    current_pose.pose.orientation.w = 1.0  # Assuming no rotation, set appropriately
    
    # Create the new pose by moving in the x direction by the specified distance
    new_pose = current_pose
    new_pose.pose.position.x += distance
    
    # Publish the new pose to move the robot
    pub.publish(new_pose)
    
    # Wait a moment for the robot to move
    rospy.sleep(3)  # Adjust this sleep time as needed for your robot's response time
    
    rospy.loginfo("Moved the robot in the x direction.")

if __name__ == '__main__':
    try:
        # Move the robot in the x direction
        move_franka_x(3)  # 0.05 meters = 5 cm
    except rospy.ROSInterruptException:
        pass

