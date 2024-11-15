#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def send_pose_command():
    # Initialize the ROS node
    rospy.init_node('send_pose_command', anonymous=True)
    
    # Create a publisher for the pose command topic
    pub = rospy.Publisher('/franka/arm/pose_command', PoseStamped, queue_size=10)

    # Wait for the publisher to connect
    rospy.sleep(1)

    # Define a new pose command (set desired position and orientation)
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "world"  # or another frame like "base_link"
    
    # Define position (x, y, z) and orientation (quaternion)
    pose.pose.position.x = 0.5  # 50 cm along the x-axis
    pose.pose.position.y = 0.0  # No movement along y-axis
    pose.pose.position.z = 0.5  # 50 cm above the base
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0  # No rotation

    # Publish the pose command
    rospy.loginfo(f"Sending pose command: {pose}")
    pub.publish(pose)

if __name__ == '__main__':
    try:
        send_pose_command()
    except rospy.ROSInterruptException:
        pass

