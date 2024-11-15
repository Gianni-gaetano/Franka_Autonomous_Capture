#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
import time

robot = input('Real(r/[R]) or Simulation(s/[S]):')

def wait_for_franka_state():
    while '/franka_state_controller/franka_states' not in [topic[0] for topic in rospy.get_published_topics()]:
        rospy.loginfo("Waiting for /franka_state_controller/franka_states to be available...")
        time.sleep(1)

rospy.init_node('move_x_direction_test')
wait_for_franka_state()  # Wait for the topic to appear
rospy.loginfo("/franka_state_controller/franka_states is now available.")

def move_in_x_direction():
    """
    Move the robot end-effector 25 cm (.25 meter) along the x-axis.
    """
    rospy.init_node('move_x_direction_test')

    # Wait for the current pose topic to be available
    rospy.wait_for_message('/franka_state_controller/franka_states', FrankaState)

    # Get the current pose of the end effector
    def get_current_pose():
        current_state = rospy.wait_for_message('/franka_state_controller/franka_states', FrankaState)
        return current_state.O_T_EE  # Expecting this to be a 4x4 transformation matrix in flat list form (16 elements)

    current_pose = get_current_pose()

    # Verify we have the expected pose format
    if len(current_pose) != 16:
        rospy.logerr("Unexpected current_pose length. Expected 16 elements, got {}".format(len(current_pose)))
        return

    # Define target pose by moving 100 cm (1.0 m) in the x direction
    target_pose = PoseStamped()
    target_pose.header.frame_id = "fr3_link0"  # Assuming the base frame is "fr3_link0"

    # Set position
    target_pose.pose.position.x = current_pose[12] + 0.25  # Move 1 meter in x direction
    target_pose.pose.position.y = current_pose[13]
    target_pose.pose.position.z = current_pose[14]

    # Set orientation
    target_pose.pose.orientation.x = current_pose[4]
    target_pose.pose.orientation.y = current_pose[5]
    target_pose.pose.orientation.z = current_pose[6]
    target_pose.pose.orientation.w = current_pose[7]

    # Debug log the target pose
    rospy.loginfo("Target Pose: Position: x: {}, y: {}, z: {} | Orientation: x: {}, y: {}, z: {}, w: {}".format(
       target_pose.pose.position.x, 
       target_pose.pose.position.y, 
       target_pose.pose.position.z,
       target_pose.pose.orientation.x,
       target_pose.pose.orientation.y,
       target_pose.pose.orientation.z,
       target_pose.pose.orientation.w
    ))

    # Set up the publisher to the desired topic for sending pose commands
    pub = rospy.Publisher('/cartesian_pose_controller/pose', PoseStamped, queue_size=10)
    rospy.sleep(1)  # Wait for the publisher to establish connection

    rospy.loginfo("Moving robot 25 cm in the x direction...")
    rate = rospy.Rate(10)  # 10 Hz

    # Publish the target pose for a short duration to ensure the robot moves
    for _ in range(20):  # Publish for 2 seconds
        target_pose.header.stamp = rospy.Time.now()
        pub.publish(target_pose)
        rate.sleep()

    rospy.loginfo("Movement complete.")

if __name__ == '__main__':
    try:
        move_in_x_direction()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion.")

