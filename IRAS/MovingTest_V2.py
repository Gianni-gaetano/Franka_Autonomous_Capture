import franka_interface

import rospy

from geometry_msgs.msg import Pose



def main():

    # Initialize the ROS node

    rospy.init_node('move_franka_x')



    # Create a Franka interface object

    franka = franka_interface.get_franka()



    # Get the current pose of the robot's end effector

    current_pose = franka.get_pose()



    # Define the movement distance (5 cm in meters)

    move_distance = 0.05  # 5 cm



    # Create a new pose for the end effector

    new_pose = Pose()

    new_pose.position.x = current_pose.position.x + move_distance  # Update the x position

    new_pose.position.y = current_pose.position.y  # Keep the same y position

    new_pose.position.z = current_pose.position.z  # Keep the same z position

    new_pose.orientation = current_pose.orientation  # Keep the same orientation



    # Move the robot to the new pose

    franka.move_to_pose(new_pose, duration=1.0)  # Move to the new pose over 1 second




