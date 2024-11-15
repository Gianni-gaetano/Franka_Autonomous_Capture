import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import actionlib

robot = input('Real(r/[R]) or Simulation(s/[S]):')

def move_joint(joint_index, positions, initial_states, duration=2.0, sleep_time=1.0):
    """
    Send a trajectory to move a single joint to each position in the list,
    keeping other joints at their initial states.
    """
    # Choose the appropriate action server topic
    if robot == 'r' or robot == 'R':
        action_server = '/position_joint_trajectory_controller/follow_joint_trajectory'
    elif robot == 's' or robot == 'S':
        action_server = '/effort_joint_trajectory_controller/follow_joint_trajectory'
    
    # Initialize the action client
    client = actionlib.SimpleActionClient(action_server, FollowJointTrajectoryAction)

    rospy.loginfo(f"Waiting for the action server at {action_server}...")
    client.wait_for_server()  # Wait for the action server to be ready

    rospy.loginfo(f"Connected to {action_server}.")

    # Initialize time from start
    time_from_start = 0.0

    for position in positions:
        # Create a new trajectory message for each position
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            "fr3_joint1", "fr3_joint2", "fr3_joint3", 
            "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"
        ]
        
        point = JointTrajectoryPoint()
        point.positions = initial_states[:]  # Copy initial states for other joints
        point.positions[joint_index] = position  # Update the joint being tested
        time_from_start += duration  # Increment time for each point
        point.time_from_start = rospy.Duration(time_from_start)
        
        traj_msg.points.append(point)

        # Create the action goal and send it
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj_msg

        rospy.loginfo(f"Sending goal for joint {joint_index + 1} to position {position}")
        client.send_goal(goal)

        # Wait for the action to complete
        client.wait_for_result()

        rospy.loginfo(f"Movement complete for joint {joint_index + 1}")
        rospy.sleep(sleep_time)
        
"""        
  
    positions = [
        initial_states[0], initial_states[1], initial_states[2], 
        initial_states[3], initial_states[4], initial_states[5], initail_states[6]
    ]

"""

def move_specified_joint(initial_states):
    Joint_wanted = input("What joint would you like to move? Joint1 = 0, Joint2 = 1.... Joint7 = 6: ")
    position_wanted = input("Where would you like to move the joint? ")

    Joint_wanted = int(Joint_wanted)  # Make sure Joint_wanted is an integer
    position_wanted = float(position_wanted)  # Convert the position to a float

    rospy.loginfo(f"Moving joint {Joint_wanted} to {position_wanted}")
    
    # Now, call move_joint with position_wanted in a list
    move_joint(Joint_wanted, [position_wanted], initial_states)




if __name__ == '__main__':
    rospy.init_node('joint_movement_test')

    # Assuming some initial joint states
    initial_joint_states = [0, -0.784219, 0, -2.35602, 0, 1.57103, 0.786879]
    
    try:
        move_specified_joint(initial_joint_states)
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion.")

