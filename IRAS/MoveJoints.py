from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
import rospy

# Initialize the ROS node
rospy.init_node('joint_trajectory_client')

robot = input('Real(r/[R]) or Simulation(s/[S]):')

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

# Create the trajectory message
traj_msg = JointTrajectory()
traj_msg.joint_names = [
    "fr3_joint1", "fr3_joint2", "fr3_joint3", 
    "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"
]

# Define the trajectory points (for all joints at once)
point = JointTrajectoryPoint()
point.positions = [0, -0.784219, 0, -2.35602, 0, 1.57103, 0.786879]  # *Here is where you define the join postions*
point.time_from_start = rospy.Duration(2.0)  # Adjust the time as needed

traj_msg.points.append(point)

# Create a goal
goal = FollowJointTrajectoryGoal()
goal.trajectory = traj_msg

# Send the goal to the action server
client.send_goal(goal)

# Wait for the result (optional)
client.wait_for_result()

#initial_joint_states = [0, -0.784219, 0, -2.35602, 0, 1.57103, 0.786879]

