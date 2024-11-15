#!/usr/bin/env python

import rospy
import csv
import math
import os
from sensor_msgs.msg import JointState
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64MultiArray
from datetime import datetime

# Flag to indicate whether the robot is moving
is_moving = False

# List to store data before writing to CSV
data_to_write = []

# Directory and filename setup
base_dir = os.path.dirname(os.path.abspath(__file__))
data_folder = os.path.join(base_dir, "RobotData")
os.makedirs(data_folder, exist_ok=True)  # Create RobotData folder if it doesn't exist
file_name = os.path.join(data_folder, input('Please Input desired file name (include .csv): '))

# Callback function to read joint states
def joint_state_callback(msg):
    global is_moving
    
    # Reading joint velocities to determine movement
    joint_vel = msg.velocity  # Get joint velocities

   #print(f"Joint Velocities: {joint_vel}")  # Debugging output

    # Update is_moving based on joint velocities
    is_moving = any(abs(vel) > 0.015 for vel in joint_vel)  # Check if any joint velocity is above threshold
    
   #if is_moving:
       #print("Robot is moving")
   #else:
       #print("Robot is stationary")

def franka_state_callback(msg):
    global is_moving
    
    # Get the joint velocity
    Theta_dot = msg.dq

    # Update `is_moving` based on joint velocities to capture stop and start changes
    movement_detected = any(abs(vel) > 0.05 for vel in Theta_dot)
    
    if movement_detected or not data_to_write:
        is_moving = True
    else:
        is_moving = False
    
    
    if is_moving or movement_detected:
        # Extract Joint Angle
        Theta = msg.q
        
        # Extract Joint Torque
        Eff = msg.tau_J
        
        # Separate into individual joint variables for positions, velocities, and torques
        j1, j2, j3, j4, j5, j6, j7 = Theta
        j1_dot, j2_dot, j3_dot, j4_dot, j5_dot, j6_dot, j7_dot = Theta_dot
        j1_tau, j2_tau, j3_tau, j4_tau, j5_tau, j6_tau, j7_tau = Eff
        
        # Extract end-effector position
        ee_position = msg.O_T_EE[12:15]  # Extract x, y, z from transformation matrix
        x, y, z = ee_position
        
        # Extract rotation matrix elements from the transformation matrix
        r11, r12, r13 = msg.O_T_EE[0:3]
        r21, r22, r23 = msg.O_T_EE[4:7]
        r31, r32, r33 = msg.O_T_EE[8:11]
        
        # Calculate roll, pitch, and yaw (in radians)
        roll = math.atan2(r32, r33)  # Rotation about x-axis
        pitch = -math.asin(r31)      # Rotation about y-axis
        yaw = math.atan2(r21, r11)   # Rotation about z-axis

        # Convert radians to degrees if desired (optional)
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        # Store data in WPR format
        store_data(
            j1=j1, j2=j2, j3=j3, j4=j4, j5=j5, j6=j6, j7=j7,
            j1_dot=j1_dot, j2_dot=j2_dot, j3_dot=j3_dot, j4_dot=j4_dot, j5_dot=j5_dot, j6_dot=j6_dot, j7_dot=j7_dot,
            j1_tau=j1_tau, j2_tau=j2_tau, j3_tau=j3_tau, j4_tau=j4_tau, j5_tau=j5_tau, j6_tau=j6_tau, j7_tau=j7_tau,
            x=x, y=y, z=z,
            w=roll_deg, p=pitch_deg, r=yaw_deg
        )

# Function to store data in a list
def store_data(j1='None', j2='None', j3='None', j4='None', j5='None', j6='None', j7='None',
               j1_dot='None', j2_dot='None', j3_dot='None', j4_dot='None', j5_dot='None', j6_dot='None', j7_dot='None',
               j1_tau='None', j2_tau='None', j3_tau='None', j4_tau='None', j5_tau='None', j6_tau='None', j7_tau='None',
               x='None', y='None', z='None',
               w='None', p='None', r='None'):
    
    # Get the current timestamp
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
    
    # Store the data in memory as a list of dictionaries
    data_to_write.append([
        timestamp,
        j1, j2, j3, j4, j5, j6, j7,
        j1_dot, j2_dot, j3_dot, j4_dot, j5_dot, j6_dot, j7_dot,
        j1_tau, j2_tau, j3_tau, j4_tau, j5_tau, j6_tau, j7_tau,
        x, y, z, w, p, r
    ])
    print(f"Data stored at {timestamp}")  # Debugging output

# Function to write all stored data to a CSV file
def write_data_to_csv():
    with open(file_name, mode='w') as csv_file:
        csv_writer = csv.writer(csv_file)
        
        # Write header
        csv_writer.writerow([
            'Timestamp', 'Theta (J1)', 'Theta (J2)', 'Theta (J3)', 'Theta (J4)',
            'Theta (J5)', 'Theta (J6)', 'Theta (J7)', 'Theta_dot (J1)', 'Theta_dot (J2)',
            'Theta_dot (J3)', 'Theta_dot (J4)', 'Theta_dot (J5)', 'Theta_dot (J6)',
            'Theta_dot (J7)', 'Joint Efforts (J1)', 'Joint Efforts (J2)', 'Joint Efforts (J3)',
            'Joint Efforts (J4)', 'Joint Efforts (J5)', 'Joint Efforts (J6)', 
            'Joint Efforts (J7)', 'Cartesian Position (x)', 'Cartesian Position (y)',
            'Cartesian Position (z)', 'Orientation (w)', 'Orientation (p)', 'Orientation (r)'
        ])
        # Write all the stored data
        csv_writer.writerows(data_to_write)
        print("Data written to CSV " )  # Debugging output

def listener():
    rospy.init_node('franka_data_listener', anonymous=True)

    # Subscribe to the relevant topics
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, franka_state_callback)

    # Listen until program is terminated - only stores data if robot is moving
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    finally:
        # When the program exits, write all the stored data to CSV
        write_data_to_csv()



    

