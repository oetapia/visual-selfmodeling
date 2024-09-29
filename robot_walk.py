import pybullet as p
import pybullet_data
import time
import math

# Function to load the robot based on its name
def load_robot(robot_name):
    if robot_name.lower() == "picrawler":
        robot_id = p.loadURDF("assets/simplified/picrawler.urdf", basePosition=[0, 0, 0.5])
    elif robot_name.lower() == "quadruped":
        robot_id = p.loadURDF("assets/simplified/quadruped.urdf", basePosition=[0, 0, 0.5])
    else:
        raise ValueError(f"Unknown robot name: {robot_name}")
    return robot_id

def set_joint_positions(robot_id, joint_positions):
    num_joints = p.getNumJoints(robot_id)
    #print(f"Number of Joints: {num_joints}")
    for i in range(min(len(joint_positions), num_joints)):  # Ensure it doesn't exceed the joint count
        p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=joint_positions[i])
        #joint_info = p.getJointInfo(robot_id, i)
        #print(f"Joint {i}: {joint_info}")


# Function for inverse kinematics (IK) to calculate joint angles
def calculate_ik(target_position):
    # Stub function - replace with real IK calculation if needed
    joint_angles = [0.0, 0.0, 0.0, 0.0]  # Replace with actual IK logic
    return joint_angles

# Initialize PyBullet
physicsClient = p.connect(p.GUI)  # Use p.DIRECT for non-GUI mode
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # URDF file search path
p.setGravity(0, 0, -9.81)

# Load ground plane
plane_id = p.loadURDF("plane.urdf", basePosition=[0, 0, 0])

# Select and load the robot (change this value to "picrawler" or "quadruped")
robot_name = "picrawler"  # or "quadruped"
robot_id = load_robot(robot_name)

# Gait parameters
time_step = 1 / 240
t = 0
amplitude = 0.5   # Adjust the amplitude of leg movements
frequency = 1.0   # Speed of walking motion
phase_shift = math.pi  # Phase shift between diagonal legs

# Define desired foot positions for each leg (replace with actual values)
desired_positions = {
    "front_left": [0.1, 0.2, 0.3],  # Replace with actual foot position (x, y, z)
    "front_right": [0.1, 0.2, 0.3],
    "back_left": [0.1, 0.2, 0.3],
    "back_right": [0.1, 0.2, 0.3],
}

# Main control loop
while True:
    # Example trot gait using sine waves for leg motion
    front_left_leg = amplitude * math.sin(frequency * t)
    back_right_leg = amplitude * math.sin(frequency * t)

    front_right_leg = amplitude * math.sin(frequency * t + phase_shift)
    back_left_leg = amplitude * math.sin(frequency * t + phase_shift)

    # Joint positions for the legs (adjust the indices according to URDF)
    joint_positions = [
        front_left_leg,   # Front Left Leg Joint
        front_right_leg,  # Front Right Leg Joint
        back_left_leg,    # Back Left Leg Joint
        back_right_leg    # Back Right Leg Joint
    ]

    # Apply the joint positions to the robot
    set_joint_positions(robot_id, joint_positions)


    # Increment time for the sine wave
    t += time_step

    # Step the simulation
    p.stepSimulation()
    time.sleep(time_step)

# Disconnect from PyBullet
p.disconnect()
