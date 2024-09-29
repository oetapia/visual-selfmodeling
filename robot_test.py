import pybullet as p
import pybullet_data
import time
import math

# Connect to PyBullet
physicsClient = p.connect(p.GUI)  # Use p.DIRECT for non-GUI mode

# Set the additional search path for URDF files
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set the gravity
p.setGravity(0, 0, -9.81)

# Load a ground plane
plane_id = p.loadURDF("plane.urdf", basePosition=[0, 0, 0])

# Load your robot
robot_id = p.loadURDF("assets/simplified/quadruped.urdf", basePosition=[0, 0, 0.5])  # Update this line

""" # Function to get camera data
def get_camera_data():
    view_matrix = p.computeViewMatrix(cameraEyePosition=[0, 0, 1],
                                       cameraTargetPosition=[0, 0, 0],
                                       cameraUpVector=[0, 1, 0])
    projection_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                     aspect=1,
                                                     nearVal=0.1,
                                                     farVal=10.0)
    img = p.getCameraImage(width=640, height=480, viewMatrix=view_matrix, projectionMatrix=projection_matrix)
    return img[2]  # RGB image """

# Function to set joint positions
def set_joint_positions(joint_positions):
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=joint_positions[i])

# Define the TSDF volume and integrate data
""" class TSDFVolume:
    def __init__(self, bounds, voxel_size):
        self.bounds = bounds
        self.voxel_size = voxel_size
        # Initialize volume here

    def integrate(self, color_image, depth_image):
        # Integrate color and depth into the TSDF volume
        pass """

# Inside the loop, after capturing RGB-D data
#tsdf_volume = TSDFVolume(bounds=[[-1, 1], [-1, 1], [0, 2]], voxel_size=0.01)

# Gait Parameters
time_step = 1/240
t = 0
amplitude = 0.5   # Adjust the amplitude of leg movements
frequency = 1.0   # Speed of walking motion
phase_shift = math.pi  # Phase shift between diagonal legs


# Main loop to control the robot
while True:
    # Trot gait for quadruped (simple sine wave for leg movement)
    front_left_leg = amplitude * math.sin(frequency * t)
    back_right_leg = amplitude * math.sin(frequency * t)
    
    front_right_leg = amplitude * math.sin(frequency * t + phase_shift)
    back_left_leg = amplitude * math.sin(frequency * t + phase_shift)
    
    # Assign the joint positions (adjust indices based on your URDF)
    joint_positions = [
        front_left_leg,   # Front Left Leg Joint
        front_right_leg,  # Front Right Leg Joint
        back_left_leg,    # Back Left Leg Joint
        back_right_leg    # Back Right Leg Joint
    ]

    

    # Set joint positions for all legs
    set_joint_positions(joint_positions)

    # Update time for sine wave
    t += time_step

    # Get camera data
    #rgb_image = get_camera_data()

    # Step the simulation
    p.stepSimulation()
    time.sleep(time_step)

p.disconnect()
