import pybullet as p
import pybullet_data
import time

# Connect to PyBullet
physicsClient = p.connect(p.GUI)  # Use p.DIRECT for non-GUI mode

# Set the additional search path for URDF files
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set the gravity
p.setGravity(0, 0, -9.81)

# Load a ground plane
p.loadURDF("plane.urdf", basePosition=[0, 0, 0])

# Load a simple robot (replace 'cube.urdf' with your robot's URDF)
robotId = p.loadURDF("cube.urdf", basePosition=[0, 0, 0.5])  # Position the robot above the plane

# Keep the simulation running
while True:  # Run indefinitely
    p.stepSimulation()
    time.sleep(1/240)  # Slow down the simulation for visualization

# Disconnect from PyBullet (this won't be reached until you close the window)
p.disconnect()
