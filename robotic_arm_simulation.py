import pybullet as p
import time
import pybullet_data

# Initialize the simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the plane and robot URDF
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

# Set gravity
p.setGravity(0, 0, -9.81)

# Get the number of joints and print information
num_joints = p.getNumJoints(robot_id)
for joint in range(num_joints):
    joint_info = p.getJointInfo(robot_id, joint)
    print(joint_info)

# Define target positions for the joints
target_positions = [0, -0.5, 0, -1.5, 0, 1.0, 0.5]

# Move the robot to the target positions
for i, target_position in enumerate(target_positions):
    p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=target_position)

# Simulate for some time to see the movement
for _ in range(240):
    p.stepSimulation()
    time.sleep(1./240.)

# Disconnect from the simulation
p.disconnect()
