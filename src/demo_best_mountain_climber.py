import pybullet as p
import pybullet_data
import time
import os

# Import the MountainClimbingSimulation class from mountain_climbing_ga.py
from mountain_climbing_ga import MountainClimbingSimulation

# --- CONFIGURATION ---
ROBOT_URDF = "best_mountain_climber_20250624_172108.urdf"
ROBOT_URDF_PATH = os.path.join(os.path.dirname(__file__), ROBOT_URDF)
MOUNTAIN_TYPE = "default"  # Change to 'steep', 'ridge', or 'plateau' if desired
VIDEO_FILENAME = "climber_demo.mp4"
SIMULATION_TIME = 15  # seconds
START_POS = (0, -6, 1)  # (x, y, z) starting position


def main():
    # Set up the simulation environment with GUI
    sim = MountainClimbingSimulation(simulation_time=SIMULATION_TIME, mountain_type=MOUNTAIN_TYPE)
    sim.setup_environment(gui=True)

    # Start video recording
    log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, VIDEO_FILENAME)

    # Load the robot URDF
    if not os.path.exists(ROBOT_URDF_PATH):
        print(f"URDF file not found: {ROBOT_URDF_PATH}")
        return
    robot_id = p.loadURDF(ROBOT_URDF_PATH, START_POS)
    p.changeDynamics(robot_id, -1, linearDamping=0.05, angularDamping=0.05, lateralFriction=0.8)

    # Run the simulation
    print(f"Simulating for {SIMULATION_TIME} seconds...")
    for step in range(int(SIMULATION_TIME * 240)):
        p.stepSimulation()
        time.sleep(1/240)

    # Stop video recording
    p.stopStateLogging(log_id)
    print(f"Video saved as {VIDEO_FILENAME}")

    # Keep the window open for a bit
    print("Simulation complete. Close the PyBullet window to exit.")
    while p.isConnected():
        time.sleep(0.1)

if __name__ == "__main__":
    main() 