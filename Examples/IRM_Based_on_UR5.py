import os
import numpy as np
from Config import load_config
from Env import Client
from RoboticsToolBox import Bestman_sim_ur5e_vacuum_long, Pose
from Visualization import Visualizer
from Motion_Planning.Manipulation.OMPL_Planner import OMPL_Planner
import csv

def generate_workspace_points(x_range, y_range, z, step):
    """Generate the grid points in the workspace."""
    x_points = [x_range]
    y_points = [y_range]
    z_points = [z]  # Fixed z-axis height
    return np.array(np.meshgrid(x_points, y_points, z_points)).T.reshape(-1, 3)


def generate_orientations():
    """Generate four different orientations (euler angles)."""
    orientations = [
        [0, 0, 0],  # Forward
        [0, np.pi / 2, 0],  # Left
        [0, -np.pi / 2, 0],  # Right
        [np.pi, 0, 0],  # Backward
    ]
    return orientations


def check_reachability_for_point(bestman, planner_class, cfg, point, orientations):
    """Check if the robot can reach the point with four orientations."""
    reachability = []
    for orientation in orientations:
        # 每次规划前重新初始化规划器，避免重复之前的动作
        bestman.sim_set_arm_to_joint_values([0, -1.57, 1.5, -1.57, -1.57, 0])

        goal_pose = Pose(point, orientation)
        planner = planner_class(bestman, cfg.Planner)
        goal = planner.set_target_pose(goal_pose)
        start = bestman.sim_get_current_joint_values()
        path = planner.plan(start, goal)

        if path is not None:
            result = bestman.sim_execute_trajectory(path, goal_pose, False)
            reachability.append(result)
        else:
            reachability.append(False)
    return reachability

def save_reachability_matrix_to_csv(reachability_matrix, filename="reachability_matrix.csv"):
    """Save the reachability matrix to a CSV file."""
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Write header
        writer.writerow(["x", "y", "z", "Orientation 1", "Orientation 2", "Orientation 3", "Orientation 4"])
        # Write each point and its reachability
        for point, reachability in reachability_matrix:
            writer.writerow([*point, *reachability])

def main(filename):
    # Load config
    config_path = "../Config/IRM.yaml"
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)

    # Init robot
    bestman = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)


    # Define workspace parameters
    x_range = -0.2
    y_range = 0
    z_height = 1.0
    step = 0.1

    # Generate workspace points
    workspace_points = generate_workspace_points(x_range, y_range, z_height, step)

    # Generate four orientations for the end-effector
    orientations = generate_orientations()

    # Initialize the reachability matrix (each point has four orientations)
    reachability_matrix = []

    # Loop over all points in the workspace and check reachability
    for point in workspace_points:
        reachability = check_reachability_for_point(bestman, OMPL_Planner, cfg, point, orientations)
        reachability_matrix.append((point, reachability))

    # Save reachability matrix to CSV file
    save_reachability_matrix_to_csv(reachability_matrix, filename="reachability_matrix.csv")
    # Print or save reachability matrix for analysis
    print("Reachability matrix calculated.")

    # Disconnect pybullet
    client.wait(5)
    client.disconnect()


if __name__ == "__main__":
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # get current file name
    filename = os.path.splitext(os.path.basename(__file__))[0]

    main(filename)
