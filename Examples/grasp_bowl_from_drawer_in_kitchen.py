"""
@Description :   This script shows how to grasp a bowl from a drawer, consider collision detection
@Author      :   Yan Ding 
@Time        :   2023/09/01 07:47:46
"""

import os
from RoboticsToolBox import Bestman, Pose
from Env import Client
from Visualization import Visualizer
from Motion_Planning.Manipulation import OMPL_Planner
from Motion_Planning.Navigation import *
from Utils import load_config


def main():
    
    # Load config
    config_path = '../Config/grasp_bowl_from_drawer_in_kitchen.yaml'
    cfg = load_config(config_path)
    print(cfg)

    # Init client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)

    # Load scene
    scene_path = '../Asset/Scene/Kitchen.json'
    client.create_scene(scene_path)

    # logID = pb_client.start_record("example_manipulation")    # start recording
    # Init robot
    bestman = Bestman(client, visualizer, cfg)
    
    # open the drawer
    client.change_object_joint_angle("elementA", 36, 0.4)

    visualizer.draw_aabb_link("elementA", 36)
    
    # navigate to standing position
    standing_pose = Pose([2.85, 2.4, 0], [0.0, 0.0, 0.0])
    # nav_planner = AStarPlanner(
    #     robot_size = bestman.get_robot_size(), 
    #     obstacles_bounds = client.get_Nav_obstacles_bounds(), 
    #     resolution = 0.05, 
    #     enable_plot = False
    # )
    nav_planner = RRTPlanner(
        robot_size = bestman.get_robot_size(), 
        obstacles_bounds = client.get_Nav_obstacles_bounds(), 
        enable_plot=False
    )
    path = nav_planner.plan(bestman.get_current_base_pose(), standing_pose)
    bestman.navigate_base(standing_pose, path, visualize = True)

    ompl_planner = OMPL_Planner(
        bestman,
        cfg.Planner
    )

    # get obstacles info
    ompl_planner.get_obstacles_info()

    # load bowl
    bowl_id = client.load_object(
        "../Asset/URDF_models/utensil_bowl_blue/model.urdf",
        [3.6, 2.4, 0.6],
        [0.0, 0.0, 0.0],
        1.0,
        "bowl",
        False
    )

    # set target object for grasping
    ompl_planner.set_target(bowl_id)

    # reach target object
    ompl_planner.plan_execute()

    # grasp target object
    bestman.active_gripper(bowl_id, 1)

    # disconnect pybullet
    client.wait(1000)
    client.disconnect()


if __name__=='__main__':
    
    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    main()