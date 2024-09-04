# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : open_fridge.py
# @Time           : 2024-08-03 15:05:51
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : A example to open fridge use ur5e robot
"""


import math
import os

import numpy as np
import pybullet as p

from Config import load_config
from Env import Client
from Motion_Planning.Manipulation.OMPL_Planner import OMPL_Planner
from RoboticsToolBox import Bestman_sim_ur5e_vacuum_long, Pose
from Visualization import Visualizer


def rotate_point_3d_around_axis(init_pose, rotate_axis, theta, clockwise=True):
    """Rotate the point (x, y, z) around the rotation axis (a, b, c) by theta angle (radians).

    Parameters:
        init_pose -- the initial point's pose (including position and Euler angle pose)
        rotate_axis -- the origin coordinates of the rotation axis (a, b, c)
        theta -- the rotation angle (radians)

    Returns:
        Pose -- the pose of the rotated point (including position and Euler angle pose)
    """

    init_position = np.array(init_pose.get_position())
    rotate_axis = np.array(rotate_axis)

    translated_position = init_position - rotate_axis

    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    if clockwise:
        rotation_matrix = np.array(
            [[cos_theta, sin_theta, 0], [-sin_theta, cos_theta, 0], [0, 0, 1]]
        )
        rotated_quaternion = p.getQuaternionFromEuler([0, 0, -theta / 2])
        rotated_orientation = p.multiplyTransforms(
            [0, 0, 0],
            rotated_quaternion,
            [0, 0, 0],
            init_pose.get_orientation(),
        )[1]
    else:
        rotation_matrix = np.array(
            [[cos_theta, -sin_theta, 0], [sin_theta, cos_theta, 0], [0, 0, 1]]
        )
        rotated_quaternion = p.getQuaternionFromEuler([0, 0, theta / 2])
        rotated_orientation = p.multiplyTransforms(
            [0, 0, 0],
            rotated_quaternion,
            [0, 0, 0],
            init_pose.get_orientation(),
        )[1]

    rotated_position = np.dot(rotation_matrix, translated_position)
    final_position = rotated_position + rotate_axis
    rotated_euler_angles = p.getEulerFromQuaternion(rotated_orientation)
    return Pose(final_position, rotated_euler_angles)


def main(filename):

    # Load config Robot near to fridge
    config_path = "../Config/open_fridge.yaml"
    cfg = load_config(config_path)
    print(cfg)

    # Initial client and visualizer
    client = Client(cfg.Client)
    visualizer = Visualizer(client, cfg.Visualizer)
    visualizer.draw_axes()

    # Load scene
    scene_path = "../Asset/Scene/Kitchen.json"
    client.create_scene(scene_path)

    # Start recording
    visualizer.start_record(filename)

    # Init robot
    bestman = Bestman_sim_ur5e_vacuum_long(client, visualizer, cfg)

    # Init visualizer
    visualizer.change_robot_color(
        bestman.sim_get_base_id(), bestman.sim_get_arm_id(), False
    )

    object_details = []
    object_stacking = []
    ground_object = []
    all_box_range = []

    for obj_name_id in client.object_ids:
        obj_info = getattr(client, obj_name_id[1])
        object_id, name, position = obj_info[0], obj_info[1], obj_info[2]
        ground_truth = client.get_ground_truth(object_id)
        if ground_truth == True:
            ground_object.append(object_id)
        obj_range = client.get_bounding_box_xyz(obj_info[0])
        obj_special_range = client.get_bounding_box(obj_info[0])
        object_details.append((name, position, obj_range, ground_truth))
        object_stacking.append((name, position, obj_special_range, ground_truth))
        # obj_range代表了一个box的长宽高 obj_special_range表示一个box的六个角
    for item in ground_object:
        num_joints = p.getNumJoints(item)
        for i in range (-1, num_joints):
            range1 = visualizer.draw_aabb_range(item, i)
            all_box_range.append(range1)
    target_box_range = []
    ob_id = 6
    num_joints_target = p.getNumJoints(ob_id)
    for i in range(-1, num_joints_target):
        range2 = visualizer.draw_aabb_range(ob_id, i)
        target_box_range.append(range2)

    print("Fridge's detail 3D bounding box is", object_details)
    print("Fridge's detail 3D bounding box is", target_box_range)
    # # Draw fridge door handle
    # visualizer.draw_aabb_link("fridge", 2)
    #
    # # Init planner
    # ompl_planner = OMPL_Planner(bestman, cfg.Planner)
    #
    # # Get goal joint values
    # min_x, min_y, min_z, max_x, max_y, max_z = client.get_link_bounding_box("fridge", 2)
    # goal_pose = Pose(
    #     [
    #         min_x - bestman.sim_get_tcp_link_height() - 0.05,
    #         (min_y + max_y) / 2,
    #         (min_z + max_z) / 2,
    #     ],
    #     [0.0, 0.0, 0.0],
    # )
    # goal = ompl_planner.set_target_pose(goal_pose)
    #
    # # Plan / Execute / Suctate fridge handle
    # start = bestman.sim_get_current_joint_values()
    # path = ompl_planner.plan(start, goal)
    # bestman.sim_execute_trajectory(path, True)
    # bestman.sim_create_movable_constraint("fridge", 1)
    #
    # visualizer.remove_all_line()
    #
    # # The end effector Move along the specified trajectory get effector to open the door
    # init_pose = bestman.sim_get_current_end_effector_pose()
    # rotate_axis = p.getLinkState(client.get_object_id("fridge"), 1)[4]
    # angles = 15
    # heta_values = [math.radians(deg) for deg in range(0, angles + 1)]
    # rotated_joints = [
    #     bestman.sim_cartesian_to_joints(
    #         rotate_point_3d_around_axis(init_pose, rotate_axis, theta, False)
    #     )
    #     for theta in heta_values
    # ]
    # bestman.sim_execute_trajectory(rotated_joints, True)
    #
    # # Wait
    # client.wait(5)
    #
    # # End record / Disconnect pybullet
    # visualizer.end_record()
    client.disconnect()


if __name__ == "__main__":

    # set work dir to Examples
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # get current file name
    filename = os.path.splitext(os.path.basename(__file__))[0]

    main(filename)
