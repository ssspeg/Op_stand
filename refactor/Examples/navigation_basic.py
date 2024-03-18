"""
@Description :   This script shows how to navigate to a goal position
@Author      :   Yan Ding 
@Time        :   2023/08/31 03:01:50
"""

import math
import sys
import os

sys.path.append('/BestMan_Pybullet/refactor')

from Motion_Planning.Robot.Bestman import Bestman
from Motion_Planning.Robot.Pose import Pose
from Env.PbClient import PbClient
from Visualization.PbVisualizer import PbVisualizer
from Motion_Planning.navigation.navigation import navigation
from utils_PbOMPL import PbOMPL

# load kitchen from three scenarios
index = 0
if index == 0:
    from Env.Kitchen_v0 import Kitchen
elif index == 1:
    from Env.Kitchen_v1 import Kitchen
else:
    assert False, "index should be 0 or 1"


pb_client = PbClient(enable_GUI=True)
pb_client.enable_vertical_view(4.0, [1.0, 1.0, 0])
pb_visualizer = PbVisualizer(pb_client)
# logID = pb_client.start_record("example_manipulation") # start recording
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose, pb_client)  # load robot
demo.get_joint_link_info("arm")  # get info about arm
init_joint = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(init_joint)  # reset arm joint position

# load table, bowl, and chair
table_id = pb_client.load_object(
    "/BestMan_Pybullet/refactor/Asset/URDF_models/furniture_table_rectangle_high/table.urdf",
    [1.0, 1.0, 0.0],
    [0.0, 0.0, 0.0],
    1.0,
    "table",
    fixed_base=True,
)
bowl_id = pb_client.load_object(
    "/BestMan_Pybullet/refactor/Asset/URDF_models/utensil_bowl_blue/model.urdf",
    [0.6, 0.6, 0.85],
    [0.0, 0.0, 0.0],
    1.0,
    "bowl",
    tag_obstacle_navigate=False,
)
chair_id = pb_client.load_object(
    "/BestMan_Pybullet/refactor/Asset/URDF_models/furniture_chair/model.urdf",
    [-0.3, 0.8, 0.1],
    [math.pi / 2.0 * 3, 0.0, math.pi / 2.0],
    1.5,
    "chair",
    tag_obstacle_navigate=False,
)

# add obstacles in the navigation
print("obstacles for navigation: {}".format(pb_client.obstacle_navigation_ids))

# get bounding box of objects
aabb_table = pb_client.get_bounding_box(table_id)
pb_visualizer.draw_aabb(table_id)
print("-" * 20 + "\n" + "aabb_table:{}".format(aabb_table))

# plot line connecting init and goal positions
target_position = [5.0, 1.0, 0]
pb_visualizer.draw_line([1, 0, 0], target_position)

# navigate algorithm
goal_base_pose = Pose(target_position, [0.0, 0.0, math.pi / 2.0])
nav = navigation(demo)
path = nav.A_star(goal_base_pose)

# navigate segbot
demo.navigate_base(goal_base_pose, path)

# # end recording
# pb_client.end_record(logID)

pb_client.wait(5)
pb_client.disconnect_pybullet()