import math
import sys
import os

"""
Get the utils module path
"""
# customized package
current_path = os.path.abspath(__file__)
utils_path = os.path.dirname(os.path.dirname(current_path)) + '/utils'
if os.path.basename(utils_path) != 'utils':
    raise ValueError('Not add the path of folder "utils", please check again!')
sys.path.append(utils_path)
from utils_Bestman import Bestman, Pose
from utils_PbClient import PbClient
from utils_PbVisualizer import PbVisualizer
from utils_Kitchen_object import Kitchen

# load cleint
pb_client = PbClient(enable_GUI=True)
pb_client.enable_vertical_view(1.0, [1.7, 3.68, 1.95], -86.4, -52.3)

# load visualizer
pb_visualizer = PbVisualizer(pb_client)

# load robot
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose, pb_client)

# reset arm joint position
pose1 = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(pose1)

# load kitchen
kitchen = Kitchen(pb_client)

pb_visualizer.set_elementB_visual_color

# # start recording
# logID = pb_client.start_record("example_video")

# open a drawer in element A
for i in range(10):
    drawer_id = i+1
    kitchen.open_it('elementA', drawer_id)

# close a drawer in element A
for i in range(10):
    drawer_id = i+1
    kitchen.close_drawer('elementA', drawer_id)

# open a drawer in element C
for i in range(3):
    drawer_id = i+1
    kitchen.open_it('elementC', drawer_id)

# close a drawer in element C
for i in [2, 1, 0]:
    drawer_id = i+1
    kitchen.close_drawer('elementC', drawer_id)

# open a drawer in element D
for i in range(1):
    drawer_id = i+1
    kitchen.open_it('elementD', drawer_id)

# close a drawer in element D
for i in range(1):
    drawer_id = i+1
    kitchen.close_drawer('elementD', drawer_id)

# open a drawer in element E
for i in range(2):
    drawer_id = i+1
    kitchen.open_it('elementE', drawer_id)

# close a drawer in element E
for i in range(2):
    drawer_id = i+1
    kitchen.close_drawer('elementE', drawer_id)

# # end recording
# pb_client.end_record(logID)

# wait a few seconds
pb_client.wait(10)

# disconnect pybullet
pb_client.disconnect_pybullet()