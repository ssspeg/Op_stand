"""
@Description :   A few functions used in bestman robot, where the robot has a base and an arm.
@Author      :   Yan Ding 
@Time        :   2023/08/30 20:43:44
"""


import pybullet as p

from .Bestman_sim import Bestman_sim


class Bestman_sim_panda(Bestman_sim):
    
    def __init__(self, client, visualizer,  cfg):
        """BestMan_sim for panda arm.
        """
        
        # Init parent class: BestMan_sim
        super().__init__(client, visualizer,  cfg)
        
        # Create a gear constraint to keep the fingers symmetrically centered
        c = p.createConstraint(
            self.arm_id,
            9,
            self.arm_id,
            10,
            jointType=p.JOINT_GEAR,
            jointAxis=[1, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0]
        )
        
        # Constraint parameters
        p.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)

    
    # ----------------------------------------------------------------
    # functions for gripper
    # ----------------------------------------------------------------
    
    def sim_active_gripper(self, value):
        """
        Activate or deactivate the gripper.

        Args:
            object_id (init): ID of the object related to gripper action.
            value (int): 0 or 1, where 0 means deactivate (ungrasp) and 1 means activate (grasp).
        """

        if value == 1:
            for i in [9, 10]:
                p.setJointMotorControl2(self.arm_id, i, p.POSITION_CONTROL, 0.04, force=10)
        elif value == 0:
            for i in [9, 10]:
                p.setJointMotorControl2(self.arm_id, i, p.POSITION_CONTROL, 0.01, force=10)
        else:
            raise(ValueError("gripper value must be 0 / 1 !"))
        
        self.client.run(10)