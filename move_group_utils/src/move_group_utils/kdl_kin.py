from math import pi
from typing import List

import numpy as np
import PyKDL as kdl
import rospy
from kdl_parser_py.urdf import treeFromParam, treeFromUrdfModel
from urdf_parser_py.urdf import URDF


def frame_to_list(frame: kdl.Frame) -> List[float]:

    rot = frame.M.GetQuaternion()

    return [frame.p.x(), frame.p.y(), frame.p.z(),
            rot[0], rot[1], rot[2], rot[3]]


class KdlKin():

    def __init__(self):

        if not rospy.has_param('robot_description'):
            return rospy.logerr('No robot description found on parameter server')

        self.urdf = URDF.from_parameter_server()
        # TODO get joint limits from urdf

        success, self.kdl_tree = treeFromUrdfModel(self.urdf)
        if not success:
            rospy.logerr("Failed to construct kdl tree")
            raise Exception("Failed to construct kdl tree")
        self.base_link = self.urdf.get_root()
        self.ee_link = self.urdf.links[-1].name

        self.manipulator_chain = self.kdl_tree.getChain(
            self.base_link, self.ee_link)

        # TODO fix hardcoded number of joints
        self.num_joints = 6

        # KDL solvers
        # TODO: check other ik solvers
        self.fk_pos_solver = kdl.ChainFkSolverPos_recursive(
            self.manipulator_chain)
        # self.ik_pos_solver = kdl.ChainIkSolverPos_LMA(self.manipulator_chain)
        self.ik_vel_solver = kdl.ChainIkSolverVel_pinv(self.manipulator_chain)
        self.ik_pos_solver = kdl.ChainIkSolverPos_NR(self.manipulator_chain,
                                                     self.fk_pos_solver,
                                                     self.ik_vel_solver)

    def get_joint_names(self, links=False, fixed=False):
        return self.urdf.get_chain(self.base_link, self.ee_link,
                                   links=links, fixed=fixed)

    def print_kdl_chain(self):
        for idx in range(self.manipulator_chain.getNrOfSegments()):
            print(('* ' + self.manipulator_chain.getSegment(idx).getName()))

    def joints_to_kdl(self, values):

        kdl_array = kdl.JntArray(self.num_joints)

        for idx in range(self.num_joints):
            kdl_array[idx] = values[idx]

        return kdl_array

    def get_fk(self, joint_positions: List[float]) -> List[float]:

        end_frame = kdl.Frame()
        self.fk_pos_solver.JntToCart(
            self.joints_to_kdl(joint_positions), end_frame)

        return frame_to_list(end_frame)

    def get_ik(self, cartesian_position: List[float]) -> List[float]:

        # get inverse kinematics
        joint_positions = kdl.JntArray(
            self.manipulator_chain.getNrOfJoints())
        self.ik_pos_solver.CartToJnt(joint_positions,
                                     kdl.Frame(kdl.Rotation.Quaternion(
                                         cartesian_position[3],
                                         cartesian_position[4],
                                         cartesian_position[5],
                                         cartesian_position[6]),
                                         kdl.Vector(cartesian_position[0],
                                                    cartesian_position[1],
                                                    cartesian_position[2])),
                                     joint_positions)

        return joint_positions

    # def get_all_ik(self, cartesian_position: List[float]) -> List[List[float]]:
    #     # get inverse kinematics
    #     joint_positions = kdl.JntArray(
    #         self.manipulator_chain.getNrOfJoints())
    #     num_sols = self.ik_pos_solver.CartToJnt(joint_positions,
    #                                             kdl.Frame(kdl.Rotation.Quaternion(
    #                                                 cartesian_position[3],
    #                                                 cartesian_position[4],
    #                                                 cartesian_position[5],
    #                                                 cartesian_position[6]),
    #                                                 kdl.Vector(cartesian_position[0],
    #                                                            cartesian_position[1],
    #                                                            cartesian_position[2])),
    #                                             joint_positions)
    #     sols = []
    #     for i in range(num_sols):
    #         sols.append(list(joint_positions[i]))
    #     return sols


if __name__ == '__main__':

    kdl_kin = KdlKin()

    # get forward kinematics
    joint_positions = [pi/2, 0.0, 0.0, 0.0, 0.0, 0.0]
    cartesian_position = kdl_kin.get_fk(joint_positions)
    print(cartesian_position)

    # # get inverse kinematics
    # cartesian_position = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    joint_positions = kdl_kin.get_ik(cartesian_position)
    print(joint_positions)
