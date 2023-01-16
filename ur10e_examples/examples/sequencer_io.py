#!/usr/bin/env python3

from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from ur_msgs.srv import SetIO

from move_group_utils.move_group_utils import MoveGroupUtils
from pilz_robot_program.pilz_robot_program import (Circ, Lin, Ptp, Sequence,
                                                   from_euler)


# define robot poses
home = (0.0, -pi/2.0, pi/2.0, -pi, -pi/2, 0)
pose_l = Pose(position=Point(0.5, -0.6, 0.4),
              orientation=from_euler(0.0, pi, 0.0))
pose_r = Pose(position=Point(0.5, 0.6, 0.4),
              orientation=from_euler(0.0, pi, 0.0))


def robot_program():

    # initialize node and moveit commander
    mgi = MoveGroupUtils()

    # wait for rviz and moveit to start
    rospy.sleep(3.0)
    
    # add collision object
    mgi.add_ground_cube()

    # wait for ur_hardware_interface/set_io service
    rospy.wait_for_service('ur_hardware_interface/set_io', 30)
    set_io = rospy.ServiceProxy('ur_hardware_interface/set_io', SetIO)

    # plan to home position
    success, plan = mgi.sequencer.plan(
        Ptp(goal=home, vel_scale=0.3, acc_scale=0.3))[:2]
    if not success:
        return rospy.logerr('Failed to plan to home position')
    mgi.sequencer.execute(plan)

    # plan to pose
    success, plan = mgi.sequencer.plan(
        Ptp(goal=pose_l, vel_scale=0.3, acc_scale=0.3))[:2]
    if not success:
        return rospy.logerr('Failed to plan to pose_l')
    mgi.sequencer.execute(plan)

    # set DO0 to 1 (ON)
    set_io(1, 0, 1)

    # plan to pose
    success, plan = mgi.sequencer.plan(
        Ptp(goal=pose_r, vel_scale=0.3, acc_scale=0.3))[:2]
    if not success:
        return rospy.logerr('Failed to plan to pose_r')
    mgi.sequencer.execute(plan)

    # set DO0 to 0 (OFF)
    set_io(1, 0, 0)

    return rospy.loginfo('Robot program completed')


if __name__ == '__main__':

    robot_program()
