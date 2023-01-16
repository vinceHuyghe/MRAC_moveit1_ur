#!/usr/bin/env python3

from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion

from move_group_utils.move_group_utils import MoveGroupUtils
from pilz_robot_program.pilz_robot_program import (Circ, Lin, Ptp, Sequence,
                                                   from_euler)

# define robot poses
home = (0.0, -pi / 2.0, pi / 2.0, 0.0, pi / 2.0, -pi / 2.0)
pose_l = Pose(position=Point(0.6, -0.6, 0.4),
              orientation=from_euler(0.0, pi, 0.0))
pose_r = Pose(position=Point(0.6, 0.6, 0.4),
              orientation=from_euler(0.0, pi, 0.0))
poses = [home, pose_l, pose_r]


def robot_program():

    # initialize node and moveit commander
    mgi = MoveGroupUtils()

    # wait for rviz and moveit to start
    rospy.sleep(3.0)
    
    # add collision object
    mgi.add_ground_cube()

    # display pose markers in rviz
    mgi.publish_pose_array([pose_l, pose_r])

    # plan to home position
    success, plan = mgi.sequencer.plan(
        Ptp(goal=home, vel_scale=0.3, acc_scale=0.3))[:2]
    if not success:
        return rospy.logerr('Failed to plan to home position')
    mgi.sequencer.execute(plan)

    # plan to pose
    for pose in poses:
        success, plan = mgi.sequencer.plan(
            Ptp(goal=pose, vel_scale=0.3, acc_scale=0.3))[:2]
        if not success:
            return rospy.logerr('Failed to plan to pose')
        mgi.sequencer.execute()

    return rospy.loginfo('Robot program completed')


if __name__ == '__main__':

    robot_program()
