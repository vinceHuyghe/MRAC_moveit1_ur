#!/usr/bin/env python3
from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion

from move_group_utils.move_group_utils import (MoveGroupUtils,
                                               publish_trajectory_markers)
from pilz_robot_program.pilz_robot_program import Lin, Ptp
from ur10e_examples.srv import RandomPose

# define robot poses
home = (0.0, -pi / 2.0, pi / 2.0, 0.0, pi / 2.0, 0.0)

pose = Pose(position=Point(0.6, 0, 0.5),
            orientation=Quaternion(0.0, 0.5, 0.0, 0.5))


def robot_program():

    rospy.wait_for_service('/random_pose', 30)

    mgi = MoveGroupUtils()

    mgi.sequencer.plan(Ptp(goal=home))
    mgi.sequencer.execute()

    mgi.sequencer.plan(Ptp(goal=pose))
    mgi.sequencer.execute()

    # Start client loop
    random_offset = rospy.ServiceProxy('/random_pose', RandomPose)

    while not rospy.is_shutdown():

        resp = random_offset(pose)

        mgi.publish_pose_array([resp.pose])

        success, plan = mgi.sequencer.plan(Lin(goal=resp.pose,
                                               vel_scale=0.3,
                                               acc_scale=0.1))[:2]
        publish_trajectory_markers(plan[0])
        mgi.sequencer.execute()


if __name__ == '__main__':

    robot_program()
