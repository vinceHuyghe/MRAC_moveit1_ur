#!/usr/bin/env python3

from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion

from move_group_utils.move_group_utils import (MoveGroupUtils,
                                               publish_trajectory_markers)
from pilz_robot_program.pilz_robot_program import (Circ, Lin, Ptp, Sequence)


def robot_program():

    # initialize node and moveit commander
    mgi = MoveGroupUtils()
    mgi.add_ground_cube()

    # set and plan to home position
    home = (0.0, -pi / 2.0, pi / 2.0, 0.0, pi / 2.0, -pi / 2.0)

    success, plan = mgi.sequencer.plan(Ptp(goal=home))[:2]
    if not success:
        return rospy.logerr('Failed to plan to home position')
    mgi.sequencer.execute()

    # initialize sequence
    sequence = Sequence()

    # append commands to sequence
    sequence.append(Ptp(goal=home))
    sequence.append(
        Ptp(
            goal=Pose(
                position=Point(0.4, 0.0, 0.6),
                orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
            ),
            vel_scale=0.1,
            acc_scale=0.05,
        )
    )

    sequence.append(
        Lin(
            goal=Pose(
                position=Point(0.4, 0.3, 0.6),
                orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
            ),
            vel_scale=0.1,
            acc_scale=0.05,
        ),
        blend_radius=0.01,
    )

    sequence.append(
        Lin(
            goal=Pose(
                position=Point(0.4, -0.3, 0.6),
                orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
            ),
            vel_scale=0.1,
            acc_scale=0.05,
        )
    )

    sequence.append(
        Circ(
            goal=Pose(
                position=Point(0.4, 0.0, 0.9),
                orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
            ),
            center=Point(0.4, 0.0, 0.6),
        ),
        blend_radius=0.01,
    )

    sequence.append(
        Circ(
            goal=Pose(
                position=Point(0.4, 0.3, 0.6),
                orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
            ),
            center=Point(0.4, 0.0, 0.6),
        ),
        blend_radius=0.01,
    )

    sequence.append(
        Lin(
            goal=Pose(
                position=Point(0.4, 0.0, 0.6),
                orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
            ),
            vel_scale=0.1,
            acc_scale=0.05,
        )
    )

    success, plan = mgi.sequencer.plan(sequence)[:2]

    # visualize trajectory
    publish_trajectory_markers(plan[0])
    mgi.display_trajectory(plan)

    if not success:
        return rospy.logerr('Failed to plan sequence')
    mgi.sequencer.execute()

    return rospy.loginfo('Robot program completed')


if __name__ == '__main__':

    robot_program()
