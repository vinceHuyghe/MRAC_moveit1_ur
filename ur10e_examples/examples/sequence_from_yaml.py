#!/usr/bin/env python3
from math import pi

import rospy
from moveit_commander.conversions import list_to_pose

from move_group_utils.move_group_utils import (MoveGroupUtils,
                                               publish_trajectory_markers,
                                               poses_list_from_yaml)
from pilz_robot_program.pilz_robot_program import Lin, Ptp, Sequence


home = (0.0, -pi/2.0, pi/2.0, 0.0, pi/2.0, -pi/2)


def robot_program():

    mgi = MoveGroupUtils()
    rospy.sleep(3.0)

    sequence = Sequence()

    sequence.append(Ptp(goal=home))

    # create pose mgs list from yaml
    poses_list = poses_list_from_yaml(
        '/dev_ws/src/ur10e_examples/toolpaths/test.yaml')

    # # alternative poses from ros param server
    # if rospy.has_param('gh_poses'):
    #     poses_list = rospy.get_param('gh_poses')
    # else:
    #     rospy.logerr('No poses found on ros param server')
    #     return

    poses = [list_to_pose(pose) for pose in poses_list]

    # publish the poses to rviz for preview
    mgi.publish_pose_array(poses)

    for p in poses:
        sequence.append(Lin(goal=p))

    success, plan = mgi.sequencer.plan(sequence)[:2]

    mgi.display_trajectory(plan)
    publish_trajectory_markers(plan[0])

    if not success:
        return rospy.logerr(f'{mgi.name}: Failed to plan to sequence')
    mgi.sequencer.execute()

    return rospy.loginfo('Robot program completed')


if __name__ == '__main__':

    robot_program()
