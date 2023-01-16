#!/usr/bin/env python3
from math import pi

import rospy
import tf
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

from move_group_utils.move_group_utils import MoveGroupUtils
from pilz_robot_program.pilz_robot_program import Lin

# define poses
pose0 = Pose(
    position=Point(0.8, 0, 0.3),
    orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
)
pose1 = Pose(
    position=Point(0.8, 0.6, 0.3),
    orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
)
pose2 = Pose(
    position=Point(0.8, -0.6, 0.3),
    orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
)
poses = [pose0, pose1, pose0, pose2]

# define end effector
ee_name = 'extruder'
ee_file_path = '/dev_ws/src/ur10e_examples/end_effector/ur_ee.stl'
ori = tf.transformations.quaternion_from_euler(0, pi / 2, 0)
tcp_pose = Pose(position=Point(0.184935, 0.0, 0.06),
                orientation=Quaternion(ori[0], ori[1], ori[2], ori[3]))


def robot_program():

    # initialize node and moveit commander
    mgi = MoveGroupUtils()

    # wait for rviz and moveit to start
    rospy.sleep(3.0)

    # create joint goal
    home = mgi.create_goal(
        [0.0, -pi / 2.0, pi / 2.0, 0.0, pi / 2.0, -pi / 2.0])

    # display pose markers in rviz
    mgi.publish_pose_array(poses)

    # attach collision object with subframe
    mgi.attach_end_effector(ee_name, tcp_pose, ee_file_path)

    # name of collision object / name of subframe
    mgi.move_group.set_end_effector_link(f'{ee_name}/tcp')
    rospy.loginfo(
        f'{mgi.name}: end effector link set to {mgi.move_group.get_end_effector_link()}'
        )

    # set planning pipeline
    mgi.move_group.set_planning_pipeline_id('ompl')

    # plan and execute to home pose
    mgi.move_group.set_joint_value_target(home)
    mgi.move_group.go()

    # plan and execute to poses (OMPL)
    for pose in poses:
        mgi.move_group.set_pose_target(pose)
        success, plan = mgi.move_group.plan()[:2]
        if success:
            mgi.move_group.execute(plan, wait=True)
        else:
            return rospy.loginfo(f'{mgi.name}: planning failed, robot program aborted')

    # plan and execute to poses (PILZ LIN)
    for pose in poses:
        plan = mgi.sequencer.plan(Lin(goal=pose, vel_scale=0.3, acc_scale=0.3))
        mgi.sequencer.execute()

    # detach collision object and reset end effector link
    mgi.detach_end_effector(ee_name)
    mgi.move_group.set_end_effector_link('tool0')
    rospy.loginfo(
        f'{mgi.name}: end effector link set to {mgi.move_group.get_end_effector_link()}'
    )

    # plan and execute to home pose
    mgi.move_group.set_joint_value_target(home)
    mgi.move_group.go()

    return rospy.loginfo(f'{mgi.name}: robot program finished')


if __name__ == '__main__':

    robot_program()
