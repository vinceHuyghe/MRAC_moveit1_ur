#!/usr/bin/env python3
from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from move_group_utils.move_group_utils import MoveGroupUtils

# define poses
pose0 = Pose(
    position=Point(0.8, -0.6, 0.3),
    orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
)
pose1 = Pose(
    position=Point(0.8, 0.6, 0.3),
    orientation=Quaternion(0.0, 1.0, 0.0, 0.0)
)
poses = [pose0, pose1]


def robot_program():

    # initialize node and moveit commander
    mgi = MoveGroupUtils()

    # wait for rviz and moveit to start
    rospy.sleep(3.0)

    # add collision object
    mgi.add_ground_cube()

    # define robot poses
    home = mgi.create_goal(
        [0.0, -pi / 2.0, pi / 2.0, 0.0, pi / 2.0, -pi / 2.0])

    # display pose markers in rviz
    mgi.publish_pose_array(poses)

    # set planning pipeline
    # Available planers ompl, chomp, stomp
    mgi.move_group.set_planning_pipeline_id('ompl')

    # set planner id, see {planner}_planning_pipeline.launch available options
    # mgi.move_group.set_planner_id('RRTConnect')

    # plan to home position
    mgi.move_group.set_joint_value_target(home)
    success, plan = mgi.move_group.plan()[:2]

    # execute plan if planning was successful
    if success:
        mgi.move_group.execute(plan, wait=True)
    else:
        return rospy.loginfo(f'{mgi.name}: planning failed, robot program aborted')

    # plan to pose
    for pose in poses:
        mgi.move_group.set_pose_target(pose)
        success, plan = mgi.move_group.plan()[:2]
        if not success:
            return rospy.loginfo(f'{mgi.name}: planning failed, robot program aborted')
        mgi.move_group.execute(plan, wait=True)

    # plan to joint target
    mgi.move_group.set_joint_value_target(
        mgi.create_goal((0.0, -pi / 2.0, pi / 2.0, 0.0, pi / 2.0, -pi / 2))
    )
    success, plan = mgi.move_group.plan()[:2]
    if not success:
        return rospy.loginfo(f'{mgi.name}: planning failed, robot program aborted')
    mgi.move_group.execute(plan, wait=True)

    # create collision object
    co_pose = PoseStamped()
    co_pose.header = Header()
    co_pose.header.frame_id = mgi.move_group.get_planning_frame()
    co_pose.pose = Pose(position=Point(1, 0, 0.2),
                        orientation=Quaternion(0, 0, 0, 1))

    co = CollisionObject()
    co.operation = CollisionObject.ADD
    co.id = 'box'
    co.header = co_pose.header
    co.pose = co_pose.pose
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [1, 0.2, 0.5]
    co.primitives = [box]

    # add collision object to planning scene
    mgi.add_collision_object(co)

    # plan and execute to poses
    for pose in poses:
        mgi.move_group.set_pose_target(pose)
        success, plan = mgi.move_group.plan()[:2]
        if not success:
            return rospy.loginfo(f'{mgi.name}: planning failed, robot program aborted')
        mgi.move_group.execute(plan, wait=True)

    return rospy.loginfo(f'{mgi.name}: robot program completed')


if __name__ == '__main__':

    robot_program()
