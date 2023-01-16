#! /usr/bin/env python3
import actionlib
import rospy
import ur10e_examples.msg
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_commander.conversions import list_to_pose

from move_group_utils.move_group_utils import poses_list_from_yaml


def exec_seq_client():

    # Creates the SimpleActionClient, passing the type of the action
    # (ExecSeq) to the constructor.
    client = actionlib.SimpleActionClient(
        '/exec_seq_action_server', ur10e_examples.msg.ExecSeqAction
    )

    # Waits until the action server has started
    client.wait_for_server()

    poses_list = poses_list_from_yaml(
        '/dev_ws/src/ur10e_examples/toolpaths/test.yaml')
    poses = [list_to_pose(pose) for pose in poses_list]

    # Creates a goal to send to the action server.
    goal = ur10e_examples.msg.ExecSeqGoal(poses)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


if __name__ == '__main__':

    rospy.init_node('execute_sequence_action_client')
    result = exec_seq_client()
    rospy.loginfo(f'action result: {result.sequence_completed}')
