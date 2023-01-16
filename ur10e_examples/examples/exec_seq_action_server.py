#!/usr/bin/env python3

from math import pi
import actionlib
import rospy
import ur10e_examples.msg

from move_group_utils.move_group_utils import (MoveGroupUtils,
                                               publish_trajectory_markers)
from pilz_robot_program.pilz_robot_program import Circ, Lin, Ptp, Sequence

home = (0.0, -pi/2.0, pi/2.0, -pi, -pi/2, 0)


class ExecSeq:

    # create messages that are used to publish feedback/result
    _feedback = ur10e_examples.msg.ExecSeqFeedback()
    _result = ur10e_examples.msg.ExecSeqResult()

    def __init__(self):

        self.mgi = MoveGroupUtils()
        self._as = actionlib.SimpleActionServer(
            self.mgi.name,
            ur10e_examples.msg.GotoAction,
            execute_cb=self.execute_callback,
            auto_start=False,
        )
        self._as.start()

    def execute_callback(self, poses):

        rospy.loginfo(f'{self.mgi.name}: exec_callback')

        sequence = Sequence()

        sequence.append(Ptp(goal=home))

        # start executing the action
        for p in poses.poses:
            sequence.append(Lin(goal=p))

        success, plan = self.mgi.sequencer.plan(sequence)[:2]

        self.mgi.display_trajectory(plan)
        publish_trajectory_markers(plan[0])

        self._feedback.planning_succeeded = success

        # publish the feedback
        self._as.publish_feedback(self._feedback)

        # check that preempt has not been requested by the client
        # in this specific use case the preempt is not required, as
        # there is only one goal to execute. It is in this example for
        # completeness
        if self._as.is_preempt_requested():
            self._as.set_preempted()
            return rospy.loginfo(f'{self.mgi.name}: Action Preempted')

        # if execute returns None motion was executed
        if not self.mgi.sequencer.execute():
            self._result.sequence_completed = True
            self._as.set_succeeded(self._result)
            return rospy.loginfo(f'{self.mgi.name}: Action Succeeded')

        return rospy.loginfo(f'{self.mgi.name}: Action Failed')


if __name__ == '__main__':

    ExecSeq()
    rospy.spin()
