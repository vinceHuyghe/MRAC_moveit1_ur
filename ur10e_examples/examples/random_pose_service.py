#!/usr/bin/env python3
import random

import rospy

from ur10e_examples.srv import RandomPose


class RandomPoseSrv():

    def __init__(self, name) -> None:

        self.name = name
        rospy.loginfo(f'{self.name} started')
        self.y = (-0.6, 0.6)
        self.z = (0.2, 1)
        self.offset = 0.2
        self.service = rospy.Service(
            '/random_pose', RandomPose, self.callback_random_pose
        )

    def callback_random_pose(self, req):

        pose = req.current_pose
        pose.position.y = RandomPoseSrv.generate_offset(
            self.offset, pose.position.y, self.y
        )
        pose.position.z = RandomPoseSrv.generate_offset(
            self.offset, pose.position.z, self.z
        )

        return pose

    @staticmethod
    def generate_offset(offset, value: float, bounds: tuple) -> bool:

        value = random.uniform((-1.0 * offset), offset)

        if bounds[0] < value < bounds[1]:
            value = value
        elif value < bounds[0]:
            value = bounds[0] + random.uniform(0, 0.2)
        elif value > bounds[1]:
            value = bounds[1] - random.uniform(0, 0.2)

        return round(value, 6)


if __name__ == '__main__':

    while not rospy.is_shutdown():
        try:
            rospy.init_node('random_pose_srv')
            RandomPoseSrv(rospy.get_name)
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
