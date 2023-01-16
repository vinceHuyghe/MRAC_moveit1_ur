#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray
from move_group_utils.srv import VisualizePoses


class PoseVisualizer:

    def __init__(self, name: str):
        self.name = name
        self.pose_array = PoseArray()
        self.pose_pub = rospy.Publisher(
            "/mgu_poses", PoseArray, queue_size=1)
        self.viz_pose_srv = rospy.Service("/mgu/visualize_poses",
                                          VisualizePoses,
                                          self.visualize_poses_callback)
        self.rate = rospy.Rate(100)

    def run(self):
        rospy.loginfo(f'{self.name}: ready to visualize poses')
        while not rospy.is_shutdown():
            self.rate.sleep()

    def visualize_poses_callback(self, req):
        self.pose_array.poses = req.poses
        self.pose_array.header.frame_id = req.frame_id
        self.pose_array.header.stamp = rospy.Time.now()
        self.pose_pub.publish(self.pose_array)
        return True


if __name__ == '__main__':

    rospy.init_node('pose_visualizer', anonymous=True)
    visualizer = PoseVisualizer(rospy.get_name())
    visualizer.run()
