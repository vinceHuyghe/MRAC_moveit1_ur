from typing import List
import rospy
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import RobotTrajectory

from move_group_utils.kdl_kin import KdlKin
from move_group_utils.srv import VisualizeTrajectory, ClearMarkerArray
from moveit_commander.conversions import list_to_pose
from std_msgs.msg import ColorRGBA


start_color = ColorRGBA(1.0, 0.0, 0.0, 0.7)
end_color = ColorRGBA(1.0, 1.0, 0.0, 0.7)
marker_scale = 0.005


def _rgd_gradient(start_color: ColorRGBA,
                  end_color: ColorRGBA,
                  steps: int) -> List[ColorRGBA]:
    color_list = []
    for i in range(steps):
        color = ColorRGBA()
        color.r = start_color.r + (end_color.r - start_color.r) * i / steps
        color.g = start_color.g + (end_color.g - start_color.g) * i / steps
        color.b = start_color.b + (end_color.b - start_color.b) * i / steps
        color.a = start_color.a + (end_color.a - start_color.a) * i / steps
        color_list.append(color)
    return color_list


class TrajectoryVisualizer:

    def __init__(self, name: str):
        self.name = name
        self.kdl_kin = KdlKin()
        self.marker_pub = rospy.Publisher('trajectory_marker_array',
                                          MarkerArray, queue_size=1)
        self.traj_viz_srv = rospy.Service('/mgu/visualize_trajectory',
                                          VisualizeTrajectory,
                                          self.visualize_trajectory_callback)
        self.clear_maker_array_srv = rospy.Service('/mgu/clear_marker_array',
                                                   ClearMarkerArray,
                                                   self.clear_markers_callback)

        self.rate = rospy.Rate(100)

    def run(self):
        rospy.loginfo(f'{self.name}: ready to visualize trajectories')
        while not rospy.is_shutdown():
            self.rate.sleep()

    def visualize_trajectory_callback(self, req):

        self.clear_markers()

        color_list = _rgd_gradient(start_color, end_color, len(
            req.trajectory.joint_trajectory.points))

        marker_array = MarkerArray()
        for i in range(len(req.trajectory.joint_trajectory.points)):
            marker = Marker()
            marker.color = color_list[i]
            marker.header = Header()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.scale.x = marker_scale
            marker.scale.y = marker_scale
            marker.scale.z = marker_scale
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = list_to_pose(self.kdl_kin.get_fk(
                req.trajectory.joint_trajectory.points[i].positions))

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

        return True

    def clear_markers_callback(self, req):

        self.clear_makers()

        return True

    def clear_markers(self):

        marker_array = MarkerArray()
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)


if __name__ == '__main__':
    rospy.init_node('trajectory_visualizer', anonymous=True)
    visualizer = TrajectoryVisualizer(rospy.get_name())
    visualizer.run()
