from math import atan2, cos, pi, sin, sqrt
from sys import argv
from typing import List

import moveit_commander
import moveit_msgs.msg
import pyassimp
import rospy
import tf.transformations
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject,
                             RobotTrajectory)
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from yaml import safe_load

from pilz_robot_program.pilz_robot_program import (Circ, Lin, MoveGroupSequence,
                                                   Ptp, Sequence, from_euler)
from move_group_utils.srv import VisualizePoses, VisualizeTrajectory, ClearMarkerArray

PLANNING_GROUP = 'manipulator'


class MoveGroupUtils:

    def __init__(self) -> None:

        moveit_commander.roscpp_initialize(argv)
        rospy.init_node('move_group_utils', anonymous=True)
        self.name = rospy.get_name()
        rospy.loginfo(f'{self.name}: node started')

        # Instantiate a `RobotCommander`_ object. Provides information such as
        # the robot's kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object.  This provides a
        # remote interface for getting, setting, and updating the robot's
        # internal understanding of the surrounding world
        self.scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object.  This object is an
        # interface to a planning group (group of joints).
        self.move_group = moveit_commander.MoveGroupCommander(PLANNING_GROUP)

        # Create a `DisplayTrajectory` ROS publisher which is used to display
        # move group sequence trajectories in Rviz
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        # Instantiate a pilz `MoveGroupSequence`_ object.
        self.sequencer = MoveGroupSequence(self.move_group)

    def create_goal(self, joint_values: List[float]) -> JointState:

        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.header.frame_id = 'base_link'
        joint_state.name = self.move_group.get_active_joints()
        joint_state.position = joint_values

        return joint_state

    def display_trajectory(self, trajectory: RobotTrajectory) -> None:

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory = trajectory

        self.display_trajectory_publisher.publish(display_trajectory)

    def publish_pose_array(self, poses: List[Pose], frame_id: str = None
                           ) -> bool:

        if frame_id is None:
            frame_id = self.move_group.get_planning_frame()

        rospy.wait_for_service('/mgu/visualize_poses', 10)
        srv = rospy.ServiceProxy('/mgu/visualize_poses', VisualizePoses)

        return srv(frame_id, poses)

    def add_collision_object(self, co: CollisionObject) -> bool:

        self.scene.add_object(co)

        return self.wait_for_state_update(f'{co.id}', object_is_known=True)

    def add_ground_cube(self) -> bool:

        pose = PoseStamped()
        pose.header.frame_id = self.robot.get_planning_frame()
        pose.header.stamp = rospy.Time.now()
        pose.pose = Pose(
            position=Point(0, 0, -0.51), orientation=Quaternion(0, 0, 0, 1)
        )

        self.scene.add_box('ground', pose, (2, 2, 1))
        
        return self.wait_for_state_update('ground', object_is_known=True)

    def get_ik(self, pose: Pose) -> JointState:

        rospy.wait_for_service('/compute_ik', 10)
        ik_srv = rospy.ServiceProxy('/compute_ik', GetPositionIK)

        req = GetPositionIKRequest()
        req.ik_request.group_name = self.move_group.get_name()
        req.ik_request.robot_state = self.robot.get_current_state()
        req.ik_request.avoid_collisions = True
        req.ik_request.ik_link_name = self.move_group.get_end_effector_link()
        req.ik_request.timeout = rospy.Duration(10)
        req.ik_request.pose_stamped.header = Header()
        req.ik_request.pose_stamped.header.frame_id = self.move_group.get_planning_frame()
        req.ik_request.pose_stamped.pose = pose

        res = ik_srv(req)

        return res.solution.joint_state

    def attach_end_effector(
        self,
        name: str,
        tcp_pose: Pose,
        filepath: str
    ) -> bool:

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = self.move_group.get_end_effector_link()
        pose.pose = Pose(position=Point(0, 0, 0),
                         orientation=Quaternion(0, 0, 0, 1))

        ac = AttachedCollisionObject()
        ac.object = make_mesh(name, pose, filepath=filepath)

        ac.link_name = self.move_group.get_end_effector_link()
        ac.object.subframe_names = ['tcp']
        ac.object.subframe_poses = [tcp_pose]
        # TODO fix sleep req
        rospy.sleep(1)
        self.scene.attach_object(ac)

        return self.wait_for_state_update(name, object_is_attached=True)

    def detach_end_effector(self, name: str) -> bool:

        self.scene.remove_attached_object(name)
        self.scene.remove_world_object(name)

        return self.wait_for_state_update(name, object_is_known=False,
                                          object_is_attached=False)

    # TODO make tcp pose arg
    def attach_camera(self, name: str) -> bool:

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = self.move_group.get_end_effector_link()
        pose.pose = Pose(
            position=Point(0, 0, 0.0115), orientation=Quaternion(0, 0, 0, 1)
        )

        ac = AttachedCollisionObject()
        ac.object.id = name
        ac.object.header = pose.header
        ac.object.pose = pose.pose
        ac.object.operation = CollisionObject.ADD
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.042, 0.042, 0.023]
        ac.object.primitives = [box]
        ac.link_name = self.move_group.get_end_effector_link()
        ac.object.subframe_names = ['tcp']
        ac.object.subframe_poses = [
            Pose(position=Point(0.0, 0.0, 0.001),
                 orientation=Quaternion(0, 0, 0, 1))
        ]
        # TODO fix sleep req
        rospy.sleep(1)
        self.scene.attach_object(ac)

        return self.wait_for_state_update(name, object_is_attached=True)

    def attach_camera(self,
                      name: str,
                      tcp_pose: Pose,
                      size: List[float]) -> bool:

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = self.move_group.get_end_effector_link()
        pose.pose = Pose(
            position=Point(0, 0, size[2]/2), orientation=Quaternion(0, 0, 0, 1)
        )

        ac = AttachedCollisionObject()
        ac.object.id = name
        ac.object.header = pose.header
        ac.object.pose = pose.pose
        ac.object.operation = CollisionObject.ADD
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = size
        ac.object.primitives = [box]
        ac.link_name = self.move_group.get_end_effector_link()
        ac.object.subframe_names = ['tcp']
        ac.object.subframe_poses = [tcp_pose]
        # TODO fix sleep req
        rospy.sleep(1)
        self.scene.attach_object(ac)

        return self.wait_for_state_update(name, object_is_attached=True)

    def wait_for_state_update(
        self,
        object_name: str,
        object_is_attached: bool = False,
        object_is_known: bool = False,
        timeout: int = 4,
    ) -> bool:

        start = rospy.get_time()
        seconds = rospy.get_time()

        while (seconds - start < timeout) and not rospy.is_shutdown():

            attached_objects = self.scene.get_attached_objects([object_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = object_name in self.scene.get_known_object_names()

            if (object_is_attached == is_attached) and (
                    object_is_known == is_known):
                rospy.loginfo(f'{self.name}: scene updated -> {object_name}')
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        rospy.logerr(f'{self.name}: failed to update scene -> {object_name}')

        return False


def publish_trajectory_markers(trajectory: RobotTrajectory) -> bool:

    rospy.wait_for_service('/mgu/visualize_trajectory', 10)

    srv = rospy.ServiceProxy(
        '/mgu/visualize_trajectory', VisualizeTrajectory)

    return srv.call(trajectory)


def clear_marker_array() -> bool:

    rospy.wait_for_service('/mgu/clear_visualized_trajectory', 10)

    srv = rospy.ServiceProxy(
        '/mgu/clear_visualized_trajectory', ClearMarkerArray)

    return srv.call()


def make_mesh(
    name: str, pose: PoseStamped, filepath: str, scale=(1, 1, 1)
) -> CollisionObject:

    co = CollisionObject()
    if pyassimp is False:
        rospy.logerr(
            'Pyassimp needs patch https://launchpadlibrarian.net/319496602/patchPyassim.txt'
        )
    scene = pyassimp.load(filepath)
    if not scene.meshes or len(scene.meshes) == 0:
        rospy.logerr('There are no meshes in the file')
    if len(scene.meshes[0].faces) == 0:
        rospy.logerr('There are no faces in the mesh')
    co.operation = CollisionObject.ADD
    co.id = name
    co.header = pose.header
    co.pose = pose.pose

    mesh = Mesh()
    first_face = scene.meshes[0].faces[0]
    if hasattr(first_face, '__len__'):
        for face in scene.meshes[0].faces:
            if len(face) == 3:
                triangle = MeshTriangle()
                triangle.vertex_indices = [face[0], face[1], face[2]]
                mesh.triangles.append(triangle)
    elif hasattr(first_face, 'indices'):
        for face in scene.meshes[0].faces:
            if len(face.indices) == 3:
                triangle = MeshTriangle()
                triangle.vertex_indices = [
                    face.indices[0],
                    face.indices[1],
                    face.indices[2],
                ]
                mesh.triangles.append(triangle)
    else:
        rospy.logerr(
            'Unable to build triangles from mesh due to mesh object structure')
    for vertex in scene.meshes[0].vertices:
        point = Point()
        point.x = vertex[0] * scale[0]
        point.y = vertex[1] * scale[1]
        point.z = vertex[2] * scale[2]
        mesh.vertices.append(point)
    co.meshes = [mesh]
    pyassimp.release(scene)
    return co


def poses_list_from_yaml(filepath: str) -> List[float]:

    poses = []

    with open(filepath, 'r', encoding='utf-8') as file:
        y = safe_load(file)
        for pose in y.get('path'):
            p = [pose['position'][0], pose['position'][1], pose['position'][2],
                 pose['quaternion'][0], pose['quaternion'][1],
                 pose['quaternion'][2], pose['quaternion'][3]]
            poses.append(p)

    return poses

# universal robot conversions
# functions below from
# https://github.com/o2ac/o2ac-ur/blob/74c82a54a693bf6a3fc995ff63e7c91ac1fda6fd/catkin_ws/src/o2ac_routines/src/o2ac_routines/helpers.py#L356


def _norm2(a, b, c=0.0):
    return sqrt(a**2 + b**2 + c**2)


def ur_axis_angle_to_quat(axis_angle: List[float]) -> List[float]:
    # https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation#Unit_quaternions
    angle = _norm2(*axis_angle)
    axis_normed = [axis_angle[0] / angle,
                   axis_angle[1] / angle, axis_angle[2] / angle]
    s = sin(angle / 2)
    return [
        s * axis_normed[0],
        s * axis_normed[1],
        s * axis_normed[2],
        cos(angle / 2),
    ]  # xyzw


def quat_to_ur_axis_angle(quaternion: List[float]) -> List[float]:
    # https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation#Unit_quaternions
    # quaternion must be [xyzw]
    angle = 2 * atan2(
        _norm2(quaternion[0], quaternion[1], quaternion[2]),
        quaternion[3],
    )
    if abs(angle) > 1e-6:
        axis_normed = [
            quaternion[0] / sin(angle / 2),
            quaternion[1] / sin(angle / 2),
            quaternion[2] / sin(angle / 2),
        ]
    else:
        axis_normed = 0.0
    return [axis_normed[0] * angle, axis_normed[1] * angle,
            axis_normed[2] * angle]
