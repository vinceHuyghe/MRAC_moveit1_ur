#!/usr/bin/env python3

from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from industrial_reconstruction_msgs.msg import NormalFilterParams
from industrial_reconstruction_msgs.srv import (StartReconstruction,
                                                StartReconstructionRequest,
                                                StopReconstruction,
                                                StopReconstructionRequest)

from move_group_utils.move_group_utils import MoveGroupUtils
from pilz_robot_program.pilz_robot_program import Lin, Ptp, Sequence

# define robot poses
home = (0.0, -pi/2.0, pi/2.0, -pi, -pi/2, 0)
pose1 = Pose(position=Point(0.6, -0.3, 0.12),
             orientation=Quaternion(0.0, 1.0, 0.0, 0.0))
pose2 = Pose(position=Point(0.6, 0.3, 0.12),
             orientation=Quaternion(0.0, 1.0, 0.0, 0.0))

# define endeffector
# tcp pose should match static tf declared in launch file
ee_name = 'D405'
tcp_pose = Pose(position=Point(0.0, 0.0, 0.045),
                orientation=Quaternion(0, 0, 0, 1))
size = [0.042, 0.042, 0.023]

# reconstruction parameters
start_srv_req = StartReconstructionRequest()
start_srv_req.tracking_frame = 'camera_depth_optical_frame'
start_srv_req.relative_frame = 'base_link'
start_srv_req.translation_distance = 0.0
start_srv_req.rotational_distance = 0.0
start_srv_req.live = True
start_srv_req.tsdf_params.voxel_length = 0.001
start_srv_req.tsdf_params.sdf_trunc = 0.002
start_srv_req.tsdf_params.min_box_values = Vector3(x=0.0, y=0.0, z=0.0)
start_srv_req.tsdf_params.max_box_values = Vector3(x=0.0, y=0.0, z=0.0)
start_srv_req.rgbd_params.depth_scale = 1000
start_srv_req.rgbd_params.depth_trunc = 0.25
start_srv_req.rgbd_params.convert_rgb_to_intensity = False

stop_srv_req = StopReconstructionRequest()
# stop_srv_req.archive_directory = '/dev_ws/src.reconstruction/'
stop_srv_req.mesh_filepath = '/home/v/test.ply'
# stop_srv_req.normal_filters = [NormalFilterParams(
#                     normal_direction=Vector3(x=0.0, y=0.0, z=1.0), angle=90)]
# stop_srv_req.min_num_faces = 1000


def robot_program():

    # initialize node and moveit commander
    mgi = MoveGroupUtils()
    
    # wait for rviz and moveit to start
    rospy.sleep(3.0)
    
    # add collision object
    mgi.add_ground_cube()

    # wait for reconstruction services
    rospy.wait_for_service('/start_reconstruction', timeout=10.0)

    # define service clients
    start_recon = rospy.ServiceProxy(
        '/start_reconstruction', StartReconstruction)
    stop_recon = rospy.ServiceProxy('/stop_reconstruction', StopReconstruction)

    # publish poses for visualization
    mgi.publish_pose_array([pose1, pose2])

    # attach camera and set new tcp
    mgi.attach_camera(ee_name, tcp_pose, size)
    mgi.move_group.set_end_effector_link(f'{ee_name}/tcp')
    rospy.loginfo(
        f'{mgi.name}: end effector link set to {mgi.move_group.get_end_effector_link()}'
    )

    # Move into position to start reconstruction
    mgi.sequencer.plan(Ptp(goal=home, vel_scale=0.1, acc_scale=0.3))
    mgi.sequencer.execute()
    mgi.sequencer.plan(Ptp(goal=pose1, vel_scale=0.1, acc_scale=0.3))
    mgi.sequencer.execute()

    # Start reconstruction with service srv_req
    resp = start_recon(start_srv_req)

    if resp:
        rospy.loginfo('robot program: reconstruction started successfully')
    else:
        rospy.loginfo('robot program: failed to start reconstruction')

    mgi.sequencer.plan(Lin(goal=pose2, vel_scale=0.001, acc_scale=0.001))
    mgi.sequencer.execute()

    # Stop reconstruction with service srv_req
    resp = stop_recon(stop_srv_req)

    if resp:
        rospy.loginfo('robot program: reconstruction stopped successfully')
    else:
        rospy.loginfo('robot program: failed to stop reconstruction')


if __name__ == '__main__':

    robot_program()
