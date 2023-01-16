#!/usr/bin/env python3
from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from industrial_reconstruction_msgs.srv import (StartReconstruction,
                                                StartReconstructionRequest,
                                                StopReconstruction,
                                                StopReconstructionRequest)
from moveit_commander.conversions import list_to_pose

from move_group_utils.move_group_utils import (MoveGroupUtils,
                                               poses_list_from_yaml,
                                               publish_trajectory_markers)
from pilz_robot_program.pilz_robot_program import Lin, Ptp, Sequence

# define poses
home = (0.0, -pi/2.0, pi/2.0, -pi, -pi/2, 0)

pose_list = poses_list_from_yaml(
    '/dev_ws/src/ur10e_examples/toolpaths/scan.yaml')
toolpath = [list_to_pose(pose) for pose in pose_list]

# define end effector
# tcp pose should match static tf declared in launch file
ee_name = 'D405'
tcp_pose = Pose(position=Point(0.0, 0.0, 0.045),
                orientation=Quaternion(0, 0, 0, 1))
size = [0.042, 0.042, 0.023]

# define reconstruction srv msgs
start_srv_req = StartReconstructionRequest()
start_srv_req.tracking_frame = 'camera_depth_optical_frame'
start_srv_req.relative_frame = 'base_link'
start_srv_req.translation_distance = 0.0
start_srv_req.rotational_distance = 0.0
# start_srv_req.live = False
# start_srv_req.tsdf_params.voxel_length = 0.0005
# start_srv_req.tsdf_params.sdf_trunc = 0.001
start_srv_req.live = False
start_srv_req.tsdf_params.voxel_length = 0.001
start_srv_req.tsdf_params.sdf_trunc = 0.004
start_srv_req.tsdf_params.min_box_values = Vector3(x=0.0, y=0.0, z=0.0)
start_srv_req.tsdf_params.max_box_values = Vector3(x=0.0, y=0.0, z=0.0)
start_srv_req.rgbd_params.depth_scale = 1000
start_srv_req.rgbd_params.depth_trunc = 0.15
start_srv_req.rgbd_params.convert_rgb_to_intensity = False

stop_srv_req = StopReconstructionRequest()
stop_srv_req.mesh_filepath = '/home/v/test.ply'

# define speed and acceleration

move_vel = 0.5
move_acc = 0.5

# fine scan
# scan_vel = 0.05
# scan_acc = 0.0002

# fast scan
scan_vel = 0.05
scan_acc = 0.01

# boolean to enable/disable reconstruction
# for testing purposes
scan = True


def robot_program():

    mgi = MoveGroupUtils()
    rospy.sleep(3)

    if scan:
        rospy.wait_for_service('/start_reconstruction', timeout=10)
        start_recon = rospy.ServiceProxy(
            '/start_reconstruction', StartReconstruction)
        stop_recon = rospy.ServiceProxy(
            '/stop_reconstruction', StopReconstruction)

    # add table collision table
    mgi.add_ground_cube()

    # attach camera
    mgi.attach_camera(ee_name, tcp_pose, size)
    mgi.move_group.set_end_effector_link(f'{ee_name}/tcp')

    # visualize the toolpath
    mgi.publish_pose_array(toolpath)

    # move home
    success, plan = mgi.sequencer.plan(Ptp(goal=home,
                                       vel_scale=move_vel,
                                       acc_scale=move_acc))[:2]
    if not success:
        return rospy.logerr('robot program: failed to plan to home position')
    mgi.sequencer.execute(plan)

    # move approach
    success, plan = mgi.sequencer.plan(Ptp(goal=toolpath[0],
                                       vel_scale=move_vel,
                                       acc_scale=move_acc))[:2]
    if not success:
        return rospy.logerr('robot program: failed to plan to approach position')
    mgi.sequencer.execute(plan)

    # scanning sequence
    sequence = Sequence()
    for pose in toolpath[1:-1]:
        sequence.append(Lin(goal=pose,
                            vel_scale=scan_vel,
                            acc_scale=scan_acc),
                        blend_radius=0.01)

    sequence.append(Lin(goal=toolpath[-1],
                    vel_scale=scan_vel,
                    acc_scale=scan_acc))

    success, plan = mgi.sequencer.plan(sequence)[:2]
    if not success:
        return rospy.logerr('robot program: failed to plan to sequence')
    publish_trajectory_markers(plan[0])

    if scan:
        # start reconstruction
        resp = start_recon(start_srv_req)
        if not resp:
            rospy.loginfo('robot program: failed to start reconstruction')
        rospy.loginfo('robot program: started reconstruction')

    mgi.sequencer.execute(plan)

    if scan:
        # stop reconstruction
        resp = stop_recon(stop_srv_req)
        if not resp:
            rospy.loginfo('robot program: failed to stop reconstruction')
        rospy.loginfo('robot program: reconstruction stopped successfully')

    # return home
    success, plan = mgi.sequencer.plan(
        Ptp(goal=home, vel_scale=move_vel, acc_scale=move_acc))[:2]
    if not success:
        return rospy.logerr('Failed to plan to home position')
    mgi.sequencer.execute(plan)

    return rospy.loginfo('robot program: program completed')


if __name__ == '__main__':

    robot_program()
