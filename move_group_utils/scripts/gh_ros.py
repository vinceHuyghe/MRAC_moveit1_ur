import rospy

from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from move_group_utils.move_group_utils import (MoveGroupUtils,
                                               Sequence,
                                               pose_from_list,
                                               publish_pose_array,
                                               poses_from_yaml)
from move_group_utils.move_group_utils import Circ, Lin, Ptp, from_euler



j_acc = 0.5
c_vel = 0.5
c_acc = 0.01
c_blend = 0.005

home = Ptp(goal=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
           vel_scale=0.5,
           acc_scale=0.3)


def robot_program(mgi: MoveGroupUtils):

    # poses_list = rospy.get_param('gh_poses')

    poses_list = poses_from_yaml(
        "/dev_ws/src/move_group_utils/toolpaths/poses.yaml")

    poses = [pose_from_list(pose) for pose in poses_list]
    # TODO
    publish_pose_array('base_link',poses)

    approach = Pose(position=Point(poses[0].position.x,
                                   poses[0].position.y,
                                   poses[0].position.z + 0.2),
                    orientation=(poses[0].orientation))

    path = Sequence()

    mgi.sequencer.execute()

    for pose in poses:
        path.append(Lin(goal=pose, vel_scale=c_vel,
                    acc_scale=c_acc), blend_radius=c_blend)
    path.append(Lin(goal=poses[0], vel_scale=c_vel,
                acc_scale=c_acc), blend_radius=c_blend)

    path.insert(0, Ptp(goal=approach, vel_scale=c_vel, acc_scale=j_acc))
    path.insert(len(path.items), Lin(goal=approach,
                vel_scale=c_vel, acc_scale=c_acc))

    

    success, plan = mgi.sequencer.plan(path)[:2]
    
    mgi.display_trajectory(plan)
    if not success:
        return rospy.loginfo(f'{mgi.name}: robot program aborted, planning failed')
       
    mgi.display_trajectory(plan)
    
    mgi.sequencer.execute()

    mgi.sequencer.plan(home)
    mgi.sequencer.execute()


if __name__ == "__main__":

    mgi = MoveGroupUtils()
    try:
        robot_program(mgi)
    except rospy.ROSInterruptException:
        pass
