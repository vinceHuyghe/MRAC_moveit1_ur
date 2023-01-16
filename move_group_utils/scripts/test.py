#!/usr/bin/env python3

from math import pi

from geometry_msgs.msg import Point, Pose, Quaternion
from move_group_utils.move_group_utils import MoveGroupUtils
from move_group_utils.move_group_utils import Circ, Lin, Ptp, from_euler
from move_group_utils.srv import VisualizeTrajectory
import rospy

def robot_program():

    mgi = MoveGroupUtils()
    
    rospy.sleep(3.0)

    start = (0.0, -pi / 2.0, pi / 2.0, 0.0, pi / 2.0, 0.0)
    pose_l = Pose(position=Point(1, -0.6, 0.3),
                  orientation=from_euler(0.0, pi, 0.0))
    pose_r = Pose(position=Point(1.6, 0.6, 1.0),
                  orientation=from_euler(0.0, pi, 0.0))

    poses = [start, pose_l, pose_r]
       
    success, plan = mgi.sequencer.plan(Ptp(goal=start, vel_scale=0.3, acc_scale=0.3))[:2]
    
    srv = rospy.ServiceProxy('/mgu/visualize_trajectory', VisualizeTrajectory)
    srv.call(plan[0])
    
    mgi.sequencer.execute()
    

if __name__ == '__main__':

    robot_program()
 