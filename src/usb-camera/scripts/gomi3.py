#!/usr/bin/python3
# -*- coding: utf-8 -*-

import hsrb_interface
import rospy
import sys
from hsrb_interface import geometry


# 把持力[N]
_GRASP_FORCE=0.2
# ボトルのtf名
_BOTTLE_TF='target_frame'
# グリッパのtf名
_HAND_TF='hand_palm_link'

# ロボット機能を使うための準備
robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
gripper = robot.get('gripper')
tts = robot.get('default_tts')

# bottleのマーカの手前0.02[m],z軸回に-1.57回転させた姿勢
bottle_to_hand = geometry.pose(z=-0.02, ek=-1.57)

# handを0.1[m]上に移動させる姿勢
hand_up = geometry.pose(x=0.1)

# handを0.5[m]手前に移動させる姿勢
hand_back = geometry.pose(z=-0.5)


if __name__=='__main__':
    
    while True:
        try:
                gripper.command(1.0)
                whole_body.move_to_go()
        except:
                # tts.say('初期化に失敗')
                rospy.logerr('fail to init')
                # sys.exit()

        try:
                # 把持用初期姿勢に遷移
                whole_body.move_to_neutral()
                # 遷移後に手先を見るようにする
                whole_body.looking_hand_constraint = True
                # ペットボトルの手前に手を持ってくる
                whole_body.move_end_effector_pose(bottle_to_hand, _BOTTLE_TF)
                # 力を指定して把持する
                gripper.apply_force(_GRASP_FORCE)
                # シミュレータのgrasp hackのための待ち時間。実機では不要
                rospy.sleep(2.0)
                # 手先相対で上にハンドを移動
                whole_body.move_end_effector_pose(hand_up, _HAND_TF)
                # 手先相対で後ろにハンドを移動
                whole_body.move_end_effector_pose(hand_back, _HAND_TF)
                # 初期姿勢に遷移
                whole_body.move_to_neutral()
        except:
                # tts.say('把持失敗')
                rospy.logerr('fail to grasp')
                # sys.exit()


# import actionlib
# import control_msgs.msg
# import controller_manager_msgs.srv
# import rospy
# import trajectory_msgs.msg

# rospy.init_node('test')

# def Arm():
#         # initialize action client
#         cli = actionlib.SimpleActionClient(
#         '/hsrb/gripper_controller/follow_joint_trajectory',
#         control_msgs.msg.FollowJointTrajectoryAction)

#         # wait for the action server to establish connection
#         cli.wait_for_server()

#         # make sure the controller is running
#         rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
#         list_controllers = rospy.ServiceProxy(
#         '/hsrb/controller_manager/list_controllers',
#         controller_manager_msgs.srv.ListControllers)
#         running = False
#         while running is False:
#                 rospy.sleep(0.1)
#                 for c in list_controllers().controller:
#                         if c.name == 'gripper_controller' and c.state == 'running':
#                                 running = True

#         # fill ROS message
#         goal = control_msgs.msg.FollowJointTrajectoryGoal()
#         traj = trajectory_msgs.msg.JointTrajectory()
#         traj.joint_names = ["hand_motor_joint"]
#         p = trajectory_msgs.msg.JointTrajectoryPoint()
#         p.positions = [0.5]
#         p.velocities = [0]
#         p.effort = [0.1]
#         p.time_from_start = rospy.Duration(3)
#         traj.points = [p]
#         goal.trajectory = traj

#         # send message to the action server
#         cli.send_goal(goal)

#         # wait for the action server to complete the order
#         cli.wait_for_result()

# if __name__=='__main__':
#         try:
#                 Arm()

#         except rospy.ROSInterruptException:
#                 pass
        