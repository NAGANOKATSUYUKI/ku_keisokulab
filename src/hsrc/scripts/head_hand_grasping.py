#!/usr/bin/python3
# -*- coding: utf-8 -*-

import hsrb_interface
import rospy
from hsrb_interface import geometry
import controller_manager_msgs.srv
import trajectory_msgs.msg
import tf
import actionlib
import control_msgs.msg
from std_msgs.msg import Bool

rospy.init_node("Head_hand_grasp_node")

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
bottle_to_hand = geometry.pose(z=-0.16, ek=-1.57)

# handを0.1[m]上に移動させる姿勢
hand_up = geometry.pose(x=0.1)

# handを0.2[m]手前に移動させる姿勢
hand_back = geometry.pose(z=-0.2)

#
listener = tf.TransformListener()
Grasp_judge = False
Grasp_judge_count = 0
trans = 0

#頭
def Head_controller(positions= None):

        # initialize action client
        cli = actionlib.SimpleActionClient(
        '/hsrb/head_trajectory_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)

        # wait for the action server to establish connection
        cli.wait_for_server()

        # make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy(
        '/hsrb/controller_manager/list_controllers',
        controller_manager_msgs.srv.ListControllers)
        running = False
        while running is False:
                rospy.sleep(0.1)
                for c in list_controllers().controller:
                        if c.name == 'head_trajectory_controller' and c.state == 'running':
                                running = True

        # fill ROS message
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        
        if positions is not None:
                p = trajectory_msgs.msg.JointTrajectoryPoint()
                p.positions = positions
                p.velocities = [0, 0]
                p.time_from_start = rospy.Duration(3)
                traj.points = [p]
        else:
                p = trajectory_msgs.msg.JointTrajectoryPoint()
                p.positions = [0.0, -0.6]
                p.velocities = [0, 0]
                p.time_from_start = rospy.Duration(3)
                traj.points = [p]
        
        goal.trajectory = traj

        # send message to the action server
        cli.send_goal(goal)

        # wait for the action server to complete the order
        cli.wait_for_result()

#ロボット位置
def Robot_position():
        try:
                (trans, rot) = listener.lookupTransform("/odom", "/base_link", rospy.Time(0))

                print("Robot Position (x, y, z):", trans)
                # print("Robot Orientation (quaternion):", rot)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print(f"Error: {e}")

#ロボットアーム
def Robot_arm():
        global trans
        try:
                (trans, rot) = listener.lookupTransform("/odom", "/target_frame", rospy.Time(0))

                # print("Robot Position (x, y, z):", trans)
                # print("Robot Orientation (quaternion):", rot)

        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        #         print(f"Error: {e}")
        except:
                rospy.loginfo(" Not_Tf ")

#移動
def Omni_base_controller(positions=None):

        # initialize action client
        cli = actionlib.SimpleActionClient(
        '/hsrb/omni_base_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)

        # wait for the action server to establish connection
        cli.wait_for_server()

        # make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy(
        '/hsrb/controller_manager/list_controllers',
        controller_manager_msgs.srv.ListControllers)
        running = False
        while running is False:
                rospy.sleep(0.1)
                for c in list_controllers().controller:
                        if c.name == 'omni_base_controller' and c.state == 'running':
                                running = True

        # fill ROS message
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["odom_x", "odom_y", "odom_t"]
        if positions is not None:
                p = trajectory_msgs.msg.JointTrajectoryPoint()
                p.positions = positions
                p.velocities = [0, 0, 0]
                p.time_from_start = rospy.Duration(15)
                traj.points = [p]
        else:
                p = trajectory_msgs.msg.JointTrajectoryPoint()
                p.positions = [-0.2, 0, 0]
                p.velocities = [0, 0, 0]
                p.time_from_start = rospy.Duration(15)
                traj.points = [p]
        goal.trajectory = traj

        # send message to the action server
        cli.send_goal(goal)

        # wait for the action server to complete the order
        cli.wait_for_result()

#体
def Arm_controller(positions=None):

        # initialize action client
        cli = actionlib.SimpleActionClient(
        '/hsrb/arm_trajectory_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)

        # wait for the action server to establish connection
        cli.wait_for_server()

        # make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy(
        '/hsrb/controller_manager/list_controllers',
        controller_manager_msgs.srv.ListControllers)
        running = False
        while running is False:
                rospy.sleep(0.1)
                for c in list_controllers().controller:
                        if c.name == 'arm_trajectory_controller' and c.state == 'running':
                                running = True

        # fill ROS message
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
                        "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

        if positions is not None:
                p = trajectory_msgs.msg.JointTrajectoryPoint()
                p.positions = positions   
                p.time_from_start = rospy.Duration(3)
                traj.points = [p]
                goal.trajectory = traj
        else:
                p = trajectory_msgs.msg.JointTrajectoryPoint()
                p.positions = [0, 0, -1.57, -1.57, 0]   #     whole_body.move_to_go()の姿勢  [0,0,0, -1.57,0]--> neutral
                p.velocities = [0, 0, 0, 0, 0]
                p.time_from_start = rospy.Duration(3)
                traj.points = [p]
                goal.trajectory = traj

        # send message to the action server
        cli.send_goal(goal)

        # wait for the action server to complete the order
        cli.wait_for_result()

#ハンド
def Hand_controller(positions=None):
        
        # initialize action client
        cli = actionlib.SimpleActionClient(
        '/hsrb/gripper_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)

        # wait for the action server to establish connection
        cli.wait_for_server()

        # make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy(
        '/hsrb/controller_manager/list_controllers',
        controller_manager_msgs.srv.ListControllers)
        running = False
        while running is False:
                rospy.sleep(0.1)
                for c in list_controllers().controller:
                        if c.name == 'gripper_controller' and c.state == 'running':
                                running = True

        # fill ROS message
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["hand_motor_joint"]
        if positions is not None:
                p = trajectory_msgs.msg.JointTrajectoryPoint()
                p.positions = positions
                p.velocities = [0]
                p.effort = [0.6]
                p.time_from_start = rospy.Duration(3)
                traj.points = [p]
        else:
                p = trajectory_msgs.msg.JointTrajectoryPoint()
                p.positions = [0.5]
                p.velocities = [0]
                p.effort = [0.1]
                p.time_from_start = rospy.Duration(3)
                traj.points = [p]
        goal.trajectory = traj

        # send message to the action server
        cli.send_goal(goal)

        # wait for the action server to complete the order
        cli.wait_for_result()

#Tf_grasp
def Tf_grasp():
        # whole_body.move_end_effector_pose([geometry.pose(x=trans[0]), geometry.pose(y=trans[1]), geometry.pose(z=trans[2])], ref_frame_id='hand_palm_link')
        whole_body.move_end_effector_pose(bottle_to_hand, _BOTTLE_TF)
        whole_body.move_end_effector_pose(geometry.pose(z = 0.1), _HAND_TF)

#Stop_msg
def Stop_msg(Stop_data):
        pub = rospy.Publisher("Stop_msg", Bool, queue_size=10)

        msg = Bool()
        msg.data = Stop_data

        pub.publish(msg)

#
def Grasp_judge_callback(Grasp_msg):
    global Grasp_judge
    Grasp_judge = Grasp_msg.data
    return Grasp_judge

if __name__=='__main__':
    sub_Grasp_judge = rospy.Subscriber('Grasp_judge', Bool, Grasp_judge_callback)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        rospy.loginfo("Received: {}".format(Grasp_judge))
        # Robot_arm()
        if Grasp_judge == True:
            if Grasp_judge_count  == 30:
                try:
                        Stop_msg(True)      #Stop_True
                        Hand_controller(positions=[0.0])
                        rospy.sleep(3.0)
                        Hand_controller(positions=[1.2])
                        # Arm_controller(positions=[0.05,0,0,-1.57,0])                         # --> 少し体を上げる
                        # Head_controller(positions=[0, -0.2])  #床上のボトルが見えなくなる


                        rospy.sleep(5.0)
                        Tf_grasp()

                        rospy.sleep(5.0)
                        Hand_controller(positions=[0.0])
                        rospy.loginfo("Grasp_Bottle --> OK")

                        rospy.sleep(5.0)
                        # 手先相対で上にハンドを移動
                        whole_body.move_end_effector_pose(hand_up, _HAND_TF)
                        # 手先相対で後ろにハンドを移動
                        whole_body.move_end_effector_pose(hand_back, _HAND_TF)
                        
                        rospy.sleep(2.0)
                        Arm_controller(positions=[0,0,0, -1.57,0])                              #  --> neutral

                        rospy.sleep(5.0)
                        omni_base.go_abs(0.0, 0.0, 0.0, 20.0)
                        # Omni_base_controller(positions=[0,0,0])
                        rospy.loginfo("初期位置 --> OK")

                        rospy.sleep(5.0)
                        Omni_base_controller(positions=[0,0,1.57])
                        Arm_controller(positions=[0.15,-1.5,0,-1.3,0])
                        rospy.sleep(3.0)
                        Hand_controller(positions=[1.2])
                        Arm_controller(positions=[0,0,0, -1.57,0])                              #  --> neutral
                        rospy.loginfo("箱に入れる --> OK")
                        rospy.sleep(3.0)
                        Omni_base_controller(positions=[0,0,0])
                        Arm_controller(positions=[0,0,0, -1.57,0])
                        Head_controller(positions=[0,0])

                        Grasp_judge = False
                        Grasp_judge_count = 0
                        Stop_msg(False)      #Stop_False

                except Exception as e:
                        rospy.logerr(f'Fail to grasp. Error: {e}')
                        Arm_controller(positions=[0,0,0, -1.57,0])                              #  --> neutral
                        Grasp_judge = False
                        Grasp_judge_count = 0
            else:
                   Grasp_judge_count = Grasp_judge_count +1
                   pass                       

        else:
            #    rospy.logwarn("Judge --> false")
               pass
        
        rate.sleep()