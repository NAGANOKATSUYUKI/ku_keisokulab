#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros, tf
import numpy as np
from geometry_msgs.msg import Point, TransformStamped
import controller_manager_msgs.srv
import trajectory_msgs.msg
from std_msgs.msg import Bool

class Tf_publish():
    def __init__(self):
        rospy.init_node("Head_hand_manager_node")
        self.camera_coords = np.array([1.0, 1.0, 1.0])
        self.hand_msgs = np.array([0.0, 0.0, 0.0])
        self.head_msgs = np.array([0.0, 0.0, 0.0])
        self.hand_count = 0
        self.listener = tf.TransformListener()
        self.Grasp_data = True
        self.Stop_msg = False

        rospy.Subscriber("point_head_topic", Point, self.Head_callback)
        rospy.Subscriber("point_hand_topic", Point, self.Hand_callback)
        rospy.Subscriber("Stop_msg", Bool, self.Stop_callback)
        
    #hand --->> Msg_manager
    def Hand_callback(self, hand_msg):
        try:
            self.hand_msgs[0] = hand_msg.x
            self.hand_msgs[1] = hand_msg.y
            self.hand_msgs[2] = hand_msg.z
            # rospy.loginfo("Hand --> OK")
            # rospy.loginfo("x= %0.1f, y= %0.1f, z=%0.1f", self.hand_msgs[0],self.hand_msgs[1], self.hand_msgs[2])
            self.Msg_manager()
        except:
            rospy.logwarn("sub --> NO")

    #head --->> Msg_manager
    def Head_callback(self, head_msg):
        try:
            if head_msg.x != 0.0 and head_msg.y != 0.0 or head_msg.x > head_msg.y :
                if 0.3 < head_msg.z < 1.18 :
                    self.camera_coords[0] =  head_msg.x
                    self.camera_coords[1] =  head_msg.y
                    self.camera_coords[2] =  head_msg.z
                    # rospy.loginfo("x= %0.1f, y= %0.1f, z=%0.1f", self.camera_coords[0],self.camera_coords[1], self.camera_coords[2])
                    self.Msg_manager()
                else:
                    rospy.logwarn("Distance over")
                    self.Grasp_data = None
            else:
                rospy.logwarn("Not Detection")
        except :
            rospy.logwarn("Unable to create tf")

    #Msg_manager --->> Head_Hand_Manager
    def Msg_manager(self):
        if np.any(self.hand_msgs != 0.0) or np.any(self.camera_coords != 0.0):
            self.Head_Hand_Manager()
            self.hand_msgs[0] = 0
            self.hand_msgs[1] = 0
            self.hand_msgs[2] = 0
            self.camera_coords[0] = 0
            self.camera_coords[1] = 0
            self.camera_coords[2] = 0
        else:
            pass

    #Head_Hand_Manager --->> tf_publish
    def Head_Hand_Manager(self):
        if self.hand_msgs[2] == 1:
            if self.hand_count == 10:
                rospy.loginfo("hand ----------------> move")
                
                #下を見る動き
                self.Head_controller(positions=[0.0, -0.6])
                rospy.sleep(0.1)
                # self.Omni_base_controller(positions=[0.0, 0, 0])　絶対位置になる
                self.Arm_controller(positions=[0, 0, -1.57, -1.57, 0]) 
                rospy.sleep(0.1)
                self.hand_msgs[2] = 0
                self.hand_count = 0
            else:
                self.hand_count = self.hand_count + 1
        else:
            # rospy.loginfo("head ---> move")
            self.tf_publish()

    #tf_create --->> Grasp_pub
    def tf_publish(self):
        tf_broadcaster = tf2_ros.TransformBroadcaster()

        gt = TransformStamped()
        gt.header.stamp = rospy.Time.now()
        gt.header.frame_id = "head_rgbd_sensor_link"#原点
        gt.child_frame_id = "target_frame"#対象
        gt.transform.translation.x = self.camera_coords[0]
        gt.transform.translation.y = self.camera_coords[1]
        gt.transform.translation.z = self.camera_coords[2]
        #対象の座標軸設定
        gt.transform.rotation.w = 1.0
        gt.transform.rotation.x = 0.0
        gt.transform.rotation.y = 0.0
        gt.transform.rotation.z = 0.0

        tf_broadcaster.sendTransform(gt)
        rospy.loginfo("tf_publish --> OK")
        # rospy.loginfo("%s", self.msg)
        if self.Stop_msg == True:
            # rospy.loginfo("%s",self.Stop_msg)
            rospy.loginfo("aS")
            pass
        else:
            # rospy.loginfo("%s",self.Stop_msg)
            self.Grasp_pub()
            pass

    def Stop_callback(self, stop_msg):
        self.Stop_msg = stop_msg

    #Grasp_pub  --->> msg
    def Grasp_pub(self):
        pub = rospy.Publisher("Grasp_judge", Bool, queue_size=10)

        msg = Bool()
        self.Grasp_data = True
        msg.data = self.Grasp_data
        # rospy.loginfo("%s", msg.data)

        pub.publish(msg)

    #頭を動かす
    def Head_controller(self, positions= None):
        pub = rospy.Publisher(
            '/hsrb/head_trajectory_controller/command',
            trajectory_msgs.msg.JointTrajectory, queue_size=10)

        # wait to establish connection between the controller
        while pub.get_num_connections() == 0:
            rospy.sleep(0.1)

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

        # publish ROS message
        pub.publish(traj)

    #ロボット位置
    def Robot_position(self):
        try:
            (trans, rot) = self.listener.lookupTransform("/odom", "/base_link", rospy.Time(0))

            print("Robot Position (x, y, z):", trans)
            # print("Robot Orientation (quaternion):", rot)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(f"Error: {e}")

    #移動
    def Omni_base_controller(self, positions=None):
        # initialize ROS publisher
        pub = rospy.Publisher(
            '/hsrb/omni_base_controller/command',
            trajectory_msgs.msg.JointTrajectory, queue_size=10)

        # wait to establish connection between the controller
        while pub.get_num_connections() == 0:
            rospy.sleep(0.1)

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

        # publish ROS message
        pub.publish(traj)
    
    #腕を動かす
    def Arm_controller(self, positions=None):
        # initialize ROS publisher
        pub = rospy.Publisher('/hsrb/arm_trajectory_controller/command',
                            trajectory_msgs.msg.JointTrajectory, queue_size=10)

        # wait to establish connection between the controller
        while pub.get_num_connections() == 0:
            rospy.sleep(0.1)

        # make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = (
            rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',
                            controller_manager_msgs.srv.ListControllers))
        running = False
        while running is False:
            rospy.sleep(0.1)
            for c in list_controllers().controller:
                if c.name == 'arm_trajectory_controller' and c.state == 'running':
                    running = True

        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
                            "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        
        if positions is not None:
            p = trajectory_msgs.msg.JointTrajectoryPoint()
            p.positions = positions   #     whole_body.move_to_go()の姿勢にする
            p.velocities = [0, 0, 0, 0, 0]
            p.time_from_start = rospy.Duration(3)
            traj.points = [p]
        else:
            p = trajectory_msgs.msg.JointTrajectoryPoint()
            p.positions = [0, 0, -1.57, -1.57, 0]   #     whole_body.move_to_go()の姿勢にする
            p.velocities = [0, 0, 0, 0, 0]
            p.time_from_start = rospy.Duration(3)
            traj.points = [p]

        # publish ROS message
        pub.publish(traj)
            
if __name__=="__main__":
    try:
        
        Tf_publish()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


#1126