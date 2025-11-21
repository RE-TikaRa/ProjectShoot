#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import math
import actionlib
import serial
import time
from std_msgs.msg import String, Int32
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Point
from tf_conversions import transformations
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
import os

# 全局变量定义
serialPort = "/dev/shoot"
baudRate = 9600
ser = None

music_path = "~/07.mp3"
id = 255
flog0 = 255
flog1 = 255
flog2 = 255
flog3 = 255
flog4 = 255
count = 0
time_var = 0  # 避免与内置 time 模块冲突
move_flog = 0
Yaw_th = 0.0055 # Yaw_th = 0.0055 0.005
Min_y = -0.55 # -0.55
Max_y = -0.40 # -0.40
# Min_y = -0.23
# Max_y = -0.21
Yaw_th1 = 0.03 # 移动靶瞄准阈值，从0.006放宽到0.008，更容易射击
ar_flog = 255
case = 255
case1 = 255
case2 = 255
case3 = 255
target_id_moving = 255
target_id_moving_2 = 255


class navigation_demo:
    def __init__(self):
        global ser
        self.is_navigating = False  # 导航状态标志，防止导航时瞄准
        self.find_cb_executed = False
        self.ar_cb_executed_case1 = False
        self.ar_cb_executed_case2 = False
        self.ar_cb_executed_case3 = False
        self.ar_cb_executed_case4 = False
        self.ar_cb_executed_case5 = False
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.arrive_pub = rospy.Publisher('/voiceWords', String, queue_size=10)
        self.find_sub = rospy.Subscriber('/object_position', Point, self.find_cb)
        self.ar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_cb)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)
        self.target_id_moving_sub = rospy.Subscriber('target_id_moving', Int32, self.target_id_moving_callback)
        self.target_id_moving_2_sub = rospy.Subscriber('target_id_moving_2', Int32, self.target_id_moving_2_callback)
        
        # 超时机制相关变量
        self.case_start_time = None  # 当前case的开始时间
        self.timeout_threshold = 12  # 超时阈值（秒）
        self.timeout_check_enabled = False  # 是否启用超时检查
        
        # 目标丢失检测变量
        self.last_target_time = None  # 最后一次接收到目标的时间
        self.target_lost_threshold = 0.5  # 目标丢失阈值（秒）
        self.target_tracking_enabled = False  # 是否启用目标跟踪检测
        
        # 初始化串口
        try:
            ser = serial.Serial(port=serialPort, baudrate=baudRate, parity="N", bytesize=8, stopbits=1)
            rospy.loginfo("串口初始化成功: %s", serialPort)
        except Exception as e:
            rospy.logerr("串口初始化失败: %s", str(e))
            ser = None

    def check_timeout(self):
        """检查当前case是否超时"""
        if not self.timeout_check_enabled or self.case_start_time is None:
            return False
        
        elapsed_time = time.time() - self.case_start_time
        if elapsed_time > self.timeout_threshold:
            rospy.logwarn("识别超时！已等待 %.1f 秒，超过阈值 %.1f 秒", elapsed_time, self.timeout_threshold)
            return True
        return False
    
    def check_target_lost(self):
        """检查目标是否丢失"""
        if not self.target_tracking_enabled or self.last_target_time is None:
            return False
        
        elapsed_time = time.time() - self.last_target_time
        if elapsed_time > self.target_lost_threshold:
            return True
        return False
    
    def stop_robot(self):
        """停止机器人运动"""
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.pub.publish(msg)
    
    def timeout_monitor(self, event):
        """独立的超时监控定时器回调 - 不依赖数据回调触发"""
        global case
        
        # 只在启用超时检查且不在导航状态时监控
        if not self.timeout_check_enabled or self.is_navigating:
            return
        
        # 检查是否超时
        if self.check_timeout():
            rospy.logwarn("超时监控器触发！case=%d", case)
            self.handle_timeout_skip()
    
    def target_lost_monitor(self, event):
        """目标丢失监控定时器回调"""
        global case
        # 只在固定靶任务时监控（case 0）
        if case != 0:
            return
        
        if self.check_target_lost():
            rospy.logwarn("目标丢失！已 %.2f 秒未检测到目标，停止旋转", 
                          time.time() - self.last_target_time)
            self.stop_robot()
            # 更新时间戳，避免重复打印日志
            self.last_target_time = time.time()
    
    def handle_timeout_skip(self):
        """处理超时跳转到下一个case"""
        global case
        rospy.logwarn("超时跳过当前任务点，case=%d", case)
        
        # 停止机器人运动
        msg = Twist()
        self.pub.publish(msg)
        
        # 根据当前case跳转到下一个
        if case == 0:
            rospy.logwarn("Case 0 超时，跳过第一个固定目标，导航到下一组目标")
            self.find_cb_executed = True
            self.goto(goals[1])
            case = 1
        elif case == 1:
            rospy.logwarn("Case 1 超时，跳过移动靶(第一次)，导航到下一个位置")
            self.ar_cb_executed_case1 = True
            self.goto(goals[2])
            case = 2
        elif case == 2:
            rospy.logwarn("Case 2 超时，跳过移动靶(第二次)，导航到动态目标位置")
            self.ar_cb_executed_case2 = True
            self.goto(goals[3])
            case = 3
        elif case == 3:
            rospy.logwarn("Case 3 超时，跳过动态移动靶1，导航到下一个位置")
            self.ar_cb_executed_case3 = True
            self.goto(goals[4])
            case = 4
        elif case == 4:
            rospy.logwarn("Case 4 超时，跳过动态移动靶2，导航到goals[6]")
            self.ar_cb_executed_case4 = True
            self.goto(goals[5])  # 中转点
            self.goto(goals[6])  # 最终位置
            case = 5
        elif case == 5:
            rospy.logwarn("Case 5 超时，跳过target_id_moving_2，直接结束")
            self.ar_cb_executed_case5 = True
            case = 6
            self.end()
        
        # 重置超时计时器
        self.case_start_time = time.time()
        rospy.loginfo("已跳转到下一个case=%d", case)
    
    def end(self):
        self.goto(goals[7])

        global time_var
        time_var = 0  # 重置计数器
        msg = Twist()
        msg.linear.x = -0.4
        msg.linear.y = 0.25
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        while time_var <= 11:
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time_var += 1
            # 比赛结束语音
        # os.system('mplayer /home/abot/SU5WNX_ws/src/robot_slam/scripts/end.mp3')

    def find_cb(self, data):
        # 导航期间不进行瞄准，避免与move_base冲突
        if self.is_navigating:
            return
        
        global id, flog0, flog1, flog2, count, move_flog, case
        
        # 只在 case=0 时处理固定靶数据，其他case使用AR标记识别
        if case != 0:
            return
        
        # 更新目标接收时间戳（用于目标丢失检测）
        self.last_target_time = time.time()
            
        id = 255
        point_msg = data
        flog0 = point_msg.x - 315 # flog0 = point_msg.x - 320
        flog1 = abs(flog0)

        rospy.loginfo("接收到目标位置: x=%f, 偏差=%f, case=%d", point_msg.x, flog0, case)

        # Case 0 - 第一个固定目标
        if abs(flog1) > 5 and case == 0:  # 从3放宽到5像素，更容易达到射击条件
            if self.find_cb_executed:
                return
            msg = Twist()
            msg.angular.z = -0.013 * flog0 #0.013
            self.pub.publish(msg)
            rospy.loginfo("调整角度: angular.z=%f, 偏差=%f", msg.angular.z, flog1)
        elif abs(flog1) <= 5 and case == 0:  # 从3放宽到5像素
            if self.find_cb_executed:
                return
            rospy.loginfo("开始射击，目标对准")
            try:
                if ser is None or not ser.is_open:
                    rospy.logerr("串口未打开，无法射击")
                    return
                ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
                rospy.loginfo("射击指令已发送")
                rospy.sleep(0.2)
                ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
                rospy.loginfo("停止射击指令已发送")
                self.find_cb_executed = True
                # 使用线程避免阻塞回调
                rospy.loginfo("开始导航到下一组目标点")
                self.goto(goals[1])
                    # if self.goto(goals[2]):
                    #     if self.goto(goals[3]):
                    #         self.goto(goals[4])
                rospy.sleep(1)
                case = 1
            except Exception as e:
                rospy.logerr("射击失败: %s", str(e))
                self.find_cb_executed = False

    def ar_cb(self, data):
        # 导航期间不进行瞄准，避免与move_base冲突
        if self.is_navigating:
            return
            
        global target_id_moving, target_id_moving_2, case
        ar_markers = data
        rospy.loginfo("检测到 %d 个AR标记, 当前case=%d", len(data.markers), case)

        for marker in data.markers:
            rospy.loginfo("AR标记ID: %d, 移动ID1: %d, 移动ID2: %d",
                          marker.id, target_id_moving, target_id_moving_2)

            if marker.id == 1 and case == 1:
                if self.ar_cb_executed_case1:
                    return
                ar_x_0 = marker.pose.pose.position.x
                ar_x_0_abs = abs(ar_x_0)
                rospy.loginfo("移动目标(ID=1) - X偏差: %f (阈值: %f)", ar_x_0_abs, Yaw_th1)
                if ar_x_0_abs >= Yaw_th1:
                    msg = Twist()
                    msg.angular.z = -0.7 * ar_x_0
                    self.pub.publish(msg)
                    rospy.loginfo("调整角度对准移动目标: angular.z=%f", msg.angular.z)
                elif ar_x_0_abs < Yaw_th1:
                    rospy.loginfo("移动目标(第一次)对准成功，开始射击")
                    try:
                        if ser is None or not ser.is_open:
                            rospy.logerr("串口未打开，无法射击")
                            return
                        self.ar_cb_executed_case1 = True
                        ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
                        rospy.loginfo("移动目标(第一次)射击指令已发送")
                        rospy.sleep(0.32)
                        ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
                        rospy.sleep(2)
                        case = 2
                        rospy.loginfo("切换到case=2, 导航到第二个位置")
                        self.goto(goals[2])
                            # self.goto(goals[6])
                        rospy.sleep(1)
                    except Exception as e:
                        rospy.logerr("移动目标(第一次)射击失败: %s", str(e))
                        self.ar_cb_executed_case1 = False

            if marker.id == 1 and case == 2:
                if self.ar_cb_executed_case2:
                    return
                ar_x_0 = marker.pose.pose.position.x
                ar_x_0_abs = abs(ar_x_0)
                rospy.loginfo("移动目标(ID=1,第二次) - X偏差: %f (阈值: %f)", ar_x_0_abs, Yaw_th1)
                if ar_x_0_abs >= Yaw_th1:
                    msg = Twist()
                    msg.angular.z = -0.7 * ar_x_0
                    self.pub.publish(msg)
                    rospy.loginfo("调整角度对准移动目标(第二次): angular.z=%f", msg.angular.z)
                elif ar_x_0_abs < Yaw_th1:
                    rospy.loginfo("移动目标(第二次)对准成功，开始射击")
                    try:
                        if ser is None or not ser.is_open:
                            rospy.logerr("串口未打开，无法射击")
                            return
                        self.ar_cb_executed_case2 = True
                        ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
                        rospy.loginfo("移动目标(第二次)射击指令已发送")
                        rospy.sleep(0.32)
                        ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
                        rospy.sleep(2)
                        case = 3
                        rospy.loginfo("切换到case=3, 寻找动态目标")
                        self.goto(goals[3])
                        rospy.sleep(1)
                    except Exception as e:
                        rospy.logerr("移动目标(第二次)射击失败: %s", str(e))
                        self.ar_cb_executed_case2 = False

            if marker.id == target_id_moving and case == 3:
                if target_id_moving == 255:
                    rospy.logwarn("target_id_moving未设置，跳过")
                    return
                if self.ar_cb_executed_case3:
                    return
                ar_x_0 = marker.pose.pose.position.x
                ar_x_0_abs = abs(ar_x_0)
                rospy.loginfo("移动目标(动态ID,第一个) - X偏差: %f (阈值: %f)", ar_x_0_abs, Yaw_th1)
                if ar_x_0_abs >= Yaw_th1:
                    msg = Twist()
                    msg.angular.z = -0.7 * ar_x_0  # 0.5
                    self.pub.publish(msg)
                    rospy.loginfo("调整角度对准移动目标(动态ID,第一个): angular.z=%f", msg.angular.z)
                elif ar_x_0_abs < Yaw_th1:
                    rospy.loginfo("移动目标(动态ID,第一个)对准成功，开始射击")
                    try:
                        if ser is None or not ser.is_open:
                            rospy.logerr("串口未打开，无法射击")
                            return
                        self.ar_cb_executed_case3 = True
                        ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
                        rospy.loginfo("移动目标(动态ID,第一个)射击指令已发送")
                        rospy.sleep(0.32)
                        ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
                        rospy.sleep(2)
                        case = 4
                        rospy.loginfo("切换到case=4, 寻找第二个动态目标")
                        self.goto(goals[4])
                        rospy.sleep(1)
                    except Exception as e:
                        rospy.logerr("移动目标(动态ID,第一个)射击失败: %s", str(e))
                        self.ar_cb_executed_case3 = False

            if marker.id == target_id_moving and case == 4:
                if target_id_moving == 255:
                    rospy.logwarn("target_id_moving未设置，跳过")
                    return
                if self.ar_cb_executed_case4:
                    return
                ar_x_0 = marker.pose.pose.position.x
                ar_x_0_abs = abs(ar_x_0)
                rospy.loginfo("移动目标(动态ID,第二个位置) - X偏差: %f (阈值: %f)", ar_x_0_abs, Yaw_th1)
                if ar_x_0_abs >= Yaw_th1:
                    msg = Twist()
                    msg.angular.z = -0.7 * ar_x_0
                    self.pub.publish(msg)
                    rospy.loginfo("调整角度对准移动目标(动态ID,第二个位置): angular.z=%f", msg.angular.z)
                elif ar_x_0_abs < Yaw_th1:
                    rospy.loginfo("移动目标(动态ID,第二个位置)对准成功，开始射击")
                    try:
                        if ser is None or not ser.is_open:
                            rospy.logerr("串口未打开，无法射击")
                            return
                        self.ar_cb_executed_case4 = True
                        ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
                        rospy.loginfo("移动目标(动态ID,第二个位置)射击指令已发送")
                        rospy.sleep(0.32)
                        ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
                        rospy.sleep(2)
                        rospy.loginfo("导航到goals[5](中转点)，然后到goals[6]")
                        self.goto(goals[5])  # 中转点
                        self.goto(goals[6])  # 最终射击位置
                        rospy.sleep(1)
                        case = 5
                        rospy.loginfo("切换到case=5, 准备射击target_id_moving_2")
                    except Exception as e:
                        rospy.logerr("移动目标(动态ID,第二个位置)射击失败: %s", str(e))
                        self.ar_cb_executed_case4 = False

            if marker.id == target_id_moving_2 and case == 5:
                if target_id_moving_2 == 255:
                    rospy.logwarn("target_id_moving_2未设置，跳过")
                    return
                if self.ar_cb_executed_case5:
                    return
                ar_x_0 = marker.pose.pose.position.x
                ar_x_0_abs = abs(ar_x_0)
                rospy.loginfo("移动目标(target_id_moving_2在goals[6]) - X偏差: %f (阈值: %f)", ar_x_0_abs, Yaw_th1)
                if ar_x_0_abs >= Yaw_th1:
                    msg = Twist()
                    msg.angular.z = -0.7 * ar_x_0
                    self.pub.publish(msg)
                    rospy.loginfo("调整角度对准移动目标(target_id_moving_2): angular.z=%f", msg.angular.z)
                elif ar_x_0_abs < Yaw_th1:
                    rospy.loginfo("移动目标(target_id_moving_2)对准成功，开始射击")
                    try:
                        if ser is None or not ser.is_open:
                            rospy.logerr("串口未打开，无法射击")
                            return
                        self.ar_cb_executed_case5 = True
                        ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
                        rospy.loginfo("移动目标(target_id_moving_2)射击指令已发送")
                        rospy.sleep(0.32)
                        ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
                        rospy.sleep(2)
                        rospy.loginfo("所有射击任务完成，导航到终点")
                        case = 6
                        self.end()
                    except Exception as e:
                        rospy.logerr("移动目标(target_id_moving_2)射击失败: %s", str(e))
                        self.ar_cb_executed_case5 = False

    def target_id_moving_callback(self, msg):
        global target_id_moving
        target_id_moving = msg.data
        rospy.loginfo("收到 target_id_moving: %d", target_id_moving)

    def target_id_moving_2_callback(self, msg):
        global target_id_moving_2
        target_id_moving_2 = msg.data
        rospy.loginfo("收到 target_id_moving_2: %d", target_id_moving_2)

    def set_pose(self, p):
        if self.move_base is None:
            return False
        x, y, th = p
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        q = transformations.quaternion_from_euler(0.0, 0.0, th / 180.0 * math.pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]

        self.set_pose_pub.publish(pose)
        return True

    def _done_cb(self, status, result):
        rospy.loginfo("navigation done! status:%d result:%s" % (status, result))

    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")

    def _feedback_cb(self, feedback):
        rospy.loginfo("[Navi] navigation feedback\r\n%s" % feedback)

    def goto(self, p):
        rospy.loginfo("[Navi] goto %s" % p)
        
        # 设置导航标志，禁用瞄准功能、超时检查和目标跟踪检测
        self.is_navigating = True
        self.timeout_check_enabled = False
        self.target_tracking_enabled = False
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2] / 180.0 * math.pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(60))
        
        # 导航结束，恢复瞄准功能并启动超时检查
        self.is_navigating = False
        self.case_start_time = time.time()
        self.timeout_check_enabled = True
        self.last_target_time = time.time()  # 重置目标接收时间
        self.target_tracking_enabled = True  # 启用目标跟踪检测
        rospy.loginfo("导航完成，启动超时检查（阈值: %.1f秒）和目标丢失检测（阈值: %.1f秒）", 
                      self.timeout_threshold, self.target_lost_threshold)
        
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!" % p)
                return True

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True

    def test_shoot(self):
        global ser
        rospy.loginfo("开始射击测试...")
        try:
            if ser is None:
                rospy.logerr("串口未初始化!")
                return False
            if not ser.is_open:
                rospy.logerr("串口未打开!")
                return False
            rospy.loginfo("串口状态正常")
            return True
        except Exception as e:
            rospy.logerr("射击测试失败: %s", str(e))
            return False

    def check_system_status(self):
        rospy.loginfo("=== 射击系统状态检查 ===")
        rospy.loginfo("当前case状态: %d", case)
        rospy.loginfo("目标移动ID1: %d (255=未设置)", target_id_moving)
        rospy.loginfo("目标移动ID2: %d (255=未设置)", target_id_moving_2)
        rospy.loginfo("串口端口: %s", serialPort)
        rospy.loginfo("find_cb_executed标志: %s", self.find_cb_executed)
        
        # 检查目标设置状态
        if target_id_moving == 255:
            rospy.loginfo("等待设置移动靶1目标 AR ID")
        else:
            rospy.loginfo("移动靶1目标已设置: AR ID %d", target_id_moving)
            
        if target_id_moving_2 == 255:
            rospy.loginfo("等待设置移动靶2目标 AR ID")
        else:
            rospy.loginfo("移动靶2目标已设置: AR ID %d", target_id_moving_2)
        
        try:
            if ser is None:
                rospy.logerr("串口未初始化!")
            elif ser.is_open:
                rospy.loginfo("串口连接正常")
            else:
                rospy.logerr("串口未连接!")
        except Exception as e:
            rospy.logerr("串口检查失败: %s", str(e))
        
        rospy.loginfo("=== 系统检查完成 ===")
        print("请在手动输入终端中设置目标序号...")


if __name__ == "__main__":
    rospy.init_node('navigation_demo', anonymous=True)

    # 解析目标点数据
    goals = [
        [0.4, -0.4, -90],# [1.13, -0.42, 0],
        [0.8, -0.2, 0],
        [1.25, -0.85, -90],
        [1.8, -1.35, 0],
        [2.15, -0.5, 90],# [1.05, -1.66, 0],
        [3.15, -0.4, -90],
        [3.15, -0.8, -90],
        [3.15, -0.45, -90],
]

    

# goals = [
# [1.13,-0.44,0], # 0
# [0.07,-0.44,0], # 1
# [0.07,-1.02,0], # 2
# [0.07,-1.67,0], # 3
# [1.03,-1.67,0], # 4
# [0.05,-1.67,0], # 5 
# [0.05,-2.24,0], # 6
# [0.05,-2.88,0], # 7
# [1.02,-2.88,0], # 8
# [0.21,-3.02,0]  # 9
#]

    #x_list = [float(x.strip()) for x in goalListX.split(",")]
    #y_list = [float(y.strip()) for y in goalListY.split(",")]
    #yaw_list = [float(yaw.strip()) for yaw in goalListYaw.split(",")]
    #for x, y, yaw in zip(x_list, y_list, yaw_list):
        #goals.append([x, y, yaw])

    #rospy.loginfo("解析后的目标点: %s", goals)
    #rospy.sleep(1)  # 等待节点初始化完成

    # 手动输入系统 - 不再需要音频发布函数

    # 初始化导航实例
    navi = navigation_demo()
    
    # 启动超时监控定时器（每0.5秒检查一次）
    timeout_monitor_timer = rospy.Timer(rospy.Duration(0.5), navi.timeout_monitor)
    rospy.loginfo("超时监控定时器已启动，检查周期: 0.5秒")
    
    # 启动目标丢失监控定时器（每0.1秒检查一次）
    target_monitor_timer = rospy.Timer(rospy.Duration(0.1), navi.target_lost_monitor)
    rospy.loginfo("目标丢失监控定时器已启动，检查周期: 0.1秒")
    
    # 系统状态检查
    navi.check_system_status()
    # 射击功能测试
    rospy.loginfo("执行射击测试...")
    shoot_test_result = navi.test_shoot()
    if not shoot_test_result:
        rospy.logerr("射击测试失败，请检查硬件连接!")

    # 等待启动信号
    while not rospy.is_shutdown():
        start_flag = rospy.get_param('/start', False)
        if start_flag:
            rospy.loginfo("开始比赛")
            break
        rospy.sleep(0.5)

    # 手动输入系统 - 等待目标设置完成
    rospy.loginfo("等待手动输入目标设置...")
    while not rospy.is_shutdown():
        if target_id_moving != 255 and target_id_moving_2 != 255:
            rospy.loginfo("目标设置完成 - 移动靶1: AR ID %d, 移动靶2: AR ID %d", 
                          target_id_moving, target_id_moving_2)
            break
        rospy.sleep(0.5)
    
    rospy.loginfo("开始执行射击任务...")
    
    # 导航到第一个目标点
    navi.goto(goals[0])
    rospy.sleep(1)
    case = 0
    rospy.loginfo("设置初始状态case=0，开始第一阶段")
    
    # 启动超时检查机制和目标跟踪检测
    navi.case_start_time = time.time()
    navi.timeout_check_enabled = True
    navi.last_target_time = time.time()
    navi.target_tracking_enabled = True
    rospy.loginfo("超时检查已启动，超时阈值: %.1f秒", navi.timeout_threshold)
    rospy.loginfo("目标跟踪检测已启动，目标丢失阈值: %.1f秒", navi.target_lost_threshold)

    # 保持节点运行
    rospy.spin()

