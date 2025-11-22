#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import math
import actionlib
import serial
import time
import sys
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

# 线路与Case配置（保留坐标方便后续微调）
USE_MIDDLE_POINTS_DEFAULT = False  # 默认不走中间点，可在启动时交互选择
ROUTE_LIBRARY = {
    'left': {
        'name': u'左侧路线',
        'case_labels': [0, 1, 2, 3],
        'case_meta': {
            0: {'mode': 'ar_fixed', 'ar_ids': [1], 'log_name': u'左侧固定靶1'},
            1: {'mode': 'ar_fixed', 'ar_ids': [2], 'log_name': u'左侧固定靶2'},
            2: {'mode': 'ar_dynamic_1', 'allowed_hint': [3, 4, 5], 'log_name': u'左侧移动靶1'},
            3: {'mode': 'ar_dynamic_2', 'allowed_hint': [6, 7, 8], 'log_name': u'左侧移动靶2'},
        },
        'waypoints': {
            'without_mid': [
                {'goal': [1.0, -0.8, 90], 'case': 0, 'label': u'左固定靶1'},
                {'goal': [1.5, -0.5, 0], 'case': 1, 'label': u'左固定靶2'},
                {'goal': [2.95, -1.0, 90], 'case': 2, 'label': u'左移动靶1'},
                {'goal': [2.7, -1.6, 180], 'case': 3, 'label': u'左移动靶2'},
            ],
            'with_mid': [
                {'goal': [0.8, -1.2, 0], 'case': None, 'label': u'左侧中间点A'},
                {'goal': [1.0, -0.8, 90], 'case': 0, 'label': u'左固定靶1'},
                {'goal': [1.5, -0.5, 0], 'case': 1, 'label': u'左固定靶2'},
                {'goal': [2.2, -1.2, 0], 'case': None, 'label': u'左侧中间点B'},
                {'goal': [2.95, -1.0, 90], 'case': 2, 'label': u'左移动靶1'},
                {'goal': [2.7, -1.6, 180], 'case': 3, 'label': u'左移动靶2'},
            ],
        },
    },
    'right': {
        'name': u'右侧路线',
        'case_labels': [4, 5, 6, 7],
        'case_meta': {
            4: {'mode': 'ar_fixed', 'ar_ids': [1], 'log_name': u'右侧固定靶1'},
            5: {'mode': 'ar_fixed', 'ar_ids': [2], 'log_name': u'右侧固定靶2'},
            6: {'mode': 'ar_dynamic_1', 'allowed_hint': [3, 4, 5], 'log_name': u'右侧移动靶1'},
            7: {'mode': 'ar_dynamic_2', 'allowed_hint': [6, 7, 8], 'log_name': u'右侧移动靶2'},
        },
        'waypoints': {
            'without_mid': [
                {'goal': [1.0, -2.4, 90], 'case': 4, 'label': u'右固定靶1'},
                {'goal': [1.5, -2.8, 0], 'case': 5, 'label': u'右固定靶2'},
                {'goal': [2.95, -2.4, 90], 'case': 6, 'label': u'右移动靶1'},
                {'goal': [2.7, -1.6, 180], 'case': 7, 'label': u'右移动靶2'},
            ],
            'with_mid': [
                {'goal': [0.8, -2.0, 0], 'case': None, 'label': u'右侧中间点A'},
                {'goal': [1.0, -2.4, 90], 'case': 4, 'label': u'右固定靶1'},
                {'goal': [1.5, -2.8, 0], 'case': 5, 'label': u'右固定靶2'},
                {'goal': [2.2, -2.0, 0], 'case': None, 'label': u'右侧中间点B'},
                {'goal': [2.95, -2.4, 90], 'case': 6, 'label': u'右移动靶1'},
                {'goal': [2.7, -1.6, 180], 'case': 7, 'label': u'右移动靶2'},
            ],
        },
    },
}


def _safe_input(prompt):
    """兼容 Python2/3 的输入函数"""
    try:
        return raw_input(prompt)
    except NameError:
        return input(prompt)


def prompt_start_side():
    """手动选择起点路线（左=1 / 右=2）"""
    prompt = u"\n请选择起点（左侧=1 / 右侧=2）："
    while True:
        choice = _safe_input(prompt).strip()
        if choice in ("1", "2"):
            return 'left' if choice == "1" else 'right'
        print(u"输入无效，请重新输入 1 或 2。")


def prompt_use_middle_points(default_flag=False):
    """询问是否启用中间点，保留后续灵活性"""
    default_text = u"y" if default_flag else u"n"
    prompt = u"是否启用中间点辅助导航？(y/n, 默认%s)：" % default_text
    while True:
        choice = _safe_input(prompt).strip().lower()
        if not choice:
            return default_flag
        if choice in ("y", "yes"):
            return True
        if choice in ("n", "no"):
            return False
        print(u"输入无效，请输入 y 或 n。")


def build_route_plan(route_key, use_middle_points):
    """根据路线和中间点配置构建路径与case映射"""
    route_cfg = ROUTE_LIBRARY[route_key]
    waypoint_key = 'with_mid' if use_middle_points else 'without_mid'
    selected_points = route_cfg['waypoints'][waypoint_key]

    goal_sequence = []
    case_meta = {}
    case_order = []

    for idx, point in enumerate(selected_points):
        entry = {
            'goal': point['goal'],
            'label': point.get('label', u'路径点%d' % idx),
            'case_label': point.get('case')
        }
        goal_sequence.append(entry)
        if point.get('case') is not None:
            case_label = point['case']
            meta = dict(route_cfg['case_meta'][case_label])
            meta['goal_index'] = idx
            case_meta[case_label] = meta
            case_order.append(case_label)

    if not case_order:
        raise ValueError("所选路线没有任何射击case，检查配置是否正确。")

    final_goal_index = len(goal_sequence) - 1
    return {
        'route_key': route_key,
        'route_name': route_cfg['name'],
        'goal_sequence': goal_sequence,
        'case_meta': case_meta,
        'case_order': case_order,
        'final_goal_index': final_goal_index,
    }


def format_goal(goal_list):
    """格式化日志中的目标点信息"""
    return "[{:.2f}, {:.2f}, {:.1f}]".format(goal_list[0], goal_list[1], goal_list[2])


class navigation_demo:
    def __init__(self, mission_plan):
        global ser
        self.mission_plan = mission_plan
        self.route_name = mission_plan['route_name']
        self.goal_sequence = mission_plan['goal_sequence']
        self.case_meta = mission_plan['case_meta']
        self.case_order = mission_plan['case_order']
        self.final_goal_index = mission_plan['final_goal_index']

        self.is_navigating = False  # 导航状态标志，防止导航时瞄准冲突
        self.find_cb_executed = False
        self.case_execution_flags = {label: False for label in self.case_order}
        self.goal_cursor = -1  # goal_sequence 中已完成的索引
        self.current_case_idx = 0  # case_order 中当前case的位置

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
        self.timeout_check_enabled = False  # 是否启用超时检测
        
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

    def get_case_meta(self, case_label=None):
        """获取当前case的配置"""
        label = case_label if case_label is not None else case
        return self.case_meta.get(label)

    def update_case_index(self, case_label):
        """根据case标签刷新当前索引"""
        if case_label in self.case_order:
            self.current_case_idx = self.case_order.index(case_label)

    def get_next_case_label(self, current_case=None):
        """取得下一个case标签，没有则返回None"""
        label = current_case if current_case is not None else case
        if label not in self.case_order:
            return None
        idx = self.case_order.index(label)
        if idx + 1 < len(self.case_order):
            return self.case_order[idx + 1]
        return None

    def move_through_sequence(self, target_index):
        """按照goal_sequence顺序前往目标索引（含中间点）"""
        while self.goal_cursor < target_index:
            self.goal_cursor += 1
            goal_entry = self.goal_sequence[self.goal_cursor]
            rospy.loginfo("导航至路径点 #%d %s -> %s",
                          self.goal_cursor,
                          goal_entry['label'],
                          format_goal(goal_entry['goal']))
            is_final_target = self.goal_cursor == target_index
            self.goto(goal_entry['goal'])
            if not is_final_target:
                # 中间点不计入超时/跟踪，仅作为过渡
                self.timeout_check_enabled = False
                self.target_tracking_enabled = False

    def navigate_to_case(self, case_label):
        """导航到指定case对应的射击点"""
        meta = self.get_case_meta(case_label)
        if not meta:
            rospy.logerr("未找到case=%s的配置，检查路线设置。", str(case_label))
            return
        self.move_through_sequence(meta['goal_index'])
        self.configure_tracking_for_case(case_label)
        self.update_case_index(case_label)
        self.find_cb_executed = False
        rospy.loginfo("已就位 %s（case=%s），等待射击触发。", meta['log_name'], case_label)

    def configure_tracking_for_case(self, case_label):
        """根据case类型启用/关闭目标丢失监控"""
        meta = self.get_case_meta(case_label)
        if not meta:
            self.target_tracking_enabled = False
            return
        if meta.get('mode') == 'vision':
            self.target_tracking_enabled = True
            self.last_target_time = time.time()
        else:
            self.target_tracking_enabled = False

    def mark_case_completed(self, case_label):
        """记录case已执行完成"""
        self.case_execution_flags[case_label] = True
        rospy.loginfo("case=%s 已完成", str(case_label))

    def resolve_expected_ids(self, stage):
        """解析当前case所需识别的AR ID列表"""
        global target_id_moving, target_id_moving_2
        mode = stage.get('mode')
        if mode == 'ar_fixed':
            return stage.get('ar_ids', [])
        if mode == 'ar_dynamic_1':
            if target_id_moving == 255:
                rospy.logwarn("等待手动输入第一个移动靶的AR ID（topic target_id_moving）")
                return []
            return [target_id_moving]
        if mode == 'ar_dynamic_2':
            if target_id_moving_2 == 255:
                rospy.logwarn("等待手动输入第二个移动靶的AR ID（topic target_id_moving_2）")
                return []
            return [target_id_moving_2]
        return []

    def proceed_to_next_case(self):
        """前往下一个case，若没有则执行结束流程"""
        global case
        next_case = self.get_next_case_label(case)
        if next_case is None:
            rospy.loginfo("全部case已处理，开始执行end()")
            case = 255
            self.end()
            return
        case = next_case
        self.navigate_to_case(case)

    def execute_shoot_command(self, stage_name):
        """向发射控制板发送一轮射击命令"""
        try:
            if ser is None or not ser.is_open:
                rospy.logerr("[%s] 串口未打开，无法射击", stage_name)
                return False
            ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
            rospy.loginfo("[%s] 开火指令已发送", stage_name)
            rospy.sleep(0.32)
            ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
            rospy.loginfo("[%s] 停火指令已发送", stage_name)
            return True
        except Exception as e:
            rospy.logerr("[%s] 射击指令失败: %s", stage_name, str(e))
            return False

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
        """固定靶场景下的目标丢失检测"""
        stage = self.get_case_meta()
        if not stage or stage.get('mode') != 'vision':
            return

        if self.check_target_lost():
            rospy.logwarn("%.2f 秒未检测到目标，停止旋转重新等待", time.time() - self.last_target_time)
            self.stop_robot()
            self.last_target_time = time.time()

    def handle_timeout_skip(self):
        """超时后跳转到下一个case"""
        global case
        rospy.logwarn("当前case=%s 超时，准备切换下一阶段", str(case))

        # 立即停止底盘运动，避免继续转动
        self.stop_robot()

        # 标记当前case已结束，防止重复触发
        if case in self.case_execution_flags:
            self.case_execution_flags[case] = True

        next_case = self.get_next_case_label(case)
        if next_case is None:
            rospy.logwarn("没有更多case，执行收尾流程")
            case = 255
            self.end()
            return

        rospy.loginfo("跳转到下一个case=%s", str(next_case))
        case = next_case
        self.navigate_to_case(case)
        self.case_start_time = time.time()

    def end(self):
        """执行收尾动作并保持与旧脚本一致的撤离动作"""
        if self.final_goal_index is not None:
            self.move_through_sequence(self.final_goal_index)

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
        # 可在此处补充语音或其他收尾动作

    def find_cb(self, data):
        """视觉识别固定靶的回调（预留，当前路线默认使用AR）"""
        if self.is_navigating:
            return

        global id, flog0, flog1, flog2, count, move_flog, case
        stage = self.get_case_meta()
        if not stage or stage.get('mode') != 'vision':
            return
        if self.case_execution_flags.get(case):
            return

        self.last_target_time = time.time()
        id = 255
        point_msg = data
        flog0 = point_msg.x - 315
        flog1 = abs(flog0)

        rospy.loginfo("接收到视觉靶位: x=%f, 偏差=%f, case=%s", point_msg.x, flog0, str(case))

        if abs(flog1) > 5:
            if self.find_cb_executed:
                return
            msg = Twist()
            msg.angular.z = -0.013 * flog0
            self.pub.publish(msg)
            rospy.loginfo("视觉对准中: angular.z=%f, 偏差=%f", msg.angular.z, flog1)
        else:
            if self.find_cb_executed:
                return
            stage_name = stage.get('log_name', '视觉靶')
            rospy.loginfo("视觉靶对准成功，准备射击: %s", stage_name)
            if self.execute_shoot_command(stage_name):
                self.find_cb_executed = True
                self.mark_case_completed(case)
                rospy.sleep(1)
                self.proceed_to_next_case()
            else:
                self.find_cb_executed = False

    def ar_cb(self, data):
        if self.is_navigating:
            return

        global case
        stage = self.get_case_meta()
        if not stage:
            return
        mode = stage.get('mode')
        if mode not in ('ar_fixed', 'ar_dynamic_1', 'ar_dynamic_2'):
            return
        if self.case_execution_flags.get(case):
            return

        markers = data.markers
        rospy.loginfo("检测到 %d 个AR靶标，当前case=%s", len(markers), str(case))
        expected_ids = self.resolve_expected_ids(stage)
        if not expected_ids:
            return

        for marker in markers:
            rospy.loginfo("AR靶ID: %d, 期望ID: %s", marker.id, expected_ids)
            if marker.id not in expected_ids:
                continue

            ar_x_0 = marker.pose.pose.position.x
            ar_x_abs = abs(ar_x_0)
            if ar_x_abs >= Yaw_th1:
                msg = Twist()
                msg.angular.z = -0.7 * ar_x_0
                self.pub.publish(msg)
                rospy.loginfo("AR对准中(%s): angular.z=%f", stage['log_name'], msg.angular.z)
            else:
                rospy.loginfo("AR对准完成，即将射击: %s", stage['log_name'])
                if self.execute_shoot_command(stage['log_name']):
                    self.mark_case_completed(case)
                    rospy.sleep(2)
                    self.proceed_to_next_case()
                else:
                    rospy.logwarn("射击失败，等待下一次回调")
            return

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
        rospy.loginfo("=== 任务状态自检 ===")
        rospy.loginfo("当前路线: %s", self.route_name)
        rospy.loginfo("Case 顺序: %s", self.case_order)
        rospy.loginfo("当前case标签: %s", str(case))
        rospy.loginfo("目标移动ID1: %d (255=未配置)", target_id_moving)
        rospy.loginfo("目标移动ID2: %d (255=未配置)", target_id_moving_2)
        rospy.loginfo("串口设备: %s", serialPort)
        rospy.loginfo("视觉回调是否已射击: %s", self.find_cb_executed)

        if target_id_moving == 255:
            rospy.loginfo("等待设置第一个移动靶 AR ID")
        else:
            rospy.loginfo("第一个移动靶将识别 AR ID %d", target_id_moving)

        if target_id_moving_2 == 255:
            rospy.loginfo("等待设置第二个移动靶 AR ID")
        else:
            rospy.loginfo("第二个移动靶将识别 AR ID %d", target_id_moving_2)

        try:
            if ser is None:
                rospy.logerr("串口尚未初始化!")
            elif ser.is_open:
                rospy.loginfo("串口已打开，可执行射击命令")
            else:
                rospy.logerr("串口已初始化但未打开!")
        except Exception as e:
            rospy.logerr("串口状态检查失败: %s", str(e))

        rospy.loginfo("=== 自检结束 ===")
        print("如需重新输入移动靶ID，请运行 2025_shoot_demo(1).py 手动发布话题。")




if __name__ == "__main__":
    # 1. 读取用户输入，确定从左/右起点出发以及是否启用中间点
    route_choice = prompt_start_side()
    use_middle_points = prompt_use_middle_points(USE_MIDDLE_POINTS_DEFAULT)
    mission_plan = build_route_plan(route_choice, use_middle_points)
    goals = [entry['goal'] for entry in mission_plan['goal_sequence']]  # 仅用于调试/查看

    rospy.init_node('navigation_demo', anonymous=True)
    rospy.loginfo("路线选择: %s，是否使用中间点: %s", mission_plan['route_name'], 'YES' if use_middle_points else 'NO')
    rospy.loginfo("Case 顺序: %s", mission_plan['case_order'])

    # 2. 初始化导航控制类并启动监控定时器
    navi = navigation_demo(mission_plan)
    timeout_monitor_timer = rospy.Timer(rospy.Duration(0.5), navi.timeout_monitor)
    rospy.loginfo("超时检测定时器已启动，周期 0.5s")
    target_monitor_timer = rospy.Timer(rospy.Duration(0.1), navi.target_lost_monitor)
    rospy.loginfo("目标丢失检测定时器已启动，周期 0.1s")

    # 3. 上电自检
    navi.check_system_status()
    rospy.loginfo("执行射击回路自检...")
    if not navi.test_shoot():
        rospy.logerr("射击自检失败，请检查硬件!")

    # 4. 等待 /start 参数被手动输入脚本置为 True
    rospy.loginfo("等待 /start 参数触发...")
    while not rospy.is_shutdown():
        start_flag = rospy.get_param('/start', False)
        if start_flag:
            rospy.loginfo("检测到 /start=True，准备继续")
            break
        rospy.sleep(0.5)

    # 5. 等待手动输入的两个移动靶 AR ID
    rospy.loginfo("等待手动输入的两个移动靶 AR ID...")
    while not rospy.is_shutdown():
        if target_id_moving != 255 and target_id_moving_2 != 255:
            rospy.loginfo("已收到移动靶ID: target_id_moving=%d, target_id_moving_2=%d",
                          target_id_moving, target_id_moving_2)
            break
        rospy.sleep(0.5)

    rospy.loginfo("所有前置条件满足，开始执行新地图流程...")

    # 6. 导航至首个射击点，并设置case为对应的标签
    case = mission_plan['case_order'][0]
    navi.navigate_to_case(case)
    rospy.sleep(1)
    rospy.loginfo("已进入首个case=%s，开始第一阶段", str(case))

    # 7. 启用超时与目标监控
    navi.case_start_time = time.time()
    navi.timeout_check_enabled = True
    navi.last_target_time = time.time()
    navi.configure_tracking_for_case(case)
    rospy.loginfo("当前超时阈值: %.1f s", navi.timeout_threshold)
    rospy.loginfo("目标丢失阈值: %.1f s", navi.target_lost_threshold)

    rospy.spin()
