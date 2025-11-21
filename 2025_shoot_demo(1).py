#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int32

def manual_input_handler():
    """
    手动输入处理函数
    输入两个移动靶的 AR 标记 ID
    """
    global pub_moving_1, pub_moving_2
    
    rospy.loginfo("手动输入模式启动")
    print("=" * 60)
    print("手动射击目标输入系统 (2025版)")
    print("=" * 60)
    print("目标配置说明:")
    print("  - Case 0: 固定目标 (视觉识别)")
    print("  - Case 1: 移动靶 ID=1 (第一次)")
    print("  - Case 2: 移动靶 ID=1 (第二次)")
    print("  - Case 3: 移动靶1 (需要输入 AR 标记 ID)")
    print("  - Case 4: 移动靶2 (需要输入 AR 标记 ID)")
    print("  - Case 5: 固定目标 (视觉识别)")
    print("=" * 60)
    
    moving_target_1 = None
    moving_target_2 = None
    
    while not rospy.is_shutdown():
        try:
            print("\n" + "=" * 40)
            print("目标设置状态:")
            print(f"移动靶1 (case 3): {moving_target_1 if moving_target_1 else '未设置'}")
            print(f"移动靶2 (case 4): {moving_target_2 if moving_target_2 else '未设置'}")
            print("=" * 40)
            
            # 输入第一个移动靶 ID
            if moving_target_1 is None:
                print("\n请输入第一个移动靶的 AR 标记 ID (建议 6-8):")
                user_input = input("移动靶1 AR ID: ").strip()
                
                if user_input.lower() == 'q':
                    rospy.loginfo("用户退出程序")
                    break
                
                try:
                    target_id = int(user_input)
                    if 1 <= target_id <= 10:
                        moving_target_1 = target_id
                        
                        # 发布到 target_id_moving 话题
                        while pub_moving_1.get_num_connections() == 0 and not rospy.is_shutdown():
                            rospy.sleep(0.1)
                        
                        pub_moving_1.publish(target_id)
                        print(f"✅ 移动靶1 (case 3) 设置为 AR ID: {target_id}")
                        rospy.loginfo(f"发布移动靶1 ID: {target_id}")
                    else:
                        print("❌ 无效输入！请输入 1-10 之间的数字")
                        continue
                except ValueError:
                    print("❌ 请输入有效的数字")
                    continue
            
            # 输入第二个移动靶 ID
            elif moving_target_2 is None:
                print("\n请输入第二个移动靶的 AR 标记 ID (建议 6-8):")
                user_input = input("移动靶2 AR ID: ").strip()
                
                if user_input.lower() == 'q':
                    rospy.loginfo("用户退出程序")
                    break
                
                try:
                    target_id = int(user_input)
                    if 1 <= target_id <= 10:
                        moving_target_2 = target_id
                        
                        # 发布到 target_id_moving_2 话题
                        while pub_moving_2.get_num_connections() == 0 and not rospy.is_shutdown():
                            rospy.sleep(0.1)
                        
                        pub_moving_2.publish(target_id)
                        print(f"✅ 移动靶2 (case 4) 设置为 AR ID: {target_id}")
                        rospy.loginfo(f"发布移动靶2 ID: {target_id}")
                        
                        # 显示完整设置并确认
                        print("\n" + "=" * 50)
                        print("🎯 目标设置完成！")
                        print(f"🏃 移动靶1 (case 3): AR ID {moving_target_1}")
                        print(f"🏃 移动靶2 (case 4): AR ID {moving_target_2}")
                        print("=" * 50)
                        
                        confirm = input("\n按回车键开始执行任务，或输入 'r' 重新设置: ").strip()
                        if confirm.lower() == 'r':
                            moving_target_1 = None
                            moving_target_2 = None
                            print("🔄 重新设置目标...")
                            continue
                        else:
                            print("🚀 开始执行射击任务...")
                            rospy.loginfo("用户确认开始执行任务")
                            # 触发主导航节点启动
                            rospy.set_param('/start', True)
                            # 保持程序运行，等待系统执行
                            while not rospy.is_shutdown():
                                rospy.sleep(1)
                            break
                    else:
                        print("❌ 无效输入！请输入 1-10 之间的数字")
                        continue
                except ValueError:
                    print("❌ 请输入有效的数字")
                    continue
                
        except KeyboardInterrupt:
            rospy.loginfo("程序被用户中断")
            break
        except Exception as e:
            rospy.logerr(f"输入处理错误: {e}")

def manual_input_node():
    """
    初始化手动输入节点
    """
    rospy.init_node('manual_input_node', anonymous=True)
    global pub_moving_1, pub_moving_2
    
    # 发布移动靶 ID 到对应话题
    pub_moving_1 = rospy.Publisher('target_id_moving', Int32, queue_size=10)
    pub_moving_2 = rospy.Publisher('target_id_moving_2', Int32, queue_size=10)
    
    rospy.loginfo("手动输入节点已启动")
    
    # 启动手动输入处理
    manual_input_handler()

if __name__ == '__main__':
    try:
        manual_input_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS节点被中断")
