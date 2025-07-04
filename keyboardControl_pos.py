#!/usr/bin/env python
# coding=utf-8

import rospy
import math
import sys, select, termios, tty
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import State

# 按键说明
msg = """
Control Your PX4 Drone (Position Control Mode)!
-----------------------------------------------
Moving around:
   i    : forward
   ,    : backward
   j    : left
   l    : right
   r    : ascend
   f    : descend

q/z : increase/decrease movement speed by 10%
k   : force stop
space: land
5   : switch to OFFBOARD mode
6   : arm the drone
7   : take off
CTRL-C to quit
"""

# 获取键值函数
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# 初始化参数
speed = 0.2      # 移动速度 m/s
target_x = 0.0   # 目标x位置
target_y = 0.0   # 目标y位置
target_z = 1.0   # 目标z位置（默认高度1米）
current_state = State()

# 回调函数：订阅mavros状态
def state_cb(state):
    global current_state
    current_state = state

# 回调函数：订阅无人机位姿（用于获取当前朝向，若需要姿态保持）
def pose_cb(m):
    global current_yaw
    # 从四元数计算偏航角（简化处理，实际应用可使用tf转换）
    q = m.pose.orientation
    current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

# 主函数
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)  # 获取终端属性
    rospy.init_node('px4_position_teleop')  # 初始化ROS节点
    
    # 创建发布者（位置控制话题）
    pose_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    
    # 创建订阅者
    rospy.Subscriber('mavros/state', State, state_cb)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, pose_cb)
    
    # 创建服务客户端
    set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
    arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    
    # 初始化位置消息
    pose = PoseStamped()
    pose.header.frame_id = "base_link"  # 坐标系，可根据实际情况修改
    
    try:
        print(msg)  # 打印控制说明
        print("当前速度: %.2f m/s" % speed)
        
        # 等待连接
        while not rospy.is_shutdown() and not current_state.connected:
            rospy.sleep(0.1)
        
        print("已连接到MAVROS，准备控制...")
        
        while(1):
            key = getKey()
            
            # 键盘控制逻辑
            if key == 'i':   # 前进
                target_x += speed
            elif key == ',': # 后退
                target_x -= speed
            elif key == 'j': # 左移
                target_y -= speed
            elif key == 'l': # 右移
                target_y += speed
            elif key == 'r': # 上升
                target_z = min(target_z + speed, 10.0)  # 限制最大高度10米
            elif key == 'f': # 下降
                target_z = max(target_z - speed, 0.5)   # 限制最小高度0.5米
            elif key == 'q': # 加速
                speed = min(speed * 1.1, 1.0)           # 限制最大速度1.0m/s
                print("当前速度: %.2f m/s" % speed)
            elif key == 'z': # 减速
                speed = max(speed * 0.9, 0.1)           # 限制最小速度0.1m/s
                print("当前速度: %.2f m/s" % speed)
            elif key == 'k': # 强制停止（位置不变）
                pass
            elif key == ' ': # 降落
                print("执行降落...")
                set_mode_srv(custom_mode='AUTO.LAND')
            elif key == '5': # 切换到OFFBOARD模式
                if current_state.mode != "OFFBOARD":
                    set_mode_srv(custom_mode='OFFBOARD')
                    print("OFFBOARD模式已启用")
            elif key == '6': # 解锁无人机
                arming_srv(True)
                print("无人机已解锁")
            elif key == '7': # 起飞
                print("执行起飞...")
                set_mode_srv(custom_mode='AUTO.TAKEOFF')
            elif key == '\x03': # Ctrl+C退出
                break
            else:
                pass
            
            # 设置目标位置
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = target_x
            pose.pose.position.y = target_y
            pose.pose.position.z = target_z
            
            # 保持当前朝向（四元数，若需要姿态稳定可取消注释）
            # pose.pose.orientation.x = q.x
            # pose.pose.orientation.y = q.y
            # pose.pose.orientation.z = q.z
            # pose.pose.orientation.w = q.w
            
            # 发布位置指令（OFFBOARD模式需要至少2Hz的频率）
            pose_pub.publish(pose)
            rospy.sleep(0.1)  # 控制发布频率
            
    except Exception as e:
        print("错误发生: ", e)
    
    finally:
        # 程序结束前发布停止指令
        stop_pose = PoseStamped()
        stop_pose.header.stamp = rospy.Time.now()
        stop_pose.pose.position.x = 0
        stop_pose.pose.position.y = 0
        stop_pose.pose.position.z = 0
        pose_pub.publish(stop_pose)
        
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)