#!/usr/bin/env python
# coding=utf-8

import rospy
import math
import sys, select, termios, tty
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.srv import SetMode, CommandBool, ParamSet
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import ParamSetRequest  # 添加缺失的导入

# 按键说明
msg = """
Control Your PX4 Drone (Attitude Control Mode)!
-----------------------------------------------
Pitch/Roll control:
   i    : pitch forward
   ,    : pitch backward
   j    : roll left
   l    : roll right

Thrust control:
   r    : increase thrust
   f    : decrease thrust

Yaw control:
   a    : yaw left
   d    : yaw right

q/z : increase/decrease attitude rate by 5%
w/x : increase/decrease thrust rate by 5%
k   : force stop
space: land
5   : switch to OFFBOARD mode (manual attitude control)
6   : arm the drone
7   : take off (AUTO.TAKEOFF)
m   : toggle display mode
p   : print current parameters
1   : decrease roll/pitch P gain
2   : increase roll/pitch P gain
3   : decrease yaw P gain
4   : increase yaw P gain
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

# 四元数转欧拉角
def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z

# 欧拉角转四元数
def euler_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [x, y, z, w]

# 打印状态信息
def print_status():
    if display_mode == 0:
        print("\r当前模式: {} | 姿态: R:{:.1f} P:{:.1f} Y:{:.1f} | 推力: {:.2f} | P增益: R/P:{:.2f} Y:{:.2f}".format(
            current_state.mode,
            math.degrees(current_roll),
            math.degrees(current_pitch),
            math.degrees(current_yaw),
            target_thrust,
            current_roll_pitch_p,
            current_yaw_p), end='')
    elif display_mode == 1:
        print("\r当前模式: {} | 位置: X:{:.2f} Y:{:.2f} Z:{:.2f} | 姿态: Y:{:.1f} | P增益: R/P:{:.2f} Y:{:.2f}".format(
            current_state.mode,
            current_position.x,
            current_position.y,
            current_position.z,
            math.degrees(current_yaw),
            current_roll_pitch_p,
            current_yaw_p), end='')

# 设置PX4参数（修正版，正确处理不同数据类型）
def set_px4_parameter(param_id, value, param_type='real'):
    try:
        set_param = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        req = ParamSetRequest()  # 使用正确的请求类
        req.param_id = param_id
        
        # 根据参数类型设置值
        if param_type == 'real':
            req.value.real = value
        elif param_type == 'integer':
            req.value.integer = int(value)
        elif param_type == 'boolean':
            req.value.boolean = bool(value)
        
        res = set_param(req)
        if res.success:
            rospy.loginfo(f"参数 {param_id} 设置为 {value} (类型: {param_type})")
            return True
        else:
            rospy.logerr(f"参数 {param_id} 设置失败: {res.result}")
            return False
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")
        return False

# 回调函数：订阅mavros状态
def state_cb(state):
    global current_state
    current_state = state

# 回调函数：订阅无人机位姿
def pose_cb(m):
    global current_roll, current_pitch, current_yaw, current_position
    q = m.pose.orientation
    current_roll, current_pitch, current_yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
    current_position = m.pose.position

# 初始化参数
attitude_rate = 0.05    # 姿态变化率 rad/s（降低灵敏度）
thrust_rate = 0.01      # 推力变化率（降低灵敏度）
target_roll = 0.0       # 目标横滚角
target_pitch = 0.0      # 目标俯仰角
target_yaw = 0.0        # 目标偏航角
target_thrust = 0.4     # 目标推力(0.0-1.0)（降低初始推力）
max_roll_pitch = 0.2    # 最大横滚/俯仰角 rad（降低最大角度限制）
max_yaw_rate = 0.3      # 最大偏航角速率 rad/s
min_thrust = 0.2        # 最小推力
max_thrust = 0.7        # 最大推力（降低最大推力上限）
current_state = State()
current_roll = 0.0
current_pitch = 0.0
current_yaw = 0.0
current_position = PoseStamped().pose.position
display_mode = 0        # 0: 显示姿态 1: 显示位置
auto_takeoff_complete = False  # 自动起飞完成标志
smoothing_factor = 0.2  # 平滑系数，0-1之间，值越小越平滑

# PID参数
current_roll_pitch_p = 0.6  # 初始滚转/俯仰P增益
current_yaw_p = 0.4         # 初始偏航P增益

# 主函数
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('px4_attitude_teleop')
    
    # 创建发布者（姿态控制话题）
    attitude_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
    
    # 创建订阅者
    rospy.Subscriber('mavros/state', State, state_cb)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, pose_cb)
    
    # 创建服务客户端
    set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
    arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    
    # 初始化姿态消息
    attitude = AttitudeTarget()
    # 设置type_mask，只忽略角速率，使用四元数姿态和推力控制
    attitude.type_mask = 7  # 1+2+4=7 (忽略body_rate_x, body_rate_y, body_rate_z)

    try:
        print(msg)
        print("当前姿态变化率: %.2f rad/s" % attitude_rate)
        print("当前推力变化率: %.2f" % thrust_rate)
        print("初始姿态P增益: 滚转/俯仰=%.2f, 偏航=%.2f" % (current_roll_pitch_p, current_yaw_p))
        
        # 设置初始PID参数（明确指定为浮点数类型）
        set_px4_parameter("MC_ROLL_P", current_roll_pitch_p, 'real')
        set_px4_parameter("MC_PITCH_P", current_roll_pitch_p, 'real')
        set_px4_parameter("MC_YAW_P", current_yaw_p, 'real')
        
        # 等待连接
        while not rospy.is_shutdown() and not current_state.connected:
            rospy.sleep(0.1)
        
        print("已连接到MAVROS，准备控制...")
        
        # 先发布一些消息，让PX4准备好接收OFFBOARD命令
        rate = rospy.Rate(10)  # 10 Hz
        for i in range(100):
            if rospy.is_shutdown():
                break
            attitude.header.stamp = rospy.Time.now()
            # 初始化为当前姿态
            q = euler_to_quaternion(current_roll, current_pitch, current_yaw)
            attitude.orientation.x = q[0]
            attitude.orientation.y = q[1]
            attitude.orientation.z = q[2]
            attitude.orientation.w = q[3]
            attitude.thrust = min_thrust
            attitude_pub.publish(attitude)
            rate.sleep()
        
        # 记录自动起飞前的状态
        takeoff_altitude = 0.0
        last_mode = ""
        
        while(1):
            key = getKey()
            
            # 键盘控制逻辑
            if key == 'i':   # 俯仰向前（机头向下）
                target_pitch = max(target_pitch - attitude_rate, -max_roll_pitch)
            elif key == ',': # 俯仰向后（机头向上）
                target_pitch = min(target_pitch + attitude_rate, max_roll_pitch)
            elif key == 'j': # 横滚向左
                target_roll = max(target_roll - attitude_rate, -max_roll_pitch)
            elif key == 'l': # 横滚向右
                target_roll = min(target_roll + attitude_rate, max_roll_pitch)
            elif key == 'r': # 增加推力
                target_thrust = min(target_thrust + thrust_rate, max_thrust)
            elif key == 'f': # 减小推力
                target_thrust = max(target_thrust - thrust_rate, min_thrust)
            elif key == 'a': # 偏航向左
                target_yaw = (target_yaw - attitude_rate) % (2 * math.pi)
            elif key == 'd': # 偏航向右
                target_yaw = (target_yaw + attitude_rate) % (2 * math.pi)
            elif key == 'q': # 增加姿态变化率
                attitude_rate = min(attitude_rate * 1.05, 0.5)
                print("\n当前姿态变化率: %.2f rad/s" % attitude_rate)
            elif key == 'z': # 减小姿态变化率
                attitude_rate = max(attitude_rate * 0.95, 0.02)
                print("\n当前姿态变化率: %.2f rad/s" % attitude_rate)
            elif key == 'w': # 增加推力变化率
                thrust_rate = min(thrust_rate * 1.05, 0.1)
                print("\n当前推力变化率: %.2f" % thrust_rate)
            elif key == 'x': # 减小推力变化率
                thrust_rate = max(thrust_rate * 0.95, 0.005)
                print("\n当前推力变化率: %.2f" % thrust_rate)
            elif key == 'k': # 强制停止
                target_roll = 0.0
                target_pitch = 0.0
                # 保持当前偏航角
                target_yaw = current_yaw
                target_thrust = min_thrust if current_state.armed else 0.0
            elif key == ' ': # 降落
                print("\n执行降落...")
                set_mode_srv(custom_mode='AUTO.LAND')
                auto_takeoff_complete = False
            elif key == '5': # 切换到OFFBOARD模式
                if current_state.mode != "OFFBOARD":
                    # 如果是从AUTO.TAKEOFF模式切换，保留当前姿态和高度
                    if current_state.mode == "AUTO.TAKEOFF":
                        target_roll = current_roll
                        target_pitch = current_pitch
                        target_yaw = current_yaw
                        # 设置适当的推力以维持当前高度
                        target_thrust = 0.55  # 根据实际情况调整
                        auto_takeoff_complete = True
                        print("\n从AUTO.TAKEOFF切换到OFFBOARD模式，保持当前姿态")
                    else:
                        print("\nOFFBOARD模式已启用")
                    set_mode_srv(custom_mode='OFFBOARD')
            elif key == '6': # 解锁无人机
                arming_srv(True)
                print("\n无人机已解锁")
            elif key == '7': # 起飞
                if current_state.armed and current_state.mode != "AUTO.TAKEOFF":
                    print("\n执行起飞...")
                    takeoff_altitude = current_position.z
                    set_mode_srv(custom_mode='AUTO.TAKEOFF')
                    auto_takeoff_complete = False
            elif key == 'm': # 切换显示模式
                display_mode = (display_mode + 1) % 2
                print("\n显示模式: {}".format("姿态" if display_mode == 0 else "位置"))
            elif key == 'p': # 打印当前参数
                print("\n当前参数:")
                print(f"姿态变化率: {attitude_rate:.3f} rad/s")
                print(f"推力变化率: {thrust_rate:.3f}")
                print(f"姿态限制: 最大滚转/俯仰 = {math.degrees(max_roll_pitch):.1f}°, 最大偏航速率 = {math.degrees(max_yaw_rate):.1f}°/s")
                print(f"推力限制: 最小 = {min_thrust:.2f}, 最大 = {max_thrust:.2f}")
                print(f"PID参数: 滚转/俯仰P = {current_roll_pitch_p:.2f}, 偏航P = {current_yaw_p:.2f}")
                print(f"平滑系数: {smoothing_factor:.2f}")
            elif key == '1': # 减小滚转/俯仰P增益
                current_roll_pitch_p = max(current_roll_pitch_p - 0.05, 0.1)
                set_px4_parameter("MC_ROLL_P", current_roll_pitch_p, 'real')
                set_px4_parameter("MC_PITCH_P", current_roll_pitch_p, 'real')
                print(f"\n滚转/俯仰P增益设置为: {current_roll_pitch_p:.2f}")
            elif key == '2': # 增加滚转/俯仰P增益
                current_roll_pitch_p = min(current_roll_pitch_p + 0.05, 2.0)
                set_px4_parameter("MC_ROLL_P", current_roll_pitch_p, 'real')
                set_px4_parameter("MC_PITCH_P", current_roll_pitch_p, 'real')
                print(f"\n滚转/俯仰P增益设置为: {current_roll_pitch_p:.2f}")
            elif key == '3': # 减小偏航P增益
                current_yaw_p = max(current_yaw_p - 0.05, 0.1)
                set_px4_parameter("MC_YAW_P", current_yaw_p, 'real')
                print(f"\n偏航P增益设置为: {current_yaw_p:.2f}")
            elif key == '4': # 增加偏航P增益
                current_yaw_p = min(current_yaw_p + 0.05, 2.0)
                set_px4_parameter("MC_YAW_P", current_yaw_p, 'real')
                print(f"\n偏航P增益设置为: {current_yaw_p:.2f}")
            elif key == '\x03': # Ctrl+C退出
                break
            else:
                pass
            
            # 自动检测起飞完成
            if current_state.mode == "AUTO.TAKEOFF" and not auto_takeoff_complete:
                if current_position.z >= takeoff_altitude + 0.8:  # 假设起飞高度增加0.8米以上为完成
                    print("\n自动起飞完成，可以切换到OFFBOARD模式继续控制")
                    auto_takeoff_complete = True
            
            # 显示当前状态
            print_status()
            
            # 设置目标姿态
            attitude.header.stamp = rospy.Time.now()
            
            # 计算目标四元数
            q = euler_to_quaternion(target_roll, target_pitch, target_yaw)
            attitude.orientation.x = q[0]
            attitude.orientation.y = q[1]
            attitude.orientation.z = q[2]
            attitude.orientation.w = q[3]
            
            # 设置推力
            attitude.thrust = target_thrust
            
            # 发布姿态指令
            attitude_pub.publish(attitude)
            rospy.sleep(0.05)  # 20Hz发布频率
            
    except Exception as e:
        print("\n错误发生: ", e)
    
    finally:
        # 程序结束前发布停止指令
        print("\n程序退出，发送停止指令...")
        stop_attitude = AttitudeTarget()
        stop_attitude.header.stamp = rospy.Time.now()
        q = euler_to_quaternion(0, 0, current_yaw)
        stop_attitude.orientation.x = q[0]
        stop_attitude.orientation.y = q[1]
        stop_attitude.orientation.z = q[2]
        stop_attitude.orientation.w = q[3]
        stop_attitude.thrust = min_thrust
        attitude_pub.publish(stop_attitude)
        
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
