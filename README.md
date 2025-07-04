# px4_keyboard_control
px4无人机键盘控制脚本

分别使用mavros话题的速度（velocity）、位置（position）、姿态（attitude）进行编写，三个python脚本

5 切换offboard模式
6 解锁
7 起飞到固定高度
i 前进
，后退
j 左移
l 右移
r 上升
f 下降
k 强制停止
空格 降落

其中速度和位置控制的非常稳定，姿态控制的不是很稳定。

环境：ubuntu20.04，ros noetic，px4固件版本1.13.2
