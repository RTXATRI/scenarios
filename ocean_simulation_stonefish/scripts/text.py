#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import sys
import select
import tty
import termios

# 初始化ROS节点
rospy.init_node('simple_thruster_control')

# 创建推进器发布器
left_thruster_pub = rospy.Publisher('/under_usv1/control/thruster/left', Float64, queue_size=10)
right_thruster_pub = rospy.Publisher('/under_usv1/control/thruster/right', Float64, queue_size=10)
VerticalThrusterQuadrant1_pub = rospy.Publisher('/under_usv1/control/thruster/VerticalThrusterQuadrant1', Float64, queue_size=10)
VerticalThrusterQuadrant2_pub = rospy.Publisher('/under_usv1/control/thruster/VerticalThrusterQuadrant2', Float64, queue_size=10)
VerticalThrusterQuadrant3_pub = rospy.Publisher('/under_usv1/control/thruster/VerticalThrusterQuadrant3', Float64, queue_size=10)
VerticalThrusterQuadrant4_pub = rospy.Publisher('/under_usv1/control/thruster/VerticalThrusterQuadrant4', Float64, queue_size=10)

# 等待发布器连接
rospy.sleep(1.0)

def control_thrusters(left_power, right_power):
    """控制推进器功率，范围-1.0到1.0"""
    left_msg = Float64()
    left_msg.data = left_power
    right_msg = Float64()
    right_msg.data = right_power
    
    left_thruster_pub.publish(left_msg)   # 控制左侧推进器
    right_thruster_pub.publish(right_msg) # 控制右侧推进器
    
    print(f"推进器命令: 左={left_power}, 右={right_power}")

def is_key_pressed():
    """检测是否有按键按下"""
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def get_key():
    """获取按下的键"""
    return sys.stdin.read(1)

def forward():
    """前进功能"""
    print("执行前进功能")
    control_thrusters(-0.5, 0.5)  # 前进的推进器功率设置

def backward():
    """后退功能"""
    print("执行后退功能")
    control_thrusters(0.5, -0.5)  # 后退的推进器功率设置

def turn_left():
    """左转功能"""
    print("执行左转功能")
    control_thrusters(-0.3, -0.3)  # 左转的推进器功率设置

def turn_right():
    """右转功能"""
    print("执行右转功能")
    control_thrusters(0.3, 0.3)    # 右转的推进器功率设置

def move_up():
    """上浮功能（如果适用）"""
    msg1 = Float64()
    msg2 = Float64()
    msg3 = Float64()
    msg4 = Float64()
    msg1.data = 0.5
    msg2.data = -0.5
    msg3.data = 0.5
    msg4.data = -0.5
    VerticalThrusterQuadrant1_pub.publish(msg1)
    VerticalThrusterQuadrant2_pub.publish(msg2)
    VerticalThrusterQuadrant3_pub.publish(msg3)
    VerticalThrusterQuadrant4_pub.publish(msg4)
    print("执行上浮功能\n")
    # 如果有垂直推进器，可以在这里添加控制代码
    # control_thrusters(0.0, 0.0)  # 根据实际推进器配置修改

def move_down():
    """下潜功能（如果适用）"""
    msg1 = Float64()
    msg2 = Float64()
    msg3 = Float64()
    msg4 = Float64()
    msg1.data = -0.5
    msg2.data = 0.5
    msg3.data = -0.5
    msg4.data = 0.5
    VerticalThrusterQuadrant1_pub.publish(msg1)
    VerticalThrusterQuadrant2_pub.publish(msg2)
    VerticalThrusterQuadrant3_pub.publish(msg3)
    VerticalThrusterQuadrant4_pub.publish(msg4)
    print("执行下潜功能\n")
    # 如果有垂直推进器，可以在这里添加控制代码
    # control_thrusters(0.0, 0.0)  # 根据实际推进器配置修改

def stop():
    """停止所有推进器"""
    print("停止推进器\n")
    control_thrusters(0.0, 0.0)
    msg1 = Float64()
    msg2 = Float64()
    msg3 = Float64()
    msg4 = Float64()
    msg1.data = 0.0
    msg2.data = 0.0
    msg3.data = 0.0
    msg4.data = 0.0
    VerticalThrusterQuadrant1_pub.publish(msg1)
    VerticalThrusterQuadrant2_pub.publish(msg2)
    VerticalThrusterQuadrant3_pub.publish(msg3)
    VerticalThrusterQuadrant4_pub.publish(msg4)

def main():
    # 保存原始终端设置
    old_settings = termios.tcgetattr(sys.stdin)
    
    try:
        # 设置终端为原始模式，以便实时读取按键
        tty.setraw(sys.stdin.fileno())
        
        print("键盘控制已启动，使用以下按键控制USV:")
        print("↑ : 前进")
        print("↓ : 后退") 
        print("W : 上浮（如果适用）")
        print("S : 下潜（如果适用）")
        print("A : 左转")
        print("D : 右转")
        print("空格键: 停止")
        print("Q : 退出程序")
        print("------------------------")
        
        while not rospy.is_shutdown():
            if is_key_pressed():
                key = get_key()
                
                if key == '\x1b':  # 可能是方向键的前缀
                    # 读取额外的两个字符来识别方向键
                    additional_chars = sys.stdin.read(2)
                    if additional_chars == '[A':  # 上箭头
                        move_up()
                    elif additional_chars == '[B':  # 下箭头
                        move_down()
                elif key == ' ':  # 空格键
                    stop()
                
                elif key == 'q' or key == 'Q':
                    print("退出程序")
                    break
            
            # 添加短暂延迟以减少CPU使用率
            rospy.sleep(0.01)
    
    except Exception as e:
        print(f"发生错误: {e}")
    
    finally:
        # 恢复原始终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        # 确保程序退出时推进器停止
        stop()
        print("程序已退出，推进器已停止")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # 确保程序退出时推进器停止
        stop()
        print("程序中断，推进器已停止")