import rclpy
from rclpy.node import Node

import serial
import threading
import time
from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Imu
import struct
from rclpy.clock import Clock
from nav_msgs.msg import Odometry
# 四元数转换工具
from tf2_geometry_msgs import tf2_geometry_msgs
from geometry_msgs.msg import Quaternion
import math
from tf_transformations import quaternion_from_euler

from geometry_msgs.msg import Twist, PoseWithCovariance, TwistWithCovariance

#实现自定义节点类 ，且继承Node这个父类
class Guscar_Base(Node):
    # 初始化guscar_base类(构造函数)
    def __init__(self,node_name):
        # 调用Node这个父类的构造函数(即初始化函数)
        super().__init__(node_name)

        print('node init')

        # 创建速度订阅
        self.vel_sub = self.create_subscription(Twist,'/cmd_vel',self.send_data,10)

        # 创建下位机速度发布
        self.guscar_base_vel_pub = self.create_publisher(Twist,'/guscar/get_vel',10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        # 创建下位机 IMU 数据发布
        # self.guscar_base_imu_pub = self.create_publisher(Imu,'/guscar/imu_raw',10)
        # 累计位置 x(m), y(m), yaw(rad)
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_yaw = 0.0
        # 上一帧时间戳，用于计算采样间隔dt
        self.last_time = self.get_clock().now()
        # 里程校正系数，
        self.odom_x_scale = 2.0
        self.odom_y_scale = 2.0
        self.odom_z_scale_pos = 1.0
        self.odom_z_scale_neg = 1.0

        # 连接下位机串口
        self.connect_ser()

        # # 发送数据给下位机
        

        # 开启一个线程来接收数据
        self.thread = threading.Thread(target=self.receive_data)
        self.thread.start()
        
    # 接收数据
    def receive_data(self):
        print('receive_data_start')
        buffer = bytearray()
        while rclpy.ok():
            
            try:
                n = self.ser.in_waiting
                if n > 0:
                    recv = self.ser.read(n)
                    buffer.extend(recv)
                    # print(f'收到字节:{n},缓存总长:{len(buffer)}')

                # 循环截帧，至少凑够15字节才处理
                while len(buffer) >= 15:
                    # 匹配帧头 0xa5 0xaa
                    if buffer[0] == 0xa5 and buffer[1] == 0xaa:
                        frame = buffer[:15]  # 取出完整15字节帧
                        buffer = buffer[15:] # 切掉已解析帧
                        self.parse_data(frame)
                    else:
                        # 帧头不对，丢弃第一个字节，继续找
                        buffer.pop(0)

            except Exception as e:
                print(f"串口读取异常: {e}")
                time.sleep(0.01)

    # 解析数据    
    def parse_data(self, data):
        # self.send()
        # 1. 校验帧尾
        if data[14] != 0x5a:
            print("丢弃：帧尾0x5a校验失败")
            return

        # 2. 和校验：计算0~12字节总和，对比第13字节
        calc_sum = sum(data[0:13]) & 0xFF  # 取低8位，匹配uint8_t sum
        recv_sum = data[13]
        if calc_sum != recv_sum:
            print(f"丢弃：校验和不匹配 计算:{calc_sum},接收:{recv_sum}")
            return

        # 3. 解析x/y/z 速度（STM32高字节在前，大端，struct用'>h'）
        # x1: 字节2(高) 字节3(低)
        x_int = struct.unpack('>h', data[2:4])[0]
        y_int = struct.unpack('>h', data[4:6])[0]
        z_int = struct.unpack('>h', data[6:8])[0]
        yaw_int = struct.unpack('>h', data[8:10])[0]

        # 除以1000还原实际速度
        x_float = x_int / 1000.0
        y_float = y_int / 1000.0
        z_float = z_int / 1000.0
        yaw_float = yaw_int/1000.0

        # print(f"解析成功 x={x_float:.3f}, y={y_float:.3f}, z={z_float:.3f},yaw={yaw_float:.3f}")

        # 封装Twist消息发布
        twist = Twist()
        twist.linear.x = x_float
        twist.linear.y = y_float
        twist.angular.z = z_float
        twist.linear.z = yaw_float
        self.guscar_base_vel_pub.publish(twist)

        self.update_odometry(x_float, y_float, z_float,yaw_float)

    def update_odometry(self, vx, vy, vz,yaw):
        
        # 1. 计算时间间隔 dt
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # 转秒
        self.last_time = current_time
        if dt <= 0:
            return
        # 2. 速度校正
        vx_cal = vx * self.odom_x_scale
        vy_cal = vy * self.odom_y_scale
        if vz >= 0:
            vz_cal = vz * self.odom_z_scale_pos
        else:
            vz_cal = vz * self.odom_z_scale_neg

        # 3. 二维旋转矩阵积分计算全局位移
        cos_yaw = math.cos(self.pos_yaw)
        sin_yaw = math.sin(self.pos_yaw)
        dx = (vx_cal * cos_yaw - vy_cal * sin_yaw) * dt
        dy = (vx_cal * sin_yaw + vy_cal * cos_yaw) * dt
        # d_yaw = vz_cal * dt

        # 4. 累加至全局位置
        self.pos_x += dx
        self.pos_y += dy
        self.pos_yaw = yaw

        # 5. 构造标准Odometry消息
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"       # 父坐标系
        odom_msg.child_frame_id = "base_footprint" # 子坐标系

        # 位置填充
        odom_msg.pose.pose.position.x = self.pos_x
        odom_msg.pose.pose.position.y = self.pos_y
        odom_msg.pose.pose.position.z = 0.0
        
        # yaw欧拉角转四元数
        q = quaternion_from_euler(0.0, 0.0, self.pos_yaw)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        

        # 当前小车局部速度
        odom_msg.twist.twist.linear.x = vx_cal
        odom_msg.twist.twist.linear.y = vy_cal
        odom_msg.twist.twist.angular.z = vz_cal

        # 简易协方差（可按需调整）
        if abs(vx_cal) < 0.001 and abs(vy_cal) < 0.001 and abs(vz_cal) < 0.001:
            # 静止，误差小
            pose_cov = [1e-6]*36
            twist_cov = [1e-6]*36
        else:
            # 运动，误差大
            pose_cov = [1e-3]*36
            twist_cov = [1e-3]*36
        odom_msg.pose.covariance = pose_cov
        odom_msg.twist.covariance = twist_cov

        print(f"里程计 x={self.pos_x:.3f}, y={self.pos_y:.3f}, z={self.pos_yaw:.3f}")

        # 发布里程计
        self.odom_pub.publish(odom_msg)

    def send(self):
        cmd = [0xb8,0xe6,0x02]
        # 协议长度
        cmd.append(12)
        # 帧尾
        cmd.append(0xd2)
        cmd.append(0xc3)

        self.ser.write(cmd)
    # 发送数据给下位机 
    def send_data(self,msg_data):
        # 帧头0	 帧头1	 类型	协议长度	   x线速度	      y线速度	   角速度	   帧尾0  帧尾1
        # 0xb8	0xe6	0x02	12	      0x00  0x00	0x00  0x00	0x00  0x00	 0xd2	0xc3

        x_vel = msg_data.linear.x
        y_vel = msg_data.linear.y
        angular = msg_data.angular.z

        x_vel2 = bytearray(struct.pack('h',int(x_vel*1000)))
        y_vel2 = bytearray(struct.pack('h',int(y_vel*1000)))
        angular2 = bytearray(struct.pack('h',int(angular*1000)))


        cmd = [0xb8,0xe6,0x02]
        # 协议长度
        cmd.append(12)
        # x线速度
        cmd.append(x_vel2[0]) # 低8位
        cmd.append(x_vel2[1]) # 高8位
        # y线速度
        cmd.append(y_vel2[0])
        cmd.append(y_vel2[1])
        # 角速度
        cmd.append(angular2[0])
        cmd.append(angular2[1])
        # 帧尾
        cmd.append(0xd2)
        cmd.append(0xc3)

        self.ser.write(cmd)


    def connect_ser(self):
        # 尝试多次连接
        count =0
        while count<5:
            count+=1
            try:
                # 开启串口
                self.ser = serial.Serial(port='/dev/ttyUSB0',baudrate=115200)
                # 判断串口是否打开成功
                flag = self.ser.isOpen()        
                print('serial open:'+str(flag))
                # 只要连接成功就退出
                return
            except Exception as e:
                print(e)


    def destroy_node(self):
        print('node end')

        # 关闭串口
        if self.ser is None: return
        self.ser.cancel_read()
        self.ser.close()

# 程序入口方法
def main():

    try:
        # 初始化ROS2的 Pyhton客户端库
        rclpy.init()
        
        # 创建自定义节点实例(对象)   节点名称
        node = Guscar_Base('guscar_base')
        # 阻塞运行，只到节点被关闭
        rclpy.spin(node)
    
        # 关闭ROS2的 Python的客户端库
        rclpy.shutdown()

    except:
        # 销毁节点，释放占用的资源
        node.destroy_node()


    


