import time
import serial
from model import GLOBAL_MODEL as gMODEL
import threading

class STM32Controller:
    def __init__(self):
        if gMODEL.platform == 0:  # Windows
            self.stm32 = serial.Serial('COM3', baudrate=115200, timeout=1)
        elif gMODEL.platform == 1:  # Linux
            self.stm32 = serial.Serial('/dev/ttyTHS1', 115200, timeout=1)
        self.buffer = bytearray() # 接收缓冲区

        # 创建锁
        self.lock = threading.Lock()

    def Data_Receive(self):
        """接收STM32数据"""
        data = None
        
        with self.lock:
            available = self.stm32.in_waiting
        if available:
            with self.lock:
                data = self.stm32.read(available)
        
        if data:
            hex_data = ' '.join(f'{byte:02X}' for byte in data)
            print(f"Received stm32 data (HEX): {hex_data}")
            self.buffer += data
        while len(self.buffer) > 0:
            before_len = len(self.buffer)

            index = 0
            while index <= len(self.buffer):
                processed = False
            
                # print(' '.join(f'{byte:02X}' for byte in self.buffer))

                # 先处理到位信号（4字节协议）
                if index + 4 <= len(self.buffer):
                    # 检查协议特征：第1字节0x39，第2字节0xFF，第3字节0xFF，第4字节0x39
                    if (self.buffer[index] == 0x39 and self.buffer[index+1] == 0xFF
                        and self.buffer[index+2] == 0xFF and self.buffer[index+3] == 0x39):
                        gMODEL.isCarReached.set(1)  # 设置到位标志
                        index += 4  # 移除已处理数据
                        processed = True

                # 处理小车旋转到位信号（4字节协议）(0x40 0xFF 0xFF 0x40)
                if not processed and index + 4 <= len(self.buffer):
                    if (self.buffer[index] == 0x40 and self.buffer[index+1] == 0xFF
                        and self.buffer[index+2] == 0xFF and self.buffer[index+3] == 0x40):
                        gMODEL.isRotateReached.set(1)  # 设置旋转到位标志
                        index += 4  # 移除已处理数据
                        processed = True

                # # 处理Poscali_Enable指令（0x93 0x11 sta 0xFF）
                # if not processed and index + 4 <= len(self.buffer):
                #     if (self.buffer[index] == 0x93 and 
                #         self.buffer[index+1] == 0x11 and 
                #         self.buffer[index+3] == 0xFF):
                #         sta = self.buffer[index+2]
                #         if sta in (0x00, 0x01):
                #             Poscali_Enable(sta)
                #             index += 4
                #             processed = True

                # # 处理Poscali_DirChoose指令（0x93 0x12 dir 0xFF）
                # if not processed and index + 4 <= len(self.buffer):
                #     if (self.buffer[index] == 0x93 and 
                #         self.buffer[index+1] == 0x12 and 
                #         self.buffer[index+3] == 0xFF):
                #         dir = self.buffer[index+2]
                #         if dir in (0x01, 0x02, 0x03, 0x04):
                #             Poscali_DirChoose(dir)
                #             index += 4
                #             processed = True

                if not processed:
                    if index >= len(self.buffer):
                        break
                    index += 1
            
            #更新缓冲区
            self.buffer = self.buffer[index:]
            if len(self.buffer) == before_len:
                break

    def Servo_Move(self, addr, angle):
        Ang_10 = int(abs(angle) * 10.0) & 0xFFFF
        cmd = bytearray(9)
        cmd[0] = 0x39                   # 协议头
        cmd[1] = 0x41                   # 功能码
        cmd[2] = addr                   # 舵机地址
        cmd[3] = Ang_10 & 0xFF          # 转动角度低八位（扩大10倍）
        cmd[4] = (Ang_10 >> 8) & 0xFF   # 转动角度高八位（扩大10倍）
        cmd[5] = 0x00                   # 保留
        cmd[6] = 0x00                   # 保留
        cmd[7] = 0x00                   # 保留
        cmd[8] = 0xFF                   # 校验位
        with self.lock:
            self.stm32.write(cmd)

    def Gripper_En(self, sta): # 夹爪控制
        """控制夹爪"""
        if sta:
            self.Servo_Move(1, 258.0) # 闭合
        else:
            self.Servo_Move(1, 215.0) # 松开

    def Servo2_Ctrl(self, sta): # 摄像头舵机控制
        """控制摄像头舵机"""
        if sta:
            self.Servo_Move(2, 180.6) # 前转
        else:
            self.Servo_Move(2, 150.6) # 默认

    def Servo3_Ctrl(self, sta): # 托盘下舵机控制
        """控制托盘下舵机"""
        if sta==0:
            self.Servo_Move(3, 268) # 初始位置
        elif sta==1:
            self.Servo_Move(3, 178) # 顺时针90度
        elif sta==2:
            self.Servo_Move(3, 88) # 工作位置（顺180）
        elif sta==3:
            # self.Servo_Move(3, 48.0)
            self.Servo_Move(3, 25) # 避让位置Dir2
        elif sta==4:
            self.Servo_Move(3, 149) # 避让位置Dir3

    def Servo4_Ctrl(self, sta): # 托盘上舵机控制（012逆时针分布）
        """控制托盘上舵机"""
        if sta==0:
            self.Servo_Move(4, 209.0) # 0号位
            # self.Servo_Move(4, 233.0)
        elif sta==1:
            self.Servo_Move(4, 124.0) # 1号位
        elif sta==2:
            self.Servo_Move(4, 42.0)  # 2号位
        elif sta==3:
            self.Servo_Move(4, 233.0) # 避让位置Dir2
        elif sta==4:
            self.Servo_Move(4, 20.0) # 避让位置Dir3

    def CarMove_Rel(self, XYt):
        """小车相对运动"""
        if(XYt[2] <= 0):
            print("无效时间！")
            return
        gMODEL.isCarReached.set(0)
        t_X = int(XYt[0]) & 0xFFFF
        t_Y = int(XYt[1]) & 0xFFFF
        time = int(XYt[2]) & 0xFF
        cmd = bytearray(9)
        cmd[0] = 0x39                   # 协议头
        cmd[1] = 0x22                   # 功能码
        cmd[2] = t_X & 0xFF             # X坐标低八位
        cmd[3] = (t_X >> 8) & 0xFF      # X坐标高八位
        cmd[4] = t_Y & 0xFF             # Y坐标低八位
        cmd[5] = (t_Y >> 8) & 0xFF      # Y坐标高八位
        cmd[6] = time                   # 移动时间（单位为0.1s）
        cmd[7] = 0x00                   # 保留
        cmd[8] = 0xFF                   # 校验位
        with self.lock:
            self.stm32.write(cmd)

    def CarMove_Abs(self, XYt):
        """小车绝对运动"""
        if(XYt[0] < 0 or XYt[1] < 0 or XYt[2] < 0):
            print("无效移动参数！")
            return
        gMODEL.isCarReached.set(0)
        t_X = int(XYt[0]) & 0xFFFF
        t_Y = int(XYt[1]) & 0xFFFF
        time = int(XYt[2]) & 0xFF
        cmd = bytearray(9)
        cmd[0] = 0x39                   # 协议头
        cmd[1] = 0x21                   # 功能码
        cmd[2] = t_X & 0xFF             # X坐标低八位
        cmd[3] = (t_X >> 8) & 0xFF      # X坐标高八位
        cmd[4] = t_Y & 0xFF             # Y坐标低八位
        cmd[5] = (t_Y >> 8) & 0xFF      # Y坐标高八位
        cmd[6] = time                   # 移动时间（单位为0.1s）
        cmd[7] = 0x00                   # 保留
        cmd[8] = 0xFF                   # 校验位
        with self.lock:
            self.stm32.write(cmd)

    def Circle_Centre_Send(self, circle_x, circle_y):
        """发送定位色环中心坐标"""
        circle_x_10 = int(circle_x * 10.0) & 0xFFFF
        circle_y_10 = int(circle_y * 10.0) & 0xFFFF
        cmd = bytearray(9)
        cmd[0] = 0x39                           # 协议头
        cmd[1] = 0x31                           # 功能码
        cmd[2] = circle_x_10 & 0xFF             # 色环X相对坐标低八位
        cmd[3] = (circle_x_10 >> 8) & 0xFF      # 色环X相对坐标高八位
        cmd[4] = circle_y_10 & 0xFF             # 色环Y相对坐标低八位
        cmd[5] = (circle_y_10 >> 8) & 0xFF      # 色环Y相对坐标高八位
        cmd[6] = 0x00                           # 保留
        cmd[7] = 0x00                           # 保留
        cmd[8] = 0xFF                           # 校验位
        with self.lock:
            self.stm32.write(cmd)

    def Start_or_Stop_PosCali(self, en, dir):
        """启动或停止小车定位"""
        cmd = bytearray(9)
        cmd[0] = 0x39                   # 协议头
        cmd[1] = 0x32                   # 功能码
        cmd[2] = 0x01 if en else 0x00   # 启动或停止
        cmd[3] = dir                    # 方向
        cmd[4] = 0x00                   # 保留
        cmd[5] = 0x00                   # 保留
        cmd[6] = 0x00                   # 保留
        cmd[7] = 0x00                   # 保留
        cmd[8] = 0xFF                   # 校验位
        with self.lock:
            self.stm32.write(cmd)

    def Clear_GyroErr(self, state): # 消除陀螺仪误差 0：静止任务开始前执行 1：静止任务完成后执行
        """清除陀螺仪误差"""
        time.sleep(0.01)
        cmd = bytearray(9)
        cmd[0] = 0x39                   # 协议头
        cmd[1] = 0x51                   # 功能码
        cmd[2] = 0x01 if state else 0x00   # 启动或停止
        cmd[3] = 0x00                   # 保留
        cmd[4] = 0x00                   # 保留
        cmd[5] = 0x00                   # 保留
        cmd[6] = 0x00                   # 保留
        cmd[7] = 0x00                   # 保留
        cmd[8] = 0xFF                   # 校验位
        with self.lock:
            self.stm32.write(cmd)
        time.sleep(0.21)

    def Car_Rotate(self, angle, speed): # 小车旋转，angle逆时针为正，speed速度为绝对值
        """小车旋转"""
        if speed <= 0:
            print("无效速度！")
            return
        gMODEL.isRotateReached.set(0)  # 重置旋转到位标志
        Angle = int(angle) & 0xFFFF
        Speed = int(speed) & 0xFF
        cmd = bytearray(9)
        cmd[0] = 0x39                   # 协议头
        cmd[1] = 0x61                   # 功能码
        cmd[2] = Angle & 0xFF           # 旋转角度低八位
        cmd[3] = (Angle >> 8) & 0xFF    # 旋转角度高八位
        cmd[4] = Speed                  # 旋转速度
        cmd[5] = 0x00                   # 保留
        cmd[6] = 0x00                   # 保留
        cmd[7] = 0x00                   # 保留
        cmd[8] = 0xFF                   # 校验位
        with self.lock:
            self.stm32.write(cmd)

    def LED_En(self, state): # 补光灯控制
        """控制补光灯"""
        cmd = bytearray(9)
        cmd[0] = 0x39                   # 协议头
        cmd[1] = 0x71                   # 功能码
        cmd[2] = 0x01 if state else 0x00   # 启动或停止
        cmd[3] = 0x00                   # 保留
        cmd[4] = 0x00                   # 保留
        cmd[5] = 0x00                   # 保留
        cmd[6] = 0x00                   # 保留
        cmd[7] = 0x00                   # 保留
        cmd[8] = 0xFF                   # 校验位
        with self.lock:
            self.stm32.write(cmd)

if __name__ == "__main__":
    pass