import threading
import time
import math
import ZDT
from model import GLOBAL_MODEL as gMODEL
from robocal import MotionParams as MP
from presenter import AppPresenter
from stm32 import STM32Controller

class TaskProcessor:
    """任务处理器"""
    
    def __init__(self, presenter: AppPresenter):
        self.PRE = presenter

        # 创建别名，提高书写效率
        self.arm = self.PRE.arm
        self.vision = self.PRE.vision
        self.stm32 = self.PRE.STM32
        self.pid1 = self.PRE.pid1
        self.pid2 = self.PRE.pid2
        
        # 任务中的关键变量
        self.feedback_distance = 0.0  # 视觉跟踪反馈距离
        self.feedback_counter = 0  # 视觉跟踪反馈计数器
        self.relX = 0.0  # 视觉识别中相对画面中心的X坐标
        self.relY = 0.0  # 视觉识别中相对画面中心的Y坐标

        # 后台任务线程状态标志
        self.running_bg_tasks = False
        self.exit_flag_bg_tasks = False
        self.thread_bg_tasks = None

        # 主任务线程状态标志
        self.running_main_task = False
        self.exit_flag_main_task = False
        self.thread_main_task = None

        # 初始化PID_Task变量
        self.PID_Task_target_update_flag = True  # PID目标更新标志
        self.PID_Task_speed_clear_flag = False # PID速度清零标志
        self.PID_Task_set_speed = {"x": 0.0, "y": 0.0, "total": 0.0} # PID设置速度
        self.PID_Task_reached_flag = False  # 用于判断是否已经到达目标位置

        # 控制后台任务执行频率参数
        self.prev_stm32_receive_time = 0.0  # 上次STM32数据接收的时间
        self.prev_zdt_receive_time = 0.0  # 上次ZDT数据接收的时间
        self.prev_message_update_time = 0.0  # 上次消息更新的时间
        self.prev_pid_task_time = 0.0  # 上次PID任务执行的时间

    def run_bg_tasks(self):
        """后台任务线程函数"""
        while not self.exit_flag_bg_tasks:
            time.sleep(0.001)
            curr_time = time.time()

            # 每5秒更新一次系统状态
            if int(curr_time) % 5 == 0:
                self.PRE.update_system_status(f"运行中 - {time.strftime('%H:%M:%S')}")

            if curr_time - self.prev_stm32_receive_time >= 0.01: # 0.01秒周期
                self.stm32.Data_Receive()
                self.prev_stm32_receive_time = curr_time

            if curr_time - self.prev_zdt_receive_time >= 0.01: # 0.01秒周期
                ZDT.Receive()
                self.prev_zdt_receive_time = curr_time

            if curr_time - self.prev_message_update_time >= 0.1: # 0.1秒周期
                self.Message_Update_Task()
                self.prev_message_update_time = curr_time

            if curr_time - self.prev_pid_task_time >= 0.02: # 0.02秒周期
                self.PID_Task()
                self.prev_pid_task_time = curr_time



    # ======== 主任务高效工具函数 ========= #
    def Arm_Init(self):
        """机械臂姿态初始化"""
        # self.PRE._move_to_start_position()
        self.PRE._move_to_debug_position()

    def ArmTo_PosFix(self):
        """机械臂移动至小车定位点"""
        self.arm.MoveP_AccDec(0, 298, 155, MP.VelAcc.ToTrackBegin)

    def ArmFix_ToTrayHover(self):
        """小车定位点到托盘悬停点"""
        ZDT.Emm_V5_Pos_Control(3, 0, 2500, 210, 2800, 1, 0)
        time.sleep(0.2)
        self.arm.MoveJ_AccDec(MP.Pos_TrayHover, MP.VelAcc.ArmFix_ToTrayHover)

    def get_r(self): # 获取小车到位标志
        """获取小车到位标志"""
        return gMODEL.isCarReached.get()

    def clear_r(self): # 清除小车到位标志
        """清除小车到位标志"""
        gMODEL.isCarReached.set(0)

    def carmove_r(self, xyt):
        """小车相对移动"""
        self.stm32.CarMove_Rel(xyt)

    def car_poscal(self, sta):
        """小车位置校准"""
        self.vision.poscali_enable_flag = sta
        self.stm32.Start_or_Stop_PosCali(sta, self.arm.armDir)
    # =================================== #

    def run_main_task(self): # 主任务流程，使用状态机
        """主任务线程函数"""
        LED_En = False  # 是否开启LED补光灯

        # 机械臂姿态初始化
        self.Arm_Init()

        # 任务状态初始化
        go_state = 111 # 跑图流程状态，111表示初始状态

        task1_state = 111 # 子任务流程状态
        task2_state = 111
        task3_1_state = 111
        task3_2_state = 111
        task3_3_state = 111

        curNum_1 = 0 # 子任务中，一个抓取或放置流程中的第几个物料
        curNum_2_1 = 0
        curNum_2_2 = 0
        curNum_3_1 = 0
        curNum_3_2 = 0
        curNum_3_3 = 0

        color_order = self.vision.read_colour_order[0] # 当前批次颜色顺序
        self.vision.choose_colour = color_order[curNum_1] # 当前颜色初始化

        # 关键点位信息、速度加速度参数
        gMODEL.catchHeight = 115 # 原料区物料抓取高度

        circle_x = [150, 0, -150] # 色环X坐标，从右向左红绿蓝
        circle_xyz = [[150, 325, 32.5], [0 ,325, 31.5], [-150, 325, 32.5]] # 色环XYZ坐标，从右向左红绿蓝
        circle_z_delta = 2 # 色环物料抓取时与放置时z轴差值
        VA_cir2trayH_dir2 = [
            MP.VelAcc.ToTrayHover_Slow, 
            MP.VelAcc.ToTrayHover_Fast, 
            MP.VelAcc.ToTrayHover_Fast
        ] # 机械臂朝向2 色环到托盘区悬停点的顺序速度加速度，从右向左红绿蓝
        VA_cir2trayP_dir2 = [
            MP.VelAcc.Dir23toTrayPlace_Slow,
            MP.VelAcc.Dir23toTrayPlace_Fast,
            MP.VelAcc.Dir23toTrayPlace_Fast
        ] # 机械臂朝向2 色环到托盘区放置点的顺序速度加速度，从右向左红绿蓝
        VA_cir2trayH_dir3 = [
            MP.VelAcc.ToTrayHover_Fast,
            MP.VelAcc.ToTrayHover_Fast,
            MP.VelAcc.ToTrayHover_Slow
        ] # 机械臂朝向3 色环到托盘区悬停点的顺序速度加速度，从右向左红绿蓝
        VA_cir2trayP_dir3 = [
            MP.VelAcc.Dir23toTrayPlace_Fast,
            MP.VelAcc.Dir23toTrayPlace_Fast,
            MP.VelAcc.Dir23toTrayPlace_Slow
        ] # 机械臂朝向3 色环到托盘区放置点的顺序速度加速度，从右向左红绿蓝

        Point_XYt = [
            [0, 0, 0],
            [0, 122, 10],
            [659, 0, 17],
            [800, 0, 24],
            [-400, 0, 15],
            [0, 1738, 41],
            [827, 0, 23],
            [0, -827, 23],
            [0, -908, 23],
            [-427, 0, 23],
            [-400, 0, 15],
            [0, 1738, 41],
            [827, 0, 23],
            [0, -827, 23],
            [0, -911, 22],
            [-1872, 0, 42],
            [0, -122, 6]
        ]

        while not self.exit_flag_main_task:
            time.sleep(0.001)

            if gMODEL.mainTaskIsRunning == True:
                # 主任务正在运行
                if go_state == 111: # 初始状态
                    self.arm.set_armDir(1) # 设置机械臂方向为1
                    self.vision.Detect_Mode = 0 # 设置视觉检测模式为0, 物料
                    go_state = 0 # 进入下一状态

                if go_state == 0: 
                    self.carmove_r(Point_XYt[1]) # Y + 122, abs: 0, 122
                    self.PRE._move_to_debug_position()
                    go_state = 1

                if go_state == 1 and self.get_r():
                    self.clear_r()
                    self.carmove_r(Point_XYt[2]) # X + 659, abs: 659, 122
                    go_state = 2

                if go_state == 2 and self.get_r():
                    self.clear_r()
                    self.vision.start_QR_Read() # 开始读取QR码
                    go_state = 3

                if go_state == 3 and (self.vision.QR_Read_Flag_Success or self.vision.QR_Read_Flag_Timeout):
                    self.carmove_r(Point_XYt[3]) # X + 800, abs: 1459, 122
                    time.sleep(1.0)
                    ZDT.Emm_V5_Pos_Control(4,1,1200,120,MP.J4clk_normal,1,0)
                    time.sleep(0.005)
                    self.arm.MoveP_AccDec(0, 370, 230, MP.VelAcc.ToTrackBegin) # 机械臂移动到物料跟踪开始位置
                    go_state = 4

                # 到达原料区
                if go_state == 4 and self.get_r(): # ---------------------------------------- 子任务1_0
                    self.clear_r()
                    # time.sleep(0.4)
                    self.stm32.Clear_GyroErr(0) # 清除陀螺仪误差, 部署前
                    gMODEL.currentBatch = 0
                    gMODEL.subTaskNum = 1_0 # 子任务1_0：原料区物料抓取
                    go_state = 5

                if go_state == 5 and gMODEL.subTaskNum == 0: # 子任务完成
                    self.stm32.Clear_GyroErr(1) # 清除陀螺仪误差, 部署后
                    self.carmove_r(Point_XYt[4]) # X - 400, abs: 1059, 122
                    go_state = 6

                if go_state == 6 and self.get_r():
                    self.clear_r()
                    self.carmove_r(Point_XYt[5]) # Y + 1738, abs: 1059, 1860
                    time.sleep(1.5) # 小车路途中开始移动机械臂
                    self.arm.set_armDir(2) # 设置机械臂方向为2
                    self.vision.Detect_Mode = 1 # 设置视觉检测模式为1, 色环
                    self.vision.choose_colour = 2 # 识别绿色色环，用于定位
                    self.ArmTo_PosFix() # 机械臂移动到小车定位点
                    if LED_En:
                        self.stm32.LED_En(1) # 开启LED补光灯
                    go_state = 7

                # 到达粗加工区
                if go_state == 7 and self.get_r():
                    self.clear_r()
                    self.car_poscal(True) # 小车位置校准
                    go_state = 8

                if go_state == 8 and self.vision.distance < 2: # ---------------------------- 子任务2_0
                    self.car_poscal(False)
                    time.sleep(0.08)
                    if LED_En:
                        self.stm32.LED_En(0)

                    self.ArmFix_ToTrayHover()
                    gMODEL.taskLock = 1 # 子任务保险锁，为1时才能执行子任务2_0
                    time.sleep(0.8)
                    self.stm32.Clear_GyroErr(0)
                    gMODEL.subTaskNum = 2_0 # 子任务2_0：物料放置到色环后抓回物料仓
                    go_state = 9

                if go_state == 9 and gMODEL.subTaskNum == 0: # 子任务完成
                    self.stm32.Clear_GyroErr(1)
                    self.carmove_r(Point_XYt[6]) # X + 827, abs: 1886, 1860
                    go_state = 10

                if go_state == 10 and self.get_r():
                    self.clear_r()
                    self.carmove_r(Point_XYt[7]) # Y - 827, abs: 1886, 1033
                    time.sleep(0.1)
                    self.arm.set_armDir(3) # 设置机械臂方向为3
                    self.vision.Detect_Mode = 1 # 设置视觉检测模式为1, 色环
                    self.vision.choose_colour = 2 # 识别绿色色环，用于定位
                    self.ArmTo_PosFix() # 机械臂移动到小车定位点
                    if LED_En:
                        self.stm32.LED_En(1)
                    go_state = 11
                
                # 到达暂存区
                if go_state == 11 and self.get_r():
                    self.clear_r()
                    self.car_poscal(True) # 小车位置校准
                    go_state = 12

                if go_state == 12 and self.vision.distance < 2: # ---------------------------- 子任务3_1
                    self.car_poscal(False)
                    time.sleep(0.08)
                    if LED_En:
                        self.stm32.LED_En(0)

                    self.ArmFix_ToTrayHover()
                    gMODEL.taskLock = 1
                    time.sleep(0.8)
                    self.stm32.Clear_GyroErr(0)
                    gMODEL.subTaskNum = 3_1 # 子任务3_1：物料放置到色环
                    go_state = 13

                if go_state == 13 and gMODEL.subTaskNum == 0: # 子任务完成
                    self.stm32.Clear_GyroErr(1)
                    self.carmove_r(Point_XYt[8]) # Y - 908, abs: 1886, 125
                    go_state = 14

                if go_state == 14 and self.get_r():
                    self.clear_r()
                    self.carmove_r(Point_XYt[9]) # X - 427, abs: 1459, 125
                    time.sleep(0.1)
                    self.arm.set_armDir(1) # 设置机械臂方向为1
                    self.vision.Detect_Mode = 0 # 设置视觉检测模式为0, 物料
                    self.arm.MoveP_AccDec(0, 370, 230, MP.VelAcc.ToTrackBegin) # 机械臂移动到物料跟踪开始位置
                    go_state = 15

                # 到达原料区， 开始第二批物料搬运
                if go_state == 15 and self.get_r(): # ---------------------------------------- 子任务1_0
                    self.clear_r()
                    gMODEL.currentBatch = 1
                    # time.sleep(0.4)
                    self.stm32.Clear_GyroErr(0) # 清除陀螺仪误差, 部署前
                    gMODEL.subTaskNum = 1_0 # 子任务1_0：原料区物料抓取
                    go_state = 16

                if go_state == 16 and gMODEL.subTaskNum == 0: # 子任务完成
                    self.stm32.Clear_GyroErr(1)
                    self.carmove_r(Point_XYt[10]) # X - 400, abs: 1059, 125
                    go_state = 17

                if go_state == 17 and self.get_r():
                    self.clear_r()
                    self.carmove_r(Point_XYt[11]) # Y + 1738, abs: 1059, 1863
                    time.sleep(1.5) # 小车路途中开始移动机械臂
                    self.arm.set_armDir(2) # 设置机械臂方向为2
                    self.vision.Detect_Mode = 1 # 设置视觉检测模式为1, 色环
                    self.vision.choose_colour = 2 # 识别绿色色环，用于定位
                    self.ArmTo_PosFix() # 机械臂移动到小车定位点
                    if LED_En:
                        self.stm32.LED_En(1)
                    go_state = 18

                # 到达粗加工区
                if go_state == 18 and self.get_r():
                    self.clear_r()
                    self.car_poscal(True) # 小车位置校准
                    go_state = 19

                if go_state == 19 and self.vision.distance < 2: # ---------------------------- 子任务2_0
                    self.car_poscal(False)
                    time.sleep(0.08)
                    if LED_En:
                        self.stm32.LED_En(0)
                    self.ArmFix_ToTrayHover()
                    gMODEL.taskLock = 1 # 子任务保险锁，为1时才能执行子任务2_0
                    time.sleep(0.8)
                    self.stm32.Clear_GyroErr(0)
                    gMODEL.subTaskNum = 2_0
                    go_state = 20

                if go_state == 20 and gMODEL.subTaskNum == 0: # 子任务完成
                    self.stm32.Clear_GyroErr(1)
                    self.carmove_r(Point_XYt[12]) # X + 827, abs: 1886, 1863
                    go_state = 21
                
                if go_state == 21 and self.get_r():
                    self.clear_r()
                    self.carmove_r(Point_XYt[13]) # Y - 827, abs: 1886, 1036
                    time.sleep(0.1)
                    self.arm.set_armDir(3) # 设置机械臂方向为3
                    self.vision.Detect_Mode = 1 # 设置视觉检测模式为1, 色环
                    self.vision.choose_colour = 2 # 识别绿色色环，用于定位
                    self.ArmTo_PosFix() # 机械臂移动到小车定位点
                    go_state = 22

                # 到达暂存区
                if go_state == 22 and self.get_r():
                    self.clear_r()
                    self.car_poscal(True) # 小车位置校准
                    go_state = 23

                if go_state == 23 and self.vision.distance < 2: # ---------------------------- 子任务3_3
                    self.car_poscal(False)
                    time.sleep(0.08)

                    self.ArmFix_ToTrayHover()
                    gMODEL.taskLock = 1
                    time.sleep(0.8)
                    self.stm32.Clear_GyroErr(0)
                    gMODEL.subTaskNum = 3_3
                    go_state = 24

                if go_state == 24 and gMODEL.subTaskNum == 0: # 子任务完成
                    self.stm32.Clear_GyroErr(1)
                    self.carmove_r(Point_XYt[14]) # Y - 911, abs: 1886, 125
                    go_state = 25

                if go_state == 25 and self.get_r():
                    self.clear_r()
                    self.carmove_r(Point_XYt[15]) # X - 1872, abs: 14, 125
                    go_state = 26

                if go_state == 26 and self.get_r():
                    self.clear_r()
                    self.carmove_r(Point_XYt[16]) # Y - 122, abs: 14, 3
                    go_state = 27
                
                # 到达终点
                if go_state == 27 and self.get_r():
                    self.clear_r()
                    gMODEL.mainTaskIsRunning = False # 任务结束
                    go_state = 111 # 重置状态机



            # 机械臂朝向：Dir1
            if gMODEL.subTaskNum == 1_0: # 子任务1_0：原料区物料抓取
                if task1_state == 111:
                    self.stm32.Servo3_Ctrl(2) # 工作位置（顺180）
                    time.sleep(0.7)
                    self.pid1.update_params(kp=20.00, ki=0.000, kd=12.00)
                    self.pid2.update_params(kp=15.00, ki=0.001, kd=10.00)
                    color_order = self.vision.read_colour_order[gMODEL.currentBatch] # 当前批次颜色顺序

                    task1_state = 0 # 进入下一状态

                if task1_state == 0:
                    self.stm32.Servo4_Ctrl(curNum_1) # 托盘上舵机，0 -> 1 -> 2

                    self.vision.choose_colour = color_order[curNum_1] # 识别当前颜色

                    gMODEL.speedmodeEnabled = True # 启用速度模式
                    gMODEL.trackingEnabled = True # 启用跟踪模式

                    self.arm.trigger_MoveZ_inSpeed(gMODEL.catchHeight, 0.7) # 触发Z轴速度模式运动，抓取高度

                    task1_state = 1 # 进入下一状态

                if task1_state == 1 and self.arm.Arm_Tz < (gMODEL.catchHeight + 1) and self.arm.Arm_Ty < 450.0:
                    # 机械臂Z轴到达抓取高度，Y轴小于450  防止机械臂伸出太远
                    if self.vision.relative_y < 0: # 物料在夹爪内侧，可直接抓取
                        time.sleep(0.03)
                        self.stm32.Gripper_En(1)
                        time.sleep(0.1)
                        
                        gMODEL.speedmodeEnabled = False # 关闭速度模式
                        gMODEL.trackingEnabled = False # 关闭跟踪模式
                        time.sleep(0.1)

                        ZDT.Emm_V5_Reached_En(1, True) # 启用电机1到位返回
                        time.sleep(0.05)

                        if self.arm.Arm_Ty < 440.0:
                            self.arm.MoveJ_AccDec(MP.Pos_TrayPlace, MP.VelAcc.Dir1toTrayPlace_Fast) # 机械臂移动到托盘区放置点
                        else:
                            self.arm.MoveJ_AccDec(MP.Pos_TrayPlace, MP.VelAcc.Dir1toTrayPlace_Fast)

                        task1_state = 2 # 进入下一状态
                    elif self.vision.relative_y >= 0 and self.vision.distance < 20: # 物料在夹爪外侧
                        time.sleep(0.03)
                        self.stm32.Gripper_En(1) # 夹爪闭合
                        time.sleep(0.1)

                        gMODEL.speedmodeEnabled = False # 关闭速度模式
                        gMODEL.trackingEnabled = False # 关闭跟踪模式
                        time.sleep(0.1)

                        ZDT.Emm_V5_Reached_En(1, True) # 启用电机1到位返回
                        time.sleep(0.05)

                        if self.arm.Arm_Ty < 440.0:
                            self.arm.MoveJ_AccDec(MP.Pos_TrayPlace, MP.VelAcc.Dir1toTrayPlace_Fast)
                        else:
                            self.arm.MoveJ_AccDec(MP.Pos_TrayPlace, MP.VelAcc.Dir1toTrayPlace_Slow)

                        task1_state = 2 # 进入下一状态

                if task1_state == 2 and ZDT.Reached_Flag:
                    # 机械臂到位
                    ZDT.set_reached_flag(0) # 清除到位标志
                    print("1号电机到位!")
                    time.sleep(0.03)

                    self.stm32.Gripper_En(0) # 夹爪打开
                    time.sleep(0.01)

                    if curNum_1 < 2:
                        self.arm.MoveP_AccDec(0, 370, 230, MP.VelAcc.ToTrackBegin) # 机械臂移动到物料跟踪开始位置
                        self.vision.choose_colour = color_order[curNum_1 + 1] # 更新颜色选择
                        task1_state = 3 # 进入下一状态
                    elif curNum_1 == 2:
                        ZDT.Emm_V5_Reached_En(1, False) # 禁用电机1到位返回
                        time.sleep(0.03)

                        self.arm.MoveJ_AccDec(MP.Pos_TrayHover, MP.VelAcc.ToTrayHover_Normal)
                        time.sleep(0.2)

                        self.stm32.Servo4_Ctrl(1) # 托盘上舵机，2 -> 1
                        time.sleep(0.3)
                        self.stm32.Servo3_Ctrl(0) # 托盘下舵机, 工作位置（逆180） -> 初始位置

                        gMODEL.subTaskNum = 0 # 子任务结束
                        curNum_1 = 0 # 重置当前物料计数
                        task1_state = 111 # 重置子任务状态
                        self.vision.choose_colour = color_order[0] # 重置颜色选择

                if task1_state == 3 and ZDT.Reached_Flag:
                    # 机械臂到位
                    ZDT.set_reached_flag(0) # 清除到位标志
                    print("1号电机到位!")
                    ZDT.Emm_V5_Reached_En(1, False) # 禁用电机1到位返回
                    time.sleep(0.03)
                    curNum_1 += 1 # 更新当前物料计数
                    task1_state = 0 # 抓取下一个物料



            # 机械臂朝向：Dir2
            elif gMODEL.subTaskNum == 2_0 and gMODEL.taskLock == 1: # 子任务2_0：物料放置到色环后抓回物料仓
                if task2_state == 111:
                    self.pid1.update_params(kp=8.00, ki=0.001, kd=5.00)
                    self.pid2.update_params(kp=8.00, ki=0.001, kd=5.00)
                    self.vision.Detect_Mode = 1 # 设置视觉检测模式为1, 色环
                    color_order = self.vision.read_colour_order[gMODEL.currentBatch] # 当前批次颜色顺序

                    task2_state = 0 # 进入下一状态

                if task2_state == 0:
                    self.vision.choose_colour = color_order[curNum_2_1] # 识别当前颜色

                    self.stm32.Servo3_Ctrl(2) # 托盘下舵机：初始位置 -> 工作位置（顺180）
                    time.sleep(0.01)
                    self.stm32.Servo4_Ctrl(curNum_2_1) # 托盘上舵机，0 -> 1 -> 2
                    time.sleep([0.08, 0.08, 0.2][curNum_2_1]) # 第三次舵机行程长，需要长延时

                    # 机械臂从悬浮点下移至抓取点
                    self.arm.MoveJ_AccDec(MP.Pos_TrayGrab, MP.VelAcc.ToTrayHover_Normal)
                    time.sleep(0.5) #==============================决赛待修改
                    self.stm32.Gripper_En(1)
                    time.sleep(0.2)

                    # 将物料从仓中抬起，避免撞击
                    ZDT.Emm_V5_Pos_Control(3, 0, 2000, 190, 1500, 1, 0)
                    time.sleep(0.1)
                    ZDT.Emm_V5_Reached_En(2, True) # 启用电机2到位返回
                    time.sleep(0.4)

                    self.stm32.Servo4_Ctrl(3) # 托盘上舵机: 避让位置Dir2

                    # 抓取物料后移动至目标色环上方
                    self.arm.MoveP_AccDec(circle_x[color_order[curNum_2_1]-1], 285, 75, MP.VelAcc.ToLay)
                    time.sleep(0.4)

                    self.stm32.Servo3_Ctrl(3) # 托盘下舵机: 避让位置Dir2

                    # 末端倾斜定位
                    ZDT.Emm_V5_Pos_Control(4,1,1200,150,MP.J4clk_tilted,1,0)
                    time.sleep(0.1)
                    self.stm32.Servo2_Ctrl(1) # 摄像头舵机：朝前

                    task2_state = 1 # 进入下一状态

                # 机械臂到达色环上方
                elif task2_state == 1 and ZDT.Reached_Flag:
                    ZDT.set_reached_flag(0) # 清除到位标志
                    print("2号电机到位!")
                    ZDT.Emm_V5_Reached_En(2, False) # 禁用电机2到位返回
                    time.sleep(0.03)

                    gMODEL.speedmodeEnabled = True # 启用速度模式
                    gMODEL.trackingEnabled = True # 启用跟踪模式

                    task2_state = 2 # 进入下一状态

                elif task2_state == 2:
                    # 连续二十次检测 distance <= 1.2
                    if self.vision.distance <= 1.2:
                        self.feedback_counter += 1
                    else:
                        self.feedback_counter = 0
                    # 当连续满足二十次时执行后续操作
                    if self.feedback_counter >= 20:
                        self.feedback_counter = 0 # 重置计数器

                        gMODEL.trackingEnabled = False # 关闭跟踪模式

                        # 记录当前位置，之后移动至此位置抓取
                        circle_xyz[color_order[curNum_2_1]-1][0] = self.arm.Arm_Tx
                        circle_xyz[color_order[curNum_2_1]-1][1] = self.arm.Arm_Ty

                        time.sleep(0.03)
                        ZDT.Emm_V5_Pos_Control(4,1,1800,180,MP.J4clk_normal,1,0) # 末端倾斜回正
                        time.sleep(0.3)

                        # 触发机械臂Z轴速度模式运动，移动至预设放置高度
                        self.arm.trigger_MoveZ_inSpeed(circle_xyz[color_order[curNum_2_1]-1][2], 0.4)

                        task2_state = 3 # 进入下一状态
                
                # 机械臂将物料放到色环上
                elif task2_state == 3 and self.arm.Arm_Tz <= circle_xyz[color_order[curNum_2_1]-1][2]:
                    time.sleep(0.5)
                    self.stm32.Gripper_En(0) # 夹爪打开
                    ZDT.Emm_V5_Reached_En(1, True) # 启用电机1到位返回
                    time.sleep(0.1)

                    if curNum_2_1 < 2:
                        # 移动至托盘悬停点
                        self.arm.MoveJ_AccDec(MP.Pos_TrayHover, 
                                              VA_cir2trayH_dir2[color_order[curNum_2_1]-1])
                    elif curNum_2_1 == 2: # 放置完最后一个物料
                        # 移动至第一个放置的物料上方 ==================================决赛待修改
                        self.arm.MoveP_AccDec(circle_xyz[color_order[0]-1][0],
                                              circle_xyz[color_order[0]-1][1],
                                              75, # 物料上方高度
                                              MP.VelAcc.ToCirCatch_Up)
                        
                    task2_state = 4 # 进入下一状态

                elif task2_state == 4 and ZDT.Reached_Flag:
                    # 机械臂到位
                    ZDT.set_reached_flag(0)
                    print("1号电机到位!")
                    ZDT.Emm_V5_Reached_En(1, False) # 禁用电机1到位返回
                    time.sleep(0.03)

                    if curNum_2_1 < 2:
                        curNum_2_1 += 1 # 更新当前物料计数
                        task2_state = 0 # 抓取下一个物料
                    # 到达第一个放置的物料上方
                    elif curNum_2_1 == 2:
                        task2_state = 5 # 进入下一状态
                        curNum_2_1 = 0 # 重置当前物料计数
                        self.vision.choose_colour = color_order[0] # 重置颜色选择

                elif task2_state == 5:
                    # 机械臂移动至第一个放置的物料的抓取点
                    self.arm.MoveP_AccDec(circle_xyz[color_order[0]-1][0],
                                          circle_xyz[color_order[0]-1][1],
                                          circle_xyz[color_order[0]-1][2] + circle_z_delta, # 物料抓取高度
                                          MP.VelAcc.ToCirCatch_Up)
                    time.sleep(0.4)
                    self.stm32.Gripper_En(1) # 抓取物料
                    ZDT.Emm_V5_Reached_En(1, True) # 启用电机1到位返回
                    time.sleep(0.1)

                    # 机械臂移动至托盘放置点
                    self.arm.MoveJ_AccDec(MP.Pos_TrayPlace, 
                                          VA_cir2trayP_dir2[color_order[0]-1])
                    
                    curNum_2_2 = 0 # 初始化抓回物料仓的物料计数

                    task2_state = 6 # 进入下一状态

                # 机械臂到达托盘放置点
                elif task2_state == 6 and ZDT.Reached_Flag:
                    # 机械臂到位
                    ZDT.set_reached_flag(0)
                    print("1号电机到位!")
                    self.stm32.Servo4_Ctrl(2 - curNum_2_2) # 托盘上舵机，2 -> 1 -> 0
                    time.sleep(0.05)
                    self.stm32.Servo3_Ctrl(2) # 托盘下舵机: 避让位置Dir2 -> 工作位置（顺180）
                    ZDT.Emm_V5_Reached_En(1, False) # 禁用电机1到位返回
                    time.sleep([0.8, 0.4, 0.4][curNum_2_2]) # 第一次舵机行程长，需要长延时

                    self.stm32.Gripper_En(0) # 夹爪打开, 放置成功
                    time.sleep(0.05)

                    if curNum_2_2 < 2:
                        ZDT.Emm_V5_Reached_En(2, True) # 启用电机2到位返回
                        time.sleep(0.03)

                        # 机械臂移动至第二、三个放置的物料的抓取点
                        self.arm.MoveP_AccDec(circle_xyz[color_order[curNum_2_2+1]-1][0],
                                              circle_xyz[color_order[curNum_2_2+1]-1][1],
                                              circle_xyz[color_order[curNum_2_2+1]-1][2] + circle_z_delta, # 物料抓取高度
                                              MP.VelAcc.ToCirCatch_Down)
                        time.sleep(0.2)
                        self.stm32.Servo4_Ctrl(3) # 托盘上舵机: 避让位置Dir2
                        time.sleep(0.05)
                        self.stm32.Servo3_Ctrl(3) # 托盘下舵机: 避让位置Dir2
                        
                        task2_state = 7 # 进入下一状态
                    elif curNum_2_2 == 2: # 放置完最后一个物料
                        self.stm32.Servo2_Ctrl(0) # 摄像头舵机：朝下

                        # 机械臂移动到托盘悬停点
                        self.arm.MoveJ_AccDec(MP.Pos_TrayHover, 
                                              MP.VelAcc.ToTrayHover_Normal)
                        time.sleep(0.1)

                        # 舵机回正
                        self.stm32.Servo4_Ctrl(1) # 托盘上舵机，1号位
                        time.sleep(0.1)
                        self.stm32.Servo3_Ctrl(0) # 托盘下舵机, 工作位置（逆180） -> 初始位置

                        task2_state = 111 # 重置子任务状态
                        gMODEL.subTaskNum = 0 # 子任务结束
                        gMODEL.taskLock = 0 # 解除子任务保险锁
                        gMODEL.speedmodeEnabled = False # 关闭速度模式

                
                # 机械臂到达第二、三个放置的物料的抓取点
                elif task2_state == 7 and ZDT.Reached_Flag:
                    # 机械臂到位
                    ZDT.set_reached_flag(0)
                    print("2号电机到位!")
                    self.stm32.Gripper_En(1) # 抓取物料
                    ZDT.Emm_V5_Reached_En(2, False) # 禁用电机2到位返回
                    time.sleep(0.05)
                    ZDT.Emm_V5_Reached_En(1, True) # 启用电机1到位返回
                    time.sleep(0.05)

                    # 机械臂移动至托盘放置点
                    self.arm.MoveJ_AccDec(MP.Pos_TrayPlace,
                                          VA_cir2trayP_dir2[color_order[curNum_2_2+1]-1])
                    curNum_2_2 += 1 # 更新当前物料计数

                    task2_state = 6 # 抓取下一个物料



            # 机械臂朝向：Dir3
            elif gMODEL.subTaskNum == 3_1 and gMODEL.taskLock == 1: # 子任务3_1：物料放置到色环，不抓回物料仓
                if task3_1_state == 111:
                    self.pid1.update_params(kp=8.00, ki=0.001, kd=5.00)
                    self.pid2.update_params(kp=8.00, ki=0.001, kd=5.00)
                    self.vision.Detect_Mode = 1 # 设置视觉检测模式为1, 色环
                    color_order = self.vision.read_colour_order[gMODEL.currentBatch] # 当前批次颜色顺序

                    task3_1_state = 0 # 进入下一状态

                if task3_1_state == 0:
                    self.vision.choose_colour = color_order[curNum_3_1] # 识别当前颜色

                    self.stm32.Servo3_Ctrl(2) # 托盘下舵机：初始位置 -> 工作位置（顺180）
                    time.sleep(0.2) # ********************与2_0子任务不同
                    self.stm32.Servo4_Ctrl(2 - curNum_3_1) # 托盘上舵机，2 -> 1 -> 0********************与2_0子任务不同
                    time.sleep([0.2, 0.08, 0.2][curNum_3_1]) # 第一、三次舵机行程长，需要长延时********************与2_0子任务不同

                    # 机械臂从悬浮点下移至抓取点
                    self.arm.MoveJ_AccDec(MP.Pos_TrayGrab, MP.VelAcc.ToTrayHover_Normal)
                    time.sleep(0.5) #==============================决赛待修改
                    self.stm32.Gripper_En(1)
                    time.sleep(0.2)

                    # 将物料从仓中抬起，避免撞击
                    ZDT.Emm_V5_Pos_Control(3, 0, 2000, 190, 1500, 1, 0)
                    time.sleep(0.1)
                    ZDT.Emm_V5_Reached_En(2, True) # 启用电机2到位返回
                    time.sleep(0.4)
                    self.stm32.Servo4_Ctrl(4) # 托盘上舵机: 避让位置Dir3********************与2_0子任务不同
                    # 抓取物料后移动至目标色环上方
                    self.arm.MoveP_AccDec(circle_x[color_order[curNum_3_1]-1], 285, 75, MP.VelAcc.ToLay)
                    time.sleep(0.4)
                    self.stm32.Servo3_Ctrl(4) # 托盘下舵机: 避让位置Dir3********************与2_0子任务不同
                    # 末端倾斜定位
                    ZDT.Emm_V5_Pos_Control(4,1,1200,150,MP.J4clk_tilted,1,0)
                    time.sleep(0.1)
                    self.stm32.Servo2_Ctrl(1) # 摄像头舵机：朝前

                    task3_1_state = 1 # 进入下一状态

                # 机械臂到达色环上方
                elif task3_1_state == 1 and ZDT.Reached_Flag:
                    ZDT.set_reached_flag(0) # 清除到位标志
                    print("2号电机到位!")
                    ZDT.Emm_V5_Reached_En(2, False) # 禁用电机2到位返回
                    time.sleep(0.03)
                    gMODEL.speedmodeEnabled = True # 启用速度模式
                    gMODEL.trackingEnabled = True # 启用跟踪模式

                    task3_1_state = 2 # 进入下一状态

                elif task3_1_state == 2:
                    # 连续二十次检测 distance <= 1.2
                    if self.vision.distance <= 1.2:
                        self.feedback_counter += 1
                    else:
                        self.feedback_counter = 0
                    # 当连续满足二十次时执行后续操作
                    if self.feedback_counter >= 20:
                        self.feedback_counter = 0 # 重置计数器

                        gMODEL.trackingEnabled = False # 关闭跟踪模式

                        # 无需记录当前位置********************与2_0子任务不同

                        time.sleep(0.03)
                        ZDT.Emm_V5_Pos_Control(4,1,1800,180,MP.J4clk_normal,1,0) # 末端倾斜回正
                        time.sleep(0.3)

                        # 触发机械臂Z轴速度模式运动，移动至预设放置高度
                        self.arm.trigger_MoveZ_inSpeed(circle_xyz[color_order[curNum_3_1]-1][2], 0.4)

                        task3_1_state = 3 # 进入下一状态

                # 机械臂将物料放到色环上
                elif task3_1_state == 3 and self.arm.Arm_Tz <= circle_xyz[color_order[curNum_3_1]-1][2]:
                    time.sleep(0.5)
                    self.stm32.Gripper_En(0)
                    ZDT.Emm_V5_Reached_En(1, True) # 启用电机1到位返回
                    time.sleep(0.1)

                    # 移动至托盘悬停点 ********************与2_0子任务不同
                    self.arm.MoveJ_AccDec(MP.Pos_TrayHover, 
                                          VA_cir2trayH_dir3[color_order[curNum_3_1]-1])
                    
                    task3_1_state = 4 # 进入下一状态

                elif task3_1_state == 4 and ZDT.Reached_Flag:
                    # 机械臂到位
                    ZDT.set_reached_flag(0)
                    print("1号电机到位!")
                    
                    ZDT.Emm_V5_Reached_En(1, False) # 禁用电机1到位返回
                    time.sleep(0.03)

                    if curNum_3_1 < 2:
                        curNum_3_1 += 1
                        task3_1_state = 0 # 抓取下一个物料并放置到色环
                    # 放置完最后一个物料后回到托盘悬停点
                    elif curNum_3_1 == 2: # ********************与2_0子任务不同
                        self.stm32.Servo2_Ctrl(0) # 摄像头舵机：朝下
                        time.sleep(0.05)
                        self.stm32.Servo4_Ctrl(1) # 托盘上舵机，1号位
                        time.sleep(0.1)
                        self.stm32.Servo3_Ctrl(0) # 托盘下舵机, 工作位置（顺180） -> 初始位置
                        time.sleep(0.05)

                        task3_1_state = 111 # 重置子任务状态
                        gMODEL.subTaskNum = 0 # 子任务结束
                        gMODEL.taskLock = 0 # 解除子任务保险锁
                        gMODEL.speedmodeEnabled = False # 关闭速度模式
                        curNum_3_1 = 0 # 重置当前物料计数
                        self.vision.choose_colour = color_order[0] # 重置颜色选择



            # 机械臂朝向：Dir3
            elif gMODEL.subTaskNum == 3_2 and gMODEL.taskLock == 1: # 子任务3_2：物料码垛到物料上，带精定位
                if task3_2_state == 111:
                    self.pid1.update_params(kp=8.00, ki=0.001, kd=5.00)
                    self.pid2.update_params(kp=8.00, ki=0.001, kd=5.00)
                    self.vision.Detect_Mode = 1 # 设置视觉检测模式为1, 色环
                    color_order = self.vision.read_colour_order[gMODEL.currentBatch] # 当前批次颜色顺序

                    task3_2_state = 0 # 进入下一状态

                if task3_2_state == 0:
                    self.vision.choose_colour = color_order[curNum_3_2]

                    self.stm32.Servo3_Ctrl(2) # 托盘下舵机：初始位置 -> 工作位置（顺180）
                    time.sleep(0.2)
                    self.stm32.Servo4_Ctrl(2 - curNum_3_2) # 托盘上舵机，2 -> 1 -> 0
                    time.sleep([0.2, 0.08, 0.2][curNum_3_2]) # 第一、三次舵机行程长，需要长延时

                    # 机械臂从悬浮点下移至抓取点
                    self.arm.MoveJ_AccDec(MP.Pos_TrayGrab, MP.VelAcc.ToTrayHover_Normal)
                    time.sleep(0.5) #==============================决赛待修改
                    self.stm32.Gripper_En(1)
                    time.sleep(0.2)

                    # 将物料从仓中抬起，避免撞击
                    ZDT.Emm_V5_Pos_Control(3, 0, 2000, 190, 1500, 1, 0)
                    time.sleep(0.1)
                    ZDT.Emm_V5_Reached_En(2, True) # 启用电机2到位返回
                    time.sleep(0.4)

                    self.stm32.Servo4_Ctrl(4) # 托盘上舵机: 避让位置Dir3

                    # 抓取物料后移动至目标色环上方************************************与3_1子任务不同
                    self.arm.MoveP_AccDec(circle_x[color_order[curNum_3_2]-1], 285, 140, MP.VelAcc.ToLay)
                    time.sleep(0.4)

                    self.stm32.Servo3_Ctrl(4) # 托盘下舵机: 避让位置Dir3

                    # 末端倾斜定位
                    ZDT.Emm_V5_Pos_Control(4,1,1200,150,MP.J4clk_tilted,1,0)
                    time.sleep(0.1)
                    self.stm32.Servo2_Ctrl(1) # 摄像头舵机：朝前

                    task3_2_state = 1 # 进入下一状态

                # 机械臂到达待码垛物料上方
                elif task3_2_state == 1 and ZDT.Reached_Flag:
                    ZDT.set_reached_flag(0)
                    print("2号电机到位!")
                    ZDT.Emm_V5_Reached_En(2, False) # 禁用电机2到位返回
                    time.sleep(0.03)
                    gMODEL.speedmodeEnabled = True # 启用速度模式
                    gMODEL.trackingEnabled = True # 启用跟踪模式

                    task3_2_state = 2 # 进入下一状态

                elif task3_2_state == 2:
                    # 连续二十次检测 distance <= 1.2
                    if self.vision.distance <= 1.2:
                        self.feedback_counter += 1
                    else:
                        self.feedback_counter = 0
                    # 当连续满足二十次时执行后续操作
                    if self.feedback_counter >= 20:
                        self.feedback_counter = 0 # 重置计数器

                        gMODEL.trackingEnabled = False # 关闭跟踪模式

                        # 无需记录当前位置

                        time.sleep(0.03)
                        ZDT.Emm_V5_Pos_Control(4,1,1800,180,MP.J4clk_normal,1,0) # 末端倾斜回正
                        time.sleep(0.3)

                        # 触发机械臂Z轴速度模式运动，移动至预设码垛高度********************与3_1子任务不同
                        self.arm.trigger_MoveZ_inSpeed(99.0, 0.5)

                        task3_2_state = 3 # 进入下一状态

                elif task3_2_state == 3 and self.arm.Arm_Tz <= 99.0: # ********************与3_1子任务不同
                    time.sleep(0.5)
                    self.stm32.Gripper_En(0) # 夹爪打开
                    ZDT.Emm_V5_Reached_En(1, True) # 启用电机1到位返回
                    time.sleep(0.1)

                    # 移动至托盘悬停点
                    self.arm.MoveJ_AccDec(MP.Pos_TrayHover, 
                                          VA_cir2trayH_dir3[color_order[curNum_3_2]-1])
                    
                    task3_2_state = 4 # 进入下一状态

                elif task3_2_state == 4 and ZDT.Reached_Flag:
                    # 机械臂到位
                    ZDT.set_reached_flag(0)
                    print("1号电机到位!")
                    
                    ZDT.Emm_V5_Reached_En(1, False) # 禁用电机1到位返回
                    time.sleep(0.03)

                    if curNum_3_2 < 2:
                        curNum_3_2 += 1
                        task3_2_state = 0 # 抓取下一个物料并放置到色环
                    # 放置完最后一个物料后回到托盘悬停点
                    elif curNum_3_2 == 2:
                        self.stm32.Servo2_Ctrl(0) # 摄像头舵机：朝下
                        time.sleep(0.05)
                        self.stm32.Servo4_Ctrl(1) # 托盘上舵机，1号位
                        time.sleep(0.1)
                        self.stm32.Servo3_Ctrl(0) # 托盘下舵机, 工作位置（顺180） -> 初始位置
                        time.sleep(0.05)

                        task3_2_state = 111 # 重置子任务状态
                        gMODEL.subTaskNum = 0 # 子任务结束
                        gMODEL.taskLock = 0 # 解除子任务保险锁
                        gMODEL.speedmodeEnabled = False # 关闭速度模式
                        curNum_3_2 = 0 # 重置当前物料计数
                        self.vision.choose_colour = color_order[0] # 重置颜色选择



            # 机械臂朝向：Dir3  old
            # elif gMODEL.subTaskNum == 3_3 and gMODEL.taskLock == 1: # 子任务3_3：物料码垛到物料上，不带精定位
            #     if task3_3_state == 111:
            #         color_order = self.vision.read_colour_order[gMODEL.currentBatch] # 当前批次颜色顺序

            #         task3_3_state = 0 # 进入下一状态

            #     if task3_3_state == 0:
            #         self.vision.choose_colour = color_order[curNum_3_3]

            #         self.stm32.Servo3_Ctrl(2) # 托盘下舵机：初始位置 -> 工作位置（顺180）
            #         time.sleep(0.2)
            #         self.stm32.Servo4_Ctrl(2 - curNum_3_3) # 托盘上舵机，2 -> 1 -> 0
            #         time.sleep([0.2, 0.08, 0.2][curNum_3_3]) # 第一、三次舵机行程长，需要长延时

            #         # 机械臂从悬浮点下移至抓取点
            #         self.arm.MoveJ_AccDec(MP.Pos_TrayGrab, MP.VelAcc.ToTrayHover_Normal)
            #         time.sleep(0.5) #==============================决赛待修改
            #         self.stm32.Gripper_En(1)
            #         time.sleep(0.2)
            #         # 将物料从仓中抬起，避免撞击
            #         ZDT.Emm_V5_Pos_Control(3, 0, 2000, 190, 1500, 1, 0)
            #         time.sleep(0.1)
            #         ZDT.Emm_V5_Reached_En(2, True) # 启用电机2到位返回
            #         time.sleep(0.4)
            #         self.stm32.Servo4_Ctrl(4) # 托盘上舵机: 避让位置Dir3

            #         # 抓取物料后移动至目标色环上方************************************与3_2子任务不同
            #         self.arm.MoveP_AccDec(circle_x[color_order[curNum_3_3]-1], 288, 104, MP.VelAcc.ToLay)
            #         time.sleep(0.4)

            #         self.stm32.Servo3_Ctrl(4) # 托盘下舵机: 避让位置Dir3

            #         # 无需末端倾斜定位************************************与3_2子任务不同

            #         task3_3_state = 1 # 进入下一状态

            #     # 机械臂到达待码垛物料上方
            #     elif task3_3_state == 1 and ZDT.Reached_Flag:
            #         ZDT.set_reached_flag(0)
            #         print("2号电机到位!")
            #         ZDT.Emm_V5_Reached_En(2, False)
            #         gMODEL.speedmodeEnabled = True # 启用速度模式
            #         time.sleep(0.1)

            #         # 直接码垛************************************与3_2子任务不同
            #         self.arm.trigger_MoveZ_inSpeed(103.0, 0.5) # 触发机械臂Z轴速度模式运动，移动至预设码垛高度

            #         task3_3_state = 2 # 进入下一状态

            #     elif task3_3_state == 2 and self.arm.Arm_Tz <= 103.0:
            #         time.sleep(0.5)
            #         self.stm32.Gripper_En(0) # 夹爪打开
            #         ZDT.Emm_V5_Reached_En(1, True) # 启用电机1到位返回
            #         time.sleep(0.1)

            #         # 移动至托盘悬停点
            #         self.arm.MoveJ_AccDec(MP.Pos_TrayHover, 
            #                               VA_cir2trayH_dir3[color_order[curNum_3_3]-1])
                    
            #         task3_3_state = 3 # 进入下一状态

            #     elif task3_3_state == 3 and ZDT.Reached_Flag:
            #         # 机械臂到位
            #         ZDT.set_reached_flag(0)
            #         print("1号电机到位!")
                    
            #         ZDT.Emm_V5_Reached_En(1, False)
            #         time.sleep(0.03)

            #         if curNum_3_3 < 2:
            #             curNum_3_3 += 1
            #             task3_3_state = 0
            #         # 放置完最后一个物料后回到托盘悬停点
            #         elif curNum_3_3 == 2:
            #             self.stm32.Servo2_Ctrl(0) # 摄像头舵机：朝下
            #             time.sleep(0.05)
            #             self.stm32.Servo4_Ctrl(1) # 托盘上舵机，1号位
            #             time.sleep(0.1)
            #             self.stm32.Servo3_Ctrl(0) # 托盘下舵机, 工作位置（顺180） -> 初始位置
            #             time.sleep(0.05)

            #             task3_3_state = 111 # 重置子任务状态
            #             gMODEL.subTaskNum = 0 # 子任务结束
            #             gMODEL.taskLock = 0 # 解除子任务保险锁
            #             gMODEL.speedmodeEnabled = False # 关闭速度模式
            #             curNum_3_3 = 0
            #             self.vision.choose_colour = color_order[0] # 重置颜色选择

            elif gMODEL.subTaskNum == 3_3 and gMODEL.taskLock == 1: # 子任务3_3：物料码垛到物料上，不带精定位
                if task3_3_state == 111:
                    color_order = self.vision.read_colour_order[gMODEL.currentBatch] # 当前批次颜色顺序

                    task3_3_state = 0 # 进入下一状态

                if task3_3_state == 0:
                    self.vision.choose_colour = color_order[curNum_3_3]

                    self.stm32.Servo3_Ctrl(2) # 托盘下舵机：初始位置 -> 工作位置（顺180）
                    time.sleep(0.2)
                    self.stm32.Servo4_Ctrl(2 - curNum_3_3) # 托盘上舵机，2 -> 1 -> 0
                    time.sleep([0.2, 0.08, 0.2][curNum_3_3]) # 第一、三次舵机行程长，需要长延时

                    # 机械臂从悬浮点下移至抓取点
                    self.arm.MoveJ_AccDec(MP.Pos_TrayGrab, MP.VelAcc.ToTrayHover_Normal)
                    time.sleep(0.5) #==============================决赛待修改
                    self.stm32.Gripper_En(1)
                    time.sleep(0.2)
                    # 将物料从仓中抬起，避免撞击
                    ZDT.Emm_V5_Pos_Control(3, 0, 2000, 190, 1500, 1, 0)
                    time.sleep(0.1)
                    ZDT.Emm_V5_Reached_En(2, True) # 启用电机2到位返回
                    time.sleep(0.4)
                    self.stm32.Servo4_Ctrl(4) # 托盘上舵机: 避让位置Dir3

                    # 抓取物料后移动至目标色环上方************************************与3_2子任务不同
                    self.arm.MoveP_AccDec(circle_x[color_order[curNum_3_3]-1], 288, 103, MP.VelAcc.ToLay)
                    time.sleep(0.4)

                    self.stm32.Servo3_Ctrl(4) # 托盘下舵机: 避让位置Dir3

                    # 无需末端倾斜定位************************************与3_2子任务不同

                    task3_3_state = 1 # 进入下一状态

                # 机械臂到达待码垛物料上方
                elif task3_3_state == 1 and ZDT.Reached_Flag:
                    ZDT.set_reached_flag(0)
                    print("2号电机到位!")
                    ZDT.Emm_V5_Reached_En(2, False)
                    time.sleep(0.2)
                    self.stm32.Gripper_En(0) # 夹爪打开
                    time.sleep(0.1)
                    ZDT.Emm_V5_Reached_En(1, True) # 启用电机1到位返回
                    time.sleep(0.1)

                    # 移动至托盘悬停点
                    self.arm.MoveJ_AccDec(MP.Pos_TrayHover, 
                                          VA_cir2trayH_dir3[color_order[curNum_3_3]-1])

                    task3_3_state = 2 # 进入下一状态

                elif task3_3_state == 2 and ZDT.Reached_Flag:
                    # 机械臂到位
                    ZDT.set_reached_flag(0)
                    print("1号电机到位!")
                    
                    ZDT.Emm_V5_Reached_En(1, False)
                    time.sleep(0.03)

                    if curNum_3_3 < 2:
                        curNum_3_3 += 1
                        task3_3_state = 0
                    # 放置完最后一个物料后回到托盘悬停点
                    elif curNum_3_3 == 2:
                        self.stm32.Servo2_Ctrl(0) # 摄像头舵机：朝下
                        time.sleep(0.05)
                        self.stm32.Servo4_Ctrl(1) # 托盘上舵机，1号位
                        time.sleep(0.1)
                        self.stm32.Servo3_Ctrl(0) # 托盘下舵机, 工作位置（顺180） -> 初始位置
                        time.sleep(0.05)

                        task3_3_state = 111 # 重置子任务状态
                        gMODEL.subTaskNum = 0 # 子任务结束
                        gMODEL.taskLock = 0 # 解除子任务保险锁
                        gMODEL.speedmodeEnabled = False # 关闭速度模式
                        curNum_3_3 = 0
                        self.vision.choose_colour = color_order[0] # 重置颜色选择











    def Message_Update_Task(self):
        """消息更新任务"""
        gMODEL.alpha0.set(self.arm.Alpha0)
        
        # self.test += 1
        # print(self.test)

    def PID_Task(self):
        """PID控制任务"""

        if gMODEL.trackingEnabled:
            # 每启动跟踪时只执行一次
            if self.PID_Task_target_update_flag:
                self.PID_Task_set_speed["total"] = 0.0
                self.PID_Task_set_speed["x"] = 0.0
                self.PID_Task_set_speed["y"] = 0.0
                self.PID_Task_speed_clear_flag = True
                self.PID_Task_target_update_flag = False

            self.relX = self.vision.relative_x
            self.relY = self.vision.relative_y
            self.feedback_distance = self.vision.distance

            # 分段PID
            if self.feedback_distance < 20.0:
                self.PID_Task_set_speed["total"] = self.PRE.pid2.compute(0, self.feedback_distance)
            else:
                self.PID_Task_set_speed["total"] = self.PRE.pid1.compute(0, self.feedback_distance)

            if self.feedback_distance:
                self.PID_Task_set_speed["x"] = self.PID_Task_set_speed["total"] * self.relX / self.feedback_distance
                self.PID_Task_set_speed["y"] = self.PID_Task_set_speed["total"] * self.relY / self.feedback_distance

            if not (self.feedback_distance <= 1.5):
                self.PID_Task_reached_flag = False
                # print(f"pid_VxVy: {self.PID_Task.set_speed['x']}, {self.PID_Task.set_speed['y']}")
                self.arm.trigger_MoveL_Speed(self.PID_Task_set_speed["x"], self.PID_Task_set_speed["y"]) # 触发直线插补运动，速度模式
            else:
                if not self.PID_Task_reached_flag:
                    self.PID_Task_reached_flag = True
                    self.arm.trigger_MoveL_Speed(0,0) # 到达目标位置，停止运动
                    print("Reached!")
                    self.PRE.add_log("目标到达！")

        elif not gMODEL.trackingEnabled:
            # 重置状态以便重新更新目标
            self.PID_Task_target_update_flag = True
            if self.PID_Task_speed_clear_flag:
                self.arm.trigger_MoveL_Speed(0, 0)  # 停止运动
                self.pid1.reset()  # 重置PID控制器
                self.pid2.reset()
                self.PID_Task_speed_clear_flag = False

    def start_background_tasks(self):
        """启动后台任务线程"""
        if self.running_bg_tasks:
            print("后台任务线程已在运行")
            return
        
        self.running_bg_tasks = True
        self.exit_flag_bg_tasks = False
        self.thread_bg_tasks = threading.Thread(target=self.run_bg_tasks, daemon=True)
        self.thread_bg_tasks.start()
        # print("测试")

    def start_main_tasks(self):
        """启动主任务线程"""
        if self.running_main_task:
            print("主任务线程已在运行")
            return

        self.running_main_task = True
        self.exit_flag_main_task = False
        self.thread_main_task = threading.Thread(target=self.run_main_task, daemon=True)
        self.thread_main_task.start()

    def stop_background_tasks(self):
        """停止后台任务线程"""
        if not self.running_bg_tasks:
            print("后台任务线程未运行")
            return

        self.running_bg_tasks = False
        self.exit_flag_bg_tasks = True
        # 只有不是当前线程时才 join
        if self.thread_bg_tasks and self.thread_bg_tasks.is_alive() and threading.current_thread() != self.thread_bg_tasks:
            self.thread_bg_tasks.join(timeout=1.0)

    def stop_main_tasks(self):
        """停止主任务线程"""
        if not self.running_main_task:
            print("主任务线程未运行")
            return

        self.running_main_task = False
        self.exit_flag_main_task = True
        # 只有不是当前线程时才 join
        if self.thread_main_task and self.thread_main_task.is_alive() and threading.current_thread() != self.thread_main_task:
            self.thread_main_task.join(timeout=1.0)


    