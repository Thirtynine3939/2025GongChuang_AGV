import math
import ZDT
import time
from model import GLOBAL_MODEL as gMODEL
import threading

class MotionParams:
    Pos_TrayPlace = [31800, 299, 584] # 放置到托盘的位置        等效全局点位：放置点: x=-118.3, y=-123.7, z=219.1
    Pos_TrayHover = [31800, 4, 1500] # 夹取前夹爪悬停的位置     等效全局点位：悬停点: x=-106.1, y=-110.9, z=249.7
    Pos_TrayGrab = [31800, 4, 110] # 从托盘夹取的位置           等效全局点位：夹取点: x=-116.8, y=-122.1, z=203.8

    J4clk_normal = 3579
    J4clk_tilted = 4779

    # VelAccParams = {
    #     "INIT": [[4000,150], [800,80], [800,80]],
    #     "INIT_ChangeDir": [[6000,180], [800,80], [800,80]],
    #     "DEFAULT": [[1200,100], [800,80], [800,80]],
    #     "ToSave1": [[6400,220], [1000,180], [1800,150]],
    #     "ToSave1_1": [[6400,220], [1000,100], [1800,150]],
    #     "ToCatch": [[6400,234], [800,80], [1600,200]],
    #     "ToPlace": [[6400,200], [800,80], [800,80]],
    #     "ToLay": [[6400,225], [1000,100], [1600,200]],
    #     "ToSave2": [[6400,200], [1800,190], [1800,150]],
    #     "ArmFix_ToS2": [[6400,220], [1900,190], [1000,30]],
    #     "ToSave2_1": [[6400,160], [1800,190], [1600,150]],
    #     "ToSave2_2": [[6400,210], [1800,190], [1600,150]],
    #     "Lay1_ToS": [[6400,120], [1800,190], [1600,4]],
    #     "Lay2_ToS": [[6400,210], [1800,190], [1600,4]],
    #     "Lay3_ToS": [[6400,210], [1800,190], [1600,4]],
    #     "ToCirCatch1": [[6400,230], [1200,150], [1600,230]],
    #     "ToCirCatch2": [[6400,215], [1800,215], [1800,215]]
    # }

    class VelAcc:
        ZERO = [[4000, 150], [800, 80], [800, 80]]
        ZERO_ChangeDir = [[6000, 180], [800, 80], [800, 80]]
        DEFAULT = [[1200, 100], [800, 80], [800, 80]]
        ToTrackBegin = [[6400, 234], [800, 80], [1600, 200]]
        ToTrayHover_Normal = [[6400, 200], [1800, 190], [1800, 190]]
        ToTrayHover_Slow = [[6400, 160], [1800, 190], [1600, 150]]
        ToTrayHover_Fast = [[6400, 210], [1800, 190], [1600, 150]]
        ArmFix_ToTrayHover = [[6400, 220], [1900, 190], [1000, 30]]
        Dir1toTrayPlace_Slow = [[6400, 220], [1000, 100], [1800, 150]]
        Dir1toTrayPlace_Fast = [[6400, 200], [1800, 180], [1800, 180]]
        Dir23toTrayPlace_Slow = [[6400, 120], [1800, 190], [1600, 4]]
        Dir23toTrayPlace_Fast = [[6400, 210], [1800, 190], [1600, 4]]
        ToTrayGrab = [[6400, 200], [1800, 190], [1800, 150]]
        ToLay = [[6400,225], [1000,100], [1600,200]]
        ToCirCatch_Up = [[6400, 215], [1800, 215], [1800, 215]]
        ToCirCatch_Down = [[6400, 230], [1200, 150], [1600, 230]]
        AUTO = [[0, 0], [0, 0], [0, 0]]  

    VelAccParams = {
        "ZERO": [[4000,150], [800,80], [800,80]],
        "ZERO_ChangeDir": [[6000,180], [800,80], [800,80]],
        "DEFAULT": [[1200,100], [800,80], [800,80]],
        "ToTrackBegin": [[6400,234], [800,80], [1600,200]],
        "ToTrayHover_Normal": [[6400,200], [1800,190], [1800,150]],
        "ToTrayHover_Slow": [[6400,160], [1800,190], [1600,150]],
        "ToTrayHover_Fast": [[6400,210], [1800,190], [1600,150]],
        "ArmFix_ToTrayHover": [[6400,220], [1900,190], [1000,30]],
        "Dir1toTrayPlace_Slow": [[6400,220], [1000,100], [1800,150]],
        "Dir1toTrayPlace_Fast": [[6400,220], [1000,180], [1800,150]],
        "Dir23toTrayPlace_Slow": [[6400,120], [1800,190], [1600,4]],
        "Dir23toTrayPlace_Fast": [[6400,210], [1800,190], [1600,4]],
        "ToTrayGrab": [[6400,200], [1800,190], [1800,150]],
        "ToCirCatch_Up": [[6400,215], [1800,215], [1800,215]],
        "ToCirCatch_Down": [[6400,230], [1200,150], [1600,230]],
    }

class RoboArm:
    def __init__(self):
        # 机械臂参数
        self.OA = 150.1
        self.AB = 150.0
        self.BC = 240.0
        self.CD1 = 85.0
        self.pi = 3.141592654
        self.Step_Size = 0.5

        # 机械臂状态
        self.last_motor1, self.last_motor2, self.last_motor3 = 0, 0, 0
        self.Arm_Tx, self.Arm_Ty, self.Arm_Tz = 0, 175, 199
        self.new_target_available = False
        self.new_speed_available = False
        self.Alpha = 0 # 底座转角（弧度）
        self.Alpha0 = 0 # 底座转角（度）
        self.armDir = 1  # 机械臂方向（按照初赛任务顺序1、2、3、4）

        # 机械臂直线插补参数
        self.new_target = [0, 175, 199]
        self.new_speed = [0, 0]
        self.new_stepsize = 0.08

        # 插补线程状态标志
        self.exit_flag = False
        self.running = False
        self.thread = None

    def start(self): # 启动插补线程
        """启动插补线程"""
        if self.running:
            print("插补线程已在运行")
            return
        
        self.running = True
        self.exit_flag = False
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def stop(self): # 停止插补线程
        """停止插补线程"""
        if not self.running:
            print("插补线程未运行")
            return
        
        self.running = False
        self.exit_flag = True
        # 只有不是当前线程时才 join
        if self.thread and self.thread.is_alive() and threading.current_thread() != self.thread:
            self.thread.join(timeout=1.0)

    def run(self): # 插补线程运行函数
        """插补线程运行函数"""
        while not self.exit_flag:
            time.sleep(0.001)
            if not gMODEL.speedmodeEnabled: # 位置模式
                if self.new_target_available:
                    self.new_target_available = False
                    # 进行轨迹规划并运动
                    target_X, target_Y, target_Z = self.new_target
                    self.Step_Size = self.new_stepsize
                    self.MoveL(target_X, target_Y, target_Z) # 直线插补运动
            elif gMODEL.speedmodeEnabled:
                if self.new_speed_available:
                    self.new_speed_available = False
                    target_Vx, target_Vy = self.new_speed
                    target_Z = self.new_target[2]
                    z_step_size = self.new_stepsize
                    self.MoveL_Speed(target_Vx, target_Vy, target_Z, z_step_size) #直线插补运动_速度

    def trigger_MoveL(self, x, y, z): # 触发直线插补运动_位置
        """触发直线插补运动，位置模式"""
        self.new_target = [x, y, z]
        self.new_target_available = True

    def trigger_MoveL_Speed(self, Vx, Vy): # 触发直线插补运动_速度
        """触发直线插补运动，速度模式"""
        self.new_speed = [Vx, Vy]
        self.new_speed_available = True
        gMODEL.speedX.set(self.new_speed[0])
        gMODEL.speedY.set(self.new_speed[1])

    def trigger_MoveZ_inSpeed(self, z, stepsize=None): # 触发速度模式中的Z轴运动
        """触发速度模式中的Z轴运动"""
        self.new_target[2] = z
        if stepsize is not None:
            self.new_stepsize = stepsize
        self.new_speed_available = True

    def trigger_Update_StepSize(self, stepsize, mode: str):
        """触发直线插补的步长更新"""
        self.new_stepsize = stepsize
        if mode == "POSITION":
            self.new_target_available = True
        elif mode == "SPEED":
            self.new_speed_available = True

    def set_armDir(self, dir):
        """设置机械臂方向 (1, 2, 3 或 4)"""
        if dir in (1, 2, 3, 4):
            self.armDir = dir
        else:
            raise ValueError("Invalid armDir value. Must be 1, 2, 3 or 4.")
        
    def set_armDir_ui(self, dir):
        """设置机械臂方向 (1, 2, 3 或 4), 并且更新UI"""
        if dir in (1, 2, 3, 4):
            self.armDir = dir
            gMODEL.armDirection.set(dir)
        else:
            raise ValueError("Invalid armDir value. Must be 1, 2, 3 or 4.")
        
    def Update_StepSize(self, size):
        """设置直线插补的步长"""
        self.Step_Size = size

    def pos_cal_forward(self, M1, M2, M3): # 机械臂正解算
        """
            计算给定电机角度对应的空间坐标
            M1: 电机1角度
            M2: 电机2角度
            M3: 电机3角度

            Alpha: 底座转角（俯视顺时针为正）
            Gamma: ∠ABC
            B1: ∠CAB
            B2: ∠CAO
            Beta: ∠OAB = B1 + B2
        """

        # 方向调整
        M1 = -M1
        M3 = -M3

        # 计算Beta
        Beta0  = M2 * 360 / 3200 / 4.5
        Beta = Beta0 * self.pi / 180.0
        Beta = 1.5 * self.pi - Beta  # 调整Beta角度

        # 计算Gamma
        Gamma0 = 12.02 - (M3 * 360 / 3200 - 4.5 * Beta0) / 13.5
        Gamma = Gamma0 * self.pi / 180.0

        # 计算Alpha
        Alpha0 = M1 * 360 / 3200 / 26.25
        Alpha = Alpha0 * self.pi / 180.0

        # 计算空间坐标
        Tz = self.OA - self.AB * math.cos(Beta) - self.BC * math.cos(Beta + Gamma - self.pi)
        Tx = (self.AB*math.sin(Beta) + self.BC*math.sin(Beta + Gamma - self.pi) + self.CD1) * math.sin(Alpha)
        Ty = (self.AB*math.sin(Beta) + self.BC*math.sin(Beta + Gamma - self.pi) + self.CD1) * math.cos(Alpha)

        # 保留一位小数
        Tx = round(Tx, 1)
        Ty = round(Ty, 1)
        Tz = round(Tz, 1)

        return Tx, Ty, Tz

    def pos_cal(self, Tx, Ty, Tz): # 局部逆解算
        """
            计算给定坐标对应的电机角度
            Alpha: 底座转角（俯视顺时针为正）
            Gamma: ∠ABC
            B1: ∠CAB
            B2: ∠CAO
        """
        self.Alpha = math.atan(Tx / Ty)  # 计算Alpha角
        AC = math.sqrt((Tz - self.OA)**2 + (Tx - self.CD1*math.sin(self.Alpha))**2 + (Ty - self.CD1*math.cos(self.Alpha))**2)
        
        cos_G = (self.AB**2 + self.BC**2 - AC**2) / (2 * self.AB * self.BC)
        cos_B1 = (self.AB**2 + AC**2 - self.BC**2) / (2 * self.AB * AC)
        
        if -1 <= cos_G <= 1:
            Gamma = math.acos(cos_G)  # 计算Gamma角
        elif cos_G < -1:
            Gamma = self.pi
        elif cos_G > 1:
            Gamma = 12.02 * self.pi / 180.0

        if -1 <= cos_B1 <= 1:
            B1 = math.acos(cos_B1)
        elif cos_B1 > 1:
            B1 = 0
        elif cos_B1 < -1:
            B1 = self.pi 

        if Tz == self.OA:
            B2 = self.pi / 2
        elif Tz > self.OA:
            B2 = self.pi / 2 + math.atan((Tz - self.OA) / (math.sqrt(Tx**2 + Ty**2) - self.CD1))
        else:
            B2 = math.atan((math.sqrt(Tx**2 + Ty**2) - self.CD1) / (self.OA - Tz))
        
        Beta = 1.5 * self.pi - (B1 + B2)
        self.Alpha0 = self.Alpha * 180.0 / self.pi
        Gamma0 = Gamma * 180.0 / self.pi
        
        if Gamma0 < 12.02:
            Gamma0 = 12.02
        
        Beta0 = Beta * 180.0 / self.pi
        
        # 计算电机目标值
        if self.armDir == 1:
            Motor1 = round(26.25 * self.Alpha0 * 3200 / 360)
        elif self.armDir == 2:
            Motor1 = round(26.25 * self.Alpha0 * 3200 / 360) - 42000
        elif self.armDir == 3:
            Motor1 = round(26.25 * self.Alpha0 * 3200 / 360) - 21000
        
        Motor2 = round(4.5 * Beta0 * 3200 / 360)
        Motor3 = round((13.5 * (12.02 - Gamma0) + 4.5 * Beta0) * 3200 / 360)
        
        return Motor1, Motor2, Motor3
    
    def my_atan2(self, y, x):
        """自定义 atan2 函数"""
        return math.atan2(x, y)
    
    def pos_cal_global(self, Tx, Ty, Tz): # 全局逆解算，基于机械臂方向1
        """
            计算给定坐标对应的电机角度
            Alpha: 底座转角（俯视顺时针为正）
            Gamma: ∠ABC
            B1: ∠CAB
            B2: ∠CAO
        """
        self.Alpha = self.my_atan2(Ty, Tx)  # 计算Alpha角
        AC = math.sqrt((Tz - self.OA)**2 + (Tx - self.CD1*math.sin(self.Alpha))**2 + (Ty - self.CD1*math.cos(self.Alpha))**2)
        
        cos_G = (self.AB**2 + self.BC**2 - AC**2) / (2 * self.AB * self.BC)
        cos_B1 = (self.AB**2 + AC**2 - self.BC**2) / (2 * self.AB * AC)
        
        if -1 <= cos_G <= 1:
            Gamma = math.acos(cos_G)  # 计算Gamma角
        elif cos_G < -1:
            Gamma = self.pi
        elif cos_G > 1:
            Gamma = 12.02 * self.pi / 180.0

        if -1 <= cos_B1 <= 1:
            B1 = math.acos(cos_B1)
        elif cos_B1 > 1:
            B1 = 0
        elif cos_B1 < -1:
            B1 = self.pi 

        if Tz == self.OA:
            B2 = self.pi / 2
        elif Tz > self.OA:
            B2 = self.pi / 2 + math.atan((Tz - self.OA) / (math.sqrt(Tx**2 + Ty**2) - self.CD1))
        else:
            B2 = math.atan((math.sqrt(Tx**2 + Ty**2) - self.CD1) / (self.OA - Tz))
        
        Beta = 1.5 * self.pi - (B1 + B2)
        self.Alpha0 = self.Alpha * 180.0 / self.pi
        Gamma0 = Gamma * 180.0 / self.pi
        
        if Gamma0 < 12.02:
            Gamma0 = 12.02
        
        Beta0 = Beta * 180.0 / self.pi
        
        # 计算电机目标值
        Motor1 = round(26.25 * self.Alpha0 * 3200 / 360)
        
        Motor2 = round(4.5 * Beta0 * 3200 / 360)
        Motor3 = round((13.5 * (12.02 - Gamma0) + 4.5 * Beta0) * 3200 / 360)
        
        return Motor1, Motor2, Motor3
    
    def MoveJ_AccDec(self, M123: list, velacc: list):
        """带加减速的关节运动控制"""
        if M123[0] < 0:
            ZDT.Emm_V5_Pos_Control(1, 1, velacc[0][0], velacc[0][1], -M123[0], 1, 1)
        else:
            ZDT.Emm_V5_Pos_Control(1, 0, velacc[0][0], velacc[0][1], M123[0], 1, 1)
        time.sleep(0.003)

        if M123[1] < 0:
            print("无效位置:Motor2")
        else:
            ZDT.Emm_V5_Pos_Control(2, 0, velacc[1][0], velacc[1][1], M123[1], 1, 1)
        time.sleep(0.003)

        if M123[2] < 0:
            ZDT.Emm_V5_Pos_Control(3, 1, velacc[2][0], velacc[2][1], -M123[2], 1, 1)
        else:
            ZDT.Emm_V5_Pos_Control(3, 0, velacc[2][0], velacc[2][1], M123[2], 1, 1)
        time.sleep(0.003)

        # 同步电机
        ZDT.Emm_V5_Synchronous_motion(0)

    def MoveP_AccDec(self, Tx, Ty, Tz, velacc: list):
        """带加减速的空间坐标控制, 自更新目标角度new_target"""
        move_time = None # 运动时间

        self.Arm_Tx, self.Arm_Ty, self.Arm_Tz = Tx, Ty, Tz

        #####################################更新全局模型，保持与UI同步
        gMODEL.xPos.set(self.Arm_Tx)
        gMODEL.yPos.set(self.Arm_Ty)
        gMODEL.zPos.set(self.Arm_Tz)
        #####################################

        # 获取当前目标角度
        Motor1, Motor2, Motor3 = self.pos_cal(Tx, Ty, Tz)

        print(f"Last_Motor: {self.last_motor1}, {self.last_motor2}, {self.last_motor3}")
        print(f'Motor1: {Motor1}, Motor2: {Motor2}, Motor3: {Motor3}')
        print(f"Arm_Txyz: {self.Arm_Tx}, {self.Arm_Ty}, {self.Arm_Tz}")

        # 计算每个电机的角度脉冲差
        diff_motor1 = abs(Motor1 - self.last_motor1)
        diff_motor2 = abs(Motor2 - self.last_motor2)
        diff_motor3 = abs(Motor3 - self.last_motor3)

        # 计算每个电机的运动时间
        move_times = []
        if diff_motor1 > 0:
            time1 = self.move_time_cal(clk=diff_motor1, vel=velacc[0][0], acc=velacc[0][1])
            move_times.append(time1)
            print(f"Motor1运动时间: {time1:.3f}秒")
        if diff_motor2 > 0:
            time2 = self.move_time_cal(clk=diff_motor2, vel=velacc[1][0], acc=velacc[1][1])
            move_times.append(time2)
            print(f"Motor2运动时间: {time2:.3f}秒")
        if diff_motor3 > 0:
            time3 = self.move_time_cal(clk=diff_motor3, vel=velacc[2][0], acc=velacc[2][1])
            move_times.append(time3)
            print(f"Motor3运动时间: {time3:.3f}秒")
        
        # 取最大运动时间
        if move_times:
            move_time = max(move_times)
            print(f"MoveP_AccDec: 最大运动时间: {move_time:.3f}秒")

        # 控制电机运动
        if Motor1 < 0:
            ZDT.Emm_V5_Pos_Control(1, 0, velacc[0][0], velacc[0][1], -Motor1, 1, 1)
        else:
            ZDT.Emm_V5_Pos_Control(1, 1, velacc[0][0], velacc[0][1], Motor1, 1, 1)
        time.sleep(0.003)

        if Motor2 < 0:
            print("无效位置:Motor2")
        else:
            ZDT.Emm_V5_Pos_Control(2, 0, velacc[1][0], velacc[1][1], Motor2, 1, 1)
        time.sleep(0.003)

        if Motor3 < 0:
            ZDT.Emm_V5_Pos_Control(3, 0, velacc[2][0], velacc[2][1], -Motor3, 1, 1)
        else:
            ZDT.Emm_V5_Pos_Control(3, 1, velacc[2][0], velacc[2][1], Motor3, 1, 1)
        time.sleep(0.003)

        # 同步电机
        ZDT.Emm_V5_Synchronous_motion(0)

        # 更新上次的角度值
        self.last_motor1, self.last_motor2, self.last_motor3 = Motor1, Motor2, Motor3

        # 更新目标角度
        self.new_target = [Tx, Ty, Tz]

        # 返回运动时间
        if move_time is not None:
            return move_time

    def move_time_cal(self, clk, vel, acc):
        """计算运动时间"""
        # 计算真加速度a 单位：r/s^2
        a = 1000 / (3 * (256 - acc))

        # 计算最大速度vmax 单位：r/s
        vmax = vel / 60

        # 计算总位移theta 单位：r
        theta = clk / 3200

        # 计算加速段和减速段的总距离Sad 单位：r
        Sad = (vmax**2) / a

        # 计算运动时间t 单位：s
        if theta < Sad: # 电机达不到最大速度，运动仅包括加速段和减速段（无匀速段）
            t = 2 * math.sqrt(theta / a)
        elif theta >= Sad: # 电机达到最大速度，运动包括加速段、匀速段和减速段
            t = vmax / a + theta / vmax
        else:
            print("Error in move_time_cal: Invalid parameters")
            return None
        
        return t


    def MoveP_AccDec_global(self, Txyz: list, velacc: list, single_velacc: list = None): # 全局版
        """带加减速的空间坐标控制, 全局逆解算"""
        move_time = None # 运动时间

        Tx, Ty, Tz = Txyz

        self.Arm_Tx, self.Arm_Ty, self.Arm_Tz = Tx, Ty, Tz

        #####################################更新全局模型，保持与UI同步
        gMODEL.xPos.set(self.Arm_Tx)
        gMODEL.yPos.set(self.Arm_Ty)
        gMODEL.zPos.set(self.Arm_Tz)
        #####################################

        # 获取当前目标角度
        Motor1, Motor2, Motor3 = self.pos_cal_global(Tx, Ty, Tz)

        print(f"Last_Motor: {self.last_motor1}, {self.last_motor2}, {self.last_motor3}")
        print(f'Motor1: {Motor1}, Motor2: {Motor2}, Motor3: {Motor3}')
        print(f"Arm_Txyz: {self.Arm_Tx}, {self.Arm_Ty}, {self.Arm_Tz}")

        if velacc[0][0] == 0 and single_velacc is not None: # 同步运动模式
            motorNum = single_velacc[0]  # 电机编号
            vel = single_velacc[1]  # 速度
            acc = single_velacc[2]  # 加速度

            velacc_sync = [[0, 0], [0, 0], [0, 0]]

            # 计算每个电机的角度脉冲差
            diff_motor1 = abs(Motor1 - self.last_motor1)
            diff_motor2 = abs(Motor2 - self.last_motor2)
            diff_motor3 = abs(Motor3 - self.last_motor3)

            if [diff_motor1, diff_motor2, diff_motor3][motorNum - 1] != 0:
                if motorNum == 1:
                    velacc_sync[0] = [vel, acc] # Motor1的速度和加速度
                    velacc_sync[1] = [round(vel * diff_motor2 / diff_motor1), 
                                      0 if diff_motor2 == 0 else round(256 - (256 - acc) * diff_motor1 / diff_motor2)] # Motor2的速度和加速度
                    velacc_sync[2] = [round(vel * diff_motor3 / diff_motor1), 
                                      0 if diff_motor3 == 0 else round(256 - (256 - acc) * diff_motor1 / diff_motor3)] # Motor3的速度和加速度
                elif motorNum == 2:
                    velacc_sync[1] = [vel, acc] # Motor2的速度和加速度
                    velacc_sync[0] = [round(vel * diff_motor1 / diff_motor2),
                                      0 if diff_motor1 == 0 else round(256 - (256 - acc) * diff_motor2 / diff_motor1)] # Motor1的速度和加速度
                    velacc_sync[2] = [round(vel * diff_motor3 / diff_motor2),
                                      0 if diff_motor3 == 0 else round(256 - (256 - acc) * diff_motor2 / diff_motor3)] # Motor3的速度和加速度
                elif motorNum == 3:
                    velacc_sync[2] = [vel, acc] # Motor3的速度和加速度
                    velacc_sync[0] = [round(vel * diff_motor1 / diff_motor3),
                                      0 if diff_motor1 == 0 else round(256 - (256 - acc) * diff_motor3 / diff_motor1)] # Motor1的速度和加速度
                    velacc_sync[1] = [round(vel * diff_motor2 / diff_motor3),
                                      0 if diff_motor2 == 0 else round(256 - (256 - acc) * diff_motor3 / diff_motor2)] # Motor2的速度和加速度
                    
                for i in range(3):
                    if velacc_sync[i][1] <= 0:
                        velacc_sync[i][1] = 1 # 限制加速度最小值为1
                    elif velacc_sync[i][1] > 250:
                        velacc_sync[i][1] = 250 # 限制加速度最大值为250
                    
                print(f"diff_motor: {diff_motor1}, {diff_motor2}, {diff_motor3}")

                print(f"velacc_sync: {velacc_sync}")

                if velacc_sync[0][0] != 0: # speed1不低于较小值时
                    if Motor1 < 0:
                        ZDT.Emm_V5_Pos_Control(1, 0, velacc_sync[0][0], velacc_sync[0][1], -Motor1, 1, 1)
                    else:
                        ZDT.Emm_V5_Pos_Control(1, 1, velacc_sync[0][0], velacc_sync[0][1], Motor1, 1, 1)
                time.sleep(0.003)

                if velacc_sync[1][0] != 0: # speed2不为0时
                    if Motor2 < 0:
                        print("无效位置:Motor2")
                    else:
                        ZDT.Emm_V5_Pos_Control(2, 0, velacc_sync[1][0], velacc_sync[1][1], Motor2, 1, 1)
                time.sleep(0.003)

                if velacc_sync[2][0] != 0: # speed3不为0时
                    if Motor3 < 0:
                        ZDT.Emm_V5_Pos_Control(3, 0, velacc_sync[2][0], velacc_sync[2][1], -Motor3, 1, 1)
                    else:
                        ZDT.Emm_V5_Pos_Control(3, 1, velacc_sync[2][0], velacc_sync[2][1], Motor3, 1, 1)
                time.sleep(0.003)

                # 同步电机
                ZDT.Emm_V5_Synchronous_motion(0)

                move_time = self.move_time_cal(clk=[diff_motor1, diff_motor2, diff_motor3][motorNum - 1],
                                               vel=vel,
                                               acc=acc)
                print(f"MoveP_AccDec_global: 电机编号{motorNum}运动时间: {move_time:.3f}秒")

            else: # 角度差为0时
                print("MoveP_AccDec_global: 电机编号指定错误")
                pass

        else: # 普通运动模式
            
            # 计算每个电机的角度脉冲差
            diff_motor1 = abs(Motor1 - self.last_motor1)
            diff_motor2 = abs(Motor2 - self.last_motor2)
            diff_motor3 = abs(Motor3 - self.last_motor3)
            
            # 计算每个电机的运动时间
            move_times = []
            if diff_motor1 > 0:
                time1 = self.move_time_cal(clk=diff_motor1, vel=velacc[0][0], acc=velacc[0][1])
                move_times.append(time1)
                print(f"Motor1运动时间: {time1:.3f}秒")
            if diff_motor2 > 0:
                time2 = self.move_time_cal(clk=diff_motor2, vel=velacc[1][0], acc=velacc[1][1])
                move_times.append(time2)
                print(f"Motor2运动时间: {time2:.3f}秒")
            if diff_motor3 > 0:
                time3 = self.move_time_cal(clk=diff_motor3, vel=velacc[2][0], acc=velacc[2][1])
                move_times.append(time3)
                print(f"Motor3运动时间: {time3:.3f}秒")
            
            # 取最大运动时间
            if move_times:
                move_time = max(move_times)
                print(f"MoveP_AccDec_global: 最大运动时间: {move_time:.3f}秒")

            # 控制电机运动
            if Motor1 < 0:
                ZDT.Emm_V5_Pos_Control(1, 0, velacc[0][0], velacc[0][1], -Motor1, 1, 1)
            else:
                ZDT.Emm_V5_Pos_Control(1, 1, velacc[0][0], velacc[0][1], Motor1, 1, 1)
            time.sleep(0.003)

            if Motor2 < 0:
                print("无效位置:Motor2")
            else:
                ZDT.Emm_V5_Pos_Control(2, 0, velacc[1][0], velacc[1][1], Motor2, 1, 1)
            time.sleep(0.003)

            if Motor3 < 0:
                ZDT.Emm_V5_Pos_Control(3, 0, velacc[2][0], velacc[2][1], -Motor3, 1, 1)
            else:
                ZDT.Emm_V5_Pos_Control(3, 1, velacc[2][0], velacc[2][1], Motor3, 1, 1)
            time.sleep(0.003)

            # 同步电机
            ZDT.Emm_V5_Synchronous_motion(0)

        # 更新上次的角度值
        self.last_motor1, self.last_motor2, self.last_motor3 = Motor1, Motor2, Motor3

        # 更新目标角度
        self.new_target = [Tx, Ty, Tz]

        # 返回运动时间
        if move_time is not None:
            return move_time

    def Move(self, Tx, Ty, Tz):  # 速度自适应移动，用于直线插补
        """速度自适应移动，用于直线插补"""
        # 获取当前目标角度
        Motor1, Motor2, Motor3 = self.pos_cal(Tx, Ty, Tz)

        # 计算每个电机的角度差
        diff_motor1 = abs(Motor1 - self.last_motor1)
        diff_motor2 = abs(Motor2 - self.last_motor2)
        diff_motor3 = abs(Motor3 - self.last_motor3)

        # 根据角度差决定各关节速度
        speed1 = round(((diff_motor1/(0.002*4)) * 600 / 3200) * 1.1) 
        speed2 = round(((diff_motor2/(0.002*4)) * 600 / 3200) * 1.1) 
        speed3 = round(((diff_motor3/(0.002*4)) * 600 / 3200) * 1.1) 

        # print(f"Last_Motor: {self.last_motor1}, {self.last_motor2}, {self.last_motor3}")
        # print(f'Motor1: {Motor1}, Motor2: {Motor2}, Motor3: {Motor3}')
        # print(f'speed1: {speed1}, speed2: {speed2}, speed3: {speed3}')

        # 限制速度最大值
        if speed1 > 8000:
            speed1 = 8000
        if speed2 > 8000:
            speed2 = 8000
        if speed3 > 8000:
            speed3 = 8000

        # 控制电机运动
        if speed1 != 0:
            if Motor1 < 0:
                ZDT.Emm_V5_Pos_Control(1, 0, speed1, 0, -Motor1, 1, 1)
            else:
                ZDT.Emm_V5_Pos_Control(1, 1, speed1, 0, Motor1, 1, 1)
        time.sleep(0.002)

        if speed2 != 0:
            if Motor2 < 0:
                print("无效位置:Motor2")
            else:
                ZDT.Emm_V5_Pos_Control(2, 0, speed2, 0, Motor2, 1, 1)
        time.sleep(0.002)

        if speed3 != 0:
            if Motor3 < 0:
                ZDT.Emm_V5_Pos_Control(3, 0, speed3, 0, -Motor3, 1, 1)
            else:
                ZDT.Emm_V5_Pos_Control(3, 1, speed3, 0, Motor3, 1, 1)
        time.sleep(0.002)

        # 同步电机
        ZDT.Emm_V5_Synchronous_motion(0)

        # 更新上次的角度值
        self.last_motor1, self.last_motor2, self.last_motor3 = Motor1, Motor2, Motor3

    def MoveL(self, Target_Tx, Target_Ty, Target_Tz): # 直线插补运动_位置
        """直线插补运动_位置"""
        # 计算当前点和目标点之间的距离
        distance = math.sqrt((Target_Tx - self.Arm_Tx)**2 + (Target_Ty - self.Arm_Ty)**2 + (Target_Tz - self.Arm_Tz)**2)

        # 计算每次步进的数量
        steps = int(distance / self.Step_Size)
        print(f"Steps:{steps}")

        if steps != 0:
            # 计算每次步进的增量
            dx = (Target_Tx - self.Arm_Tx) / steps
            dy = (Target_Ty - self.Arm_Ty) / steps
            dz = (Target_Tz - self.Arm_Tz) / steps
            print(f"dx:{dx},dy:{dy},dz:{dz}")

            # 通过插补逐步移动到目标位置
            for step in range(steps):
                self.Arm_Tx += dx
                self.Arm_Ty += dy
                self.Arm_Tz += dz
                self.Move(self.Arm_Tx, self.Arm_Ty, self.Arm_Tz)  # 控制机械臂移动
                print(f"step_Arm_Txyz:{self.Arm_Tx},{self.Arm_Ty},{self.Arm_Tz}")
                time.sleep(0.002)

                # 检查是否有新的目标坐标更新
                if self.new_target_available:
                    return  # 如果有新的目标，提前退出当前的插补运动

            # 确保最终位置精确到目标位置
            self.Arm_Tx = Target_Tx
            self.Arm_Ty = Target_Ty
            self.Arm_Tz = Target_Tz
            # self.Move(self.Arm_Tx, self.Arm_Ty, self.Arm_Tz,speed)
            self.pos_cal(self.Arm_Tx, self.Arm_Ty, self.Arm_Tz)
            print(f"step_Arm_Txyz:{self.Arm_Tx},{self.Arm_Ty},{self.Arm_Tz}") 

    def MoveL_Speed(self, Speed_X, Speed_Y, Target_Tz, z_step_size): # 直线插补运动_速度(XY平面)
        """直线插补运动_速度(XY平面)"""
        flag = 0

        x_step = Speed_X / 1000.0
        y_step = Speed_Y / 1000.0

        z_distance = abs(Target_Tz - self.Arm_Tz)
        z_steps = int(z_distance / z_step_size)

        if z_steps != 0:
            flag = 1
            dz = (Target_Tz - self.Arm_Tz) / z_steps

        while Speed_X!=0 or Speed_Y!=0 or z_steps!=0:
            self.Arm_Tx += x_step
            self.Arm_Ty += y_step

            if self.Arm_Tx < -200.0:
                self.Arm_Tx = -200.0
            elif self.Arm_Tx > 200.0:
                self.Arm_Tx = 200.0

            if self.Arm_Ty < 241.0:
                self.Arm_Ty = 241.0
            elif self.Arm_Ty > 450.0:
                self.Arm_Ty = 450.0

            if z_steps != 0:
                self.Arm_Tz += dz
                z_steps -= 1

            ################################################更新全局模型，保持与UI同步
            gMODEL.xPos.set(self.Arm_Tx)
            gMODEL.yPos.set(self.Arm_Ty)
            ################################################

            self.Move(self.Arm_Tx, self.Arm_Ty, self.Arm_Tz)
            # print(f"speed_Arm_Txyz:{self.Arm_Tx},{self.Arm_Ty},{self.Arm_Tz}")
            time.sleep(0.002)

            if self.new_speed_available:
                return
        if flag != 0:
            self.Arm_Tz = Target_Tz

if __name__ == "__main__":
    arm = RoboArm()
    # 测试机械臂正解算

    # Pos_TrayPlace = [31800, 299, 584] # 放置到托盘的位置
    # Pos_TrayHover = [31800, 4, 1500] # 夹取前夹爪悬停的位置
    # Pos_TrayGrab = [31800, 4, 110] # 从托盘夹取的位置

    # x, y, z = arm.pos_cal_forward(0, 331, 4)
    # print(f"正解算结果: x={x}, y={y}, z={z}")

    # # 托盘放置点正解算
    # Px, Py, Pz = arm.pos_cal_forward(MotionParams.Pos_TrayPlace[0], MotionParams.Pos_TrayPlace[1], MotionParams.Pos_TrayPlace[2])
    # print(f"托盘放置点正解算: x={Px}, y={Py}, z={Pz}")

    # # 托盘悬停点正解算
    # Hx, Hy, Hz = arm.pos_cal_forward(MotionParams.Pos_TrayHover[0], MotionParams.Pos_TrayHover[1], MotionParams.Pos_TrayHover[2])
    # print(f"托盘悬停点正解算: x={Hx}, y={Hy}, z={Hz}")

    # # 托盘夹取点正解算
    # Gx, Gy, Gz = arm.pos_cal_forward(MotionParams.Pos_TrayGrab[0], MotionParams.Pos_TrayGrab[1], MotionParams.Pos_TrayGrab[2])
    # print(f"托盘夹取点正解算: x={Gx}, y={Gy}, z={Gz}")

    # # 托盘放置点逆解算
    # P_M1, P_M2, P_M3 = arm.pos_cal_global(Px, Py, Pz)
    # print(f"托盘放置点逆解算: M1={P_M1}, M2={P_M2}, M3={P_M3}")

    # # 托盘悬停点逆解算
    # H_M1, H_M2, H_M3 = arm.pos_cal_global(Hx, Hy, Hz)
    # print(f"托盘悬停点逆解算: M1={H_M1}, M2={H_M2}, M3={H_M3}")

    # # 托盘夹取点逆解算
    # G_M1, G_M2, G_M3 = arm.pos_cal_global(Gx, Gy, Gz)
    # print(f"托盘夹取点逆解算: M1={G_M1}, M2={G_M2}, M3={G_M3}")

    # arm.MoveP_AccDec_global([-118.3, -123.7, 249.7], MotionParams.VelAcc.ToTrayHover_Fast)
    # arm.MoveP_AccDec_global([-118.3, -123.7, 203.8], MotionParams.VelAcc.AUTO, [3, 1600, 185])

    arm.MoveP_AccDec(0, 298, 300, MotionParams.VelAcc.ToTrackBegin)
    arm.MoveP_AccDec(0, 298, 155, [[0, 0], [1600, 160], [1600, 200]])

    # time2 = arm.move_time_cal(clk=3592, vel=723, acc=1)
    # print(f"运动时间: {time2:.3f}秒")