import time
import threading
from PySide6.QtCore import QObject, Signal, Slot, QTimer

class RobotModel:
    def __init__(self):
        # 运行平台win或linux选择 0为win 1为linux
        self.platform = 0

        # 创建锁
        self.lock = threading.Lock()

        # 系统状态
        self.connection_status = "未连接"
        self.system_status = "离线"

        # 系统信息
        self.start_time = time.time()
        self.operation_count = 0
        
        # 舵机状态
        self.servo1_state = False  # 夹爪状态
        self.servo2_state = False  # 摄像头状态
        self.servo3_state = 0  # 托盘下舵机状态
        self.servo4_state = 1  # 托盘上舵机状态

        # 任务状态参数
        self.mainTaskIsRunning = False      # 主任务是否正在运行
        self.subTaskNum = 0                 # 子任务编号
        self.colourOrder = [1, 2, 3]        # 颜色顺序
        self.currentBatch = 0               # 当前物料批次
        self.taskLock = False               # 任务保险锁
        self.catchHeight = 113              # 原料区抓取高度 单位mm

        # 机械臂状态参数
        self.trackingEnabled = False  # 启用目标跟踪
        self.speedmodeEnabled = False  # 启用速度模式

        # 上位机界面绑定参数
        self.xPos = FloatVar(0.0)  # X轴位置
        self.yPos = FloatVar(175.0)  # Y轴位置
        self.zPos = FloatVar(199.0)  # Z轴位置
        self.speedX = FloatVar(0.0)  # X轴速度
        self.speedY = FloatVar(0.0)  # Y轴速度
        self.alpha0 = FloatVar(0.0)  # 底座转角
        self.armDirection = IntVar(1)  # 机械臂方向（1、2、3 或 4）
        self.QRdata = StringVar("初始化中")  # QR码数据
        self.isCarReached = IntVar(0)     # 小车是否到位
        self.isRotateReached = IntVar(0)  # 小车旋转到位标志

        self.hMin = IntVar(0)  # HSV色调最小值
        self.hMax = IntVar(179)  # HSV色调最大值
        self.sMin = IntVar(0)  # HSV饱和度最小值
        self.sMax = IntVar(255)  # HSV饱和度最大值
        self.vMin = IntVar(0)  # HSV明度最小值
        self.vMax = IntVar(255)  # HSV明度最大值

    def increment_operation_count(self):
        """增加操作计数"""
        self.operation_count += 1
    
    def get_uptime(self):
        """获取系统运行时间"""
        return time.time() - self.start_time
    
class FloatVar(QObject):
    """自定义浮点数变量类"""
    value_changed = Signal(float)

    def __init__(self, value=0.0):
        super().__init__()
        self._value = float(value)

    def get(self):
        return self._value

    def set(self, value):
        if self._value != value:
            self._value = float(value)
            self.value_changed.emit(self._value)

class IntVar(QObject):
    """自定义整数变量类"""
    value_changed = Signal(int)

    def __init__(self, value=0):
        super().__init__()
        self._value = int(value)

    def get(self):
        return self._value

    def set(self, value):
        if self._value != value:
            self._value = int(value)
            self.value_changed.emit(self._value)

class StringVar(QObject):
    """自定义字符串变量类"""
    value_changed = Signal(str)

    def __init__(self, value=""):
        super().__init__()
        self._value = str(value)

    def get(self):
        return self._value

    def set(self, value):
        if self._value != value:
            self._value = str(value)
            self.value_changed.emit(self._value)
    
GLOBAL_MODEL = RobotModel()