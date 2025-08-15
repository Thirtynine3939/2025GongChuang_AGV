from PySide6.QtCore import QObject, Signal, Slot, QTimer
from PySide6.QtWidgets import QMainWindow, QLineEdit, QSlider
from model import GLOBAL_MODEL as gMODEL
import time

import ZDT
from stm32 import STM32Controller
from robocal import RoboArm 
from robocal import MotionParams as MP
from video_processor import VideoProcessor
from positional_PID import PositionalPID

from HMI.hmi_view import MainWindow
from video_window import VideoWindow
import numpy as np
import cv2

class AppPresenter(QObject):
    # 定义信号用于线程间通信
    update_status_signal = Signal(str, str)  # (状态类型, 状态值)
    log_message_signal = Signal(str)
    show_error_signal = Signal(str)
    update_image_signal = Signal(dict)  # 用于传递OpenCV图像

    def __init__(self, parent=None):
        """
        初始化Presenter
        
        :param vp: VideoProcessor实例
        """
        super().__init__(parent)

        # 初始化各层组件
        self.STM32 = STM32Controller()
        self.arm = RoboArm()
        self.pid1 = PositionalPID(kp=16.00, ki=0.001, kd=5.00)
        self.pid2 = PositionalPID(kp=10.00, ki=0.0001, kd=5.00)
        self.view = MainWindow()
        if gMODEL.platform == 0:  # Windows
            self.vision = VideoProcessor(self.STM32, self.arm, self.view, 30, 0, 1)
        elif gMODEL.platform == 1:  # Linux
            self.vision = VideoProcessor(self.STM32, self.arm, self.view, 30, 2, 0)

        # 设置定时器检查视频帧
        self.video_timer = QTimer()
        self.video_timer.timeout.connect(self.check_video_frames)
        self.video_timer.start(30)  # 约30fps

        # 是否显示cv窗口控制变量
        self.show1_window = False
        self.show2_window = False
        self.show3_window = False
        
        # PySide6 视频窗口实例
        self.video_window1 = None
        self.video_window2 = None
        self.video_window3 = None

        # 初始化运行界面UI事件绑定
        self._init_ui_connect_running()

        # 初始化调试界面UI事件绑定
        self._init_ui_connect_debug()

        # 连接信号槽
        self.update_status_signal.connect(self.handle_status_update)
        self.log_message_signal.connect(self.view.add_log_message)
        self.show_error_signal.connect(self.view.show_error_message)
        self.update_image_signal.connect(self.handle_image_update)

        # 初始状态更新
        self.update_which_HSV()  # 初始化调节HSV用的UI
        self.update_center_slider() # 初始化摄像头中心坐标滑动条
        self.update_blue_delta() # 初始化蓝色色环补偿
        self.update_connection_status("已连接")
        self.update_system_status("系统初始化完成")
        self.add_log("应用程序启动")
        # 更新QR_data变量初始值
        # gMODEL.QRdata.set(self.vision.QR_DATA)

    # 初始化UI更新方法
    def update_center_slider(self):
        """初始化摄像头中心坐标滑动条"""
        self.view.slider_centerX.setValue(self.vision.center_x)
        self.view.slider_centerY.setValue(self.vision.center_y)

    def update_blue_delta(self):
        """初始化蓝色色环补偿"""
        self.view.entry_blueDeltaX.setText(f"{self.vision.blueCircleDelta_x}")
        self.view.entry_blueDeltaY.setText(f"{self.vision.blueCircleDelta_y}")

    #--------------------------绑定视图事件_运行界面-----------------------------#
    def _init_ui_connect_running(self):
        """绑定视图事件_运行界面"""
        # 全屏切换按钮
        self.view.btn_fullScreen.clicked.connect(lambda: self.view.showFullScreen() if not self.view.isFullScreen() else self.view.my_showNormal())

        # 开始读取二维码按钮
        self.view.btn_readQR.clicked.connect(self._start_reading_QR)

        # 转盘高度更新按钮
        self.view.btn_saveHeight.clicked.connect(self._update_table_height)

        # 出发姿态按钮
        self.view.btn_toStartPos.clicked.connect(self._move_to_start_position)

        # 调试姿态按钮
        self.view.btn_toDebugPos.clicked.connect(self._move_to_debug_position)

        # QRdata变量
        gMODEL.QRdata.value_changed.connect(lambda value: self.view.label_QR_Data.setText(value))

        # GOOOOOOOOOOOOOOO按钮
        self.view.btn_GO.clicked.connect(self.GOOOOOOOOOOOOOO)

    ###########################绑定视图事件_调试界面##############################
    def _init_ui_connect_debug(self):
        """绑定视图事件_调试界面"""
        # 位置输入框
        self.view.entry_xPos.editingFinished.connect(lambda: gMODEL.xPos.set(max(-200.0, min(200.0, float(self.view.entry_xPos.text())))))
        self.view.entry_yPos.editingFinished.connect(lambda: gMODEL.yPos.set(max(-200.0, min(200.0, float(self.view.entry_yPos.text())))))
        self.view.entry_zPos.editingFinished.connect(lambda: gMODEL.zPos.set(max(-200.0, min(200.0, float(self.view.entry_zPos.text())))))

        # 位置调整滑动条
        self.view.slider_xPos.valueChanged.connect(lambda value: gMODEL.xPos.set(value / 10.0))
        self.view.slider_yPos.valueChanged.connect(lambda value: gMODEL.yPos.set(value / 10.0))
        self.view.slider_zPos.valueChanged.connect(lambda value: gMODEL.zPos.set(value / 10.0))

        # 位置变量
        gMODEL.xPos.value_changed.connect(self.update_ui_xPos)
        gMODEL.yPos.value_changed.connect(self.update_ui_yPos)
        gMODEL.zPos.value_changed.connect(self.update_ui_zPos)

        # 位置调整滑动条实时控制开关
        self.view.sw_realTime.stateChanged.connect(self.on_realTime_switch)

        # 运动至设定位置按钮
        self.view.btn_mToSet.clicked.connect(self._update_targetPosition)

        # 插补步进距离大小调整滑动条
        self.view.slider_stepSize.valueChanged.connect(self._update_stepSize)

        # XY平面速度调整滑动条
        self.view.slider_speedX.valueChanged.connect(lambda value: gMODEL.speedX.set(value / 10.0))
        self.view.slider_speedY.valueChanged.connect(lambda value: gMODEL.speedY.set(value / 10.0))
        self.view.slider_speedX.valueChanged.connect(self._update_speed)  # 实时更新速度
        self.view.slider_speedY.valueChanged.connect(self._update_speed)  # 实时更新速度

        # XY平面速度变量
        gMODEL.speedX.value_changed.connect(self.update_ui_speedX)
        gMODEL.speedY.value_changed.connect(self.update_ui_speedY)

        # 速度模式开关
        self.view.sw_speedMode.stateChanged.connect(self.on_speedMode_switch)

        # 视觉跟踪开关
        self.view.sw_tracking.stateChanged.connect(self.on_trackingMode_switch)

        # 视觉跟踪开关2
        self.view.sw_tracking_2.stateChanged.connect(self.on_trackingMode_switch_2)

        # 机械臂失能开关（J1 ~ J3)
        self.view.sw_disEnJ13.stateChanged.connect(self.on_disEnJ13_switch)

        # 机械臂失能开关（J1 ~ J4)
        self.view.sw_disEnJ14.stateChanged.connect(self.on_disEnJ14_switch)

        # 读取各关节位置脉冲按钮
        self.view.btn_readClk.clicked.connect(self._read_joint_clkPos)

        # 底座转角变量
        gMODEL.alpha0.value_changed.connect(self.update_ui_alpha0)

        # 机械臂朝向设置多选框
        self.view.cBox_armDir.currentIndexChanged.connect(self._update_armDirection)

        # 机械臂朝向变量
        gMODEL.armDirection.value_changed.connect(self.update_ui_armDirection)

        # 末端倾斜定位模式开关
        self.view.sw_tiltEnd.stateChanged.connect(self.on_tiltEnd_switch)

        # 速度加速度参数选择多选框
        self.view.cBox_chooseVA.currentIndexChanged.connect(self._choose_velAccParams)

        # 点位移动按钮
        self.view.btn_toZero.clicked.connect(self._rel_move_to_Zero)
        self.view.btn_toTrackBegin.clicked.connect(self._rel_move_to_TrackBegin)
        self.view.btn_toPosFix.clicked.connect(self._rel_move_to_PosFix)
        self.view.btn_toTrayHover.clicked.connect(self._abs_move_to_TrayHover)
        self.view.btn_toTrayPlace.clicked.connect(self._abs_move_to_TrayPlace)
        self.view.btn_toTrayGrab.clicked.connect(self._abs_move_to_TrayGrab)
        self.view.btn_posFix2trayHover.clicked.connect(self._special_move_posFix2trayHover)

        # 速度加速度参数修改多选框
        self.view.cBox_modifyVA.currentTextChanged.connect(self._modify_velAccParams)

        # 速度加速度参数修改保存按钮
        self.view.btn_saveVAparams.clicked.connect(self._save_velAccParams)

        # 指定舵机运动角度按钮
        self.view.btn_moveServo.clicked.connect(self._move_servo_to_angle)

        # 舵机1_夹爪夹取开关
        self.view.sw_Grab.stateChanged.connect(self.on_servo1_switch)

        # 舵机2_摄像头前转倾斜开关
        self.view.sw_tiltCam.stateChanged.connect(self.on_servo2_switch)

        # 舵机3_托盘下舵机控制多选框
        self.view.cBox_setPosSevro3.currentTextChanged.connect(self._set_servo3_pos)

        # 舵机4_托盘上舵机控制多选框
        self.view.cBox_setPosSevro4.currentTextChanged.connect(self._set_servo4_pos)

        # pid1更新按钮
        self.view.btn_updatePID1.clicked.connect(self._update_pid1_params)

        # pid2更新按钮
        self.view.btn_updatePID2.clicked.connect(self._update_pid2_params)

        # 修改目标颜色多选框
        self.view.cBox_targetColor.currentTextChanged.connect(self._update_target_color)

        # 修改红色1或红色2阈值多选框
        self.view.cBox_Red1or2.currentTextChanged.connect(self._update_red1or2)

        # 修改目标对象多选框
        self.view.cBox_targetObject.currentTextChanged.connect(self._update_target_object)

        # HSV参数输入框
        self.view.entry_hMin.editingFinished.connect(lambda: gMODEL.hMin.set(max(0, min(179, int(self.view.entry_hMin.text())))))
        self.view.entry_hMax.editingFinished.connect(lambda: gMODEL.hMax.set(max(0, min(179, int(self.view.entry_hMax.text())))))
        self.view.entry_sMin.editingFinished.connect(lambda: gMODEL.sMin.set(max(0, min(255, int(self.view.entry_sMin.text())))))
        self.view.entry_sMax.editingFinished.connect(lambda: gMODEL.sMax.set(max(0, min(255, int(self.view.entry_sMax.text())))))
        self.view.entry_vMin.editingFinished.connect(lambda: gMODEL.vMin.set(max(0, min(255, int(self.view.entry_vMin.text())))))
        self.view.entry_vMax.editingFinished.connect(lambda: gMODEL.vMax.set(max(0, min(255, int(self.view.entry_vMax.text())))))

        # HSV参数滑动条
        self.view.slider_hMin.valueChanged.connect(lambda value: gMODEL.hMin.set(value))
        self.view.slider_hMax.valueChanged.connect(lambda value: gMODEL.hMax.set(value))
        self.view.slider_sMin.valueChanged.connect(lambda value: gMODEL.sMin.set(value))
        self.view.slider_sMax.valueChanged.connect(lambda value: gMODEL.sMax.set(value))
        self.view.slider_vMin.valueChanged.connect(lambda value: gMODEL.vMin.set(value))
        self.view.slider_vMax.valueChanged.connect(lambda value: gMODEL.vMax.set(value))

        # HSV参数变量
        gMODEL.hMin.value_changed.connect(lambda value: self.update_ui_HSV(value, self.view.entry_hMin, self.view.slider_hMin, "h", "Min"))
        gMODEL.hMax.value_changed.connect(lambda value: self.update_ui_HSV(value, self.view.entry_hMax, self.view.slider_hMax, "h", "Max"))
        gMODEL.sMin.value_changed.connect(lambda value: self.update_ui_HSV(value, self.view.entry_sMin, self.view.slider_sMin, "s", "Min"))
        gMODEL.sMax.value_changed.connect(lambda value: self.update_ui_HSV(value, self.view.entry_sMax, self.view.slider_sMax, "s", "Max"))
        gMODEL.vMin.value_changed.connect(lambda value: self.update_ui_HSV(value, self.view.entry_vMin, self.view.slider_vMin, "v", "Min"))
        gMODEL.vMax.value_changed.connect(lambda value: self.update_ui_HSV(value, self.view.entry_vMax, self.view.slider_vMax, "v", "Max"))

        # 重置当前HSV参数按钮
        self.view.btn_resetHSV.clicked.connect(self._reset_now_HSV_params)

        # 调整摄像头画面中心坐标滑动条
        self.view.slider_centerX.valueChanged.connect(lambda value: self.vision.set_centerX(value))
        self.view.slider_centerY.valueChanged.connect(lambda value: self.vision.set_centerY(value))

        # 小车到位标志变量
        gMODEL.isCarReached.value_changed.connect(lambda value: self.view.entryR_isCarReached.setText(f"{value}"))

        # 小车相对运动按钮
        self.view.btn_carRelMove.clicked.connect(self._car_rel_move)

        # 小车视觉定位修正开关
        self.view.sw_carPosFix.stateChanged.connect(self.on_carPosFix_switch)

        # 颜色顺序选择多选框
        self.view.cBox_colorOrder.currentTextChanged.connect(self._update_color_order)

        # 取消J1J2到位返回功能按钮
        self.view.btn_cancelReached.clicked.connect(self._cancel_reached)

        # 子任务测试开始按钮
        self.view.btn_taskStart.clicked.connect(self._start_subtask_test)

        # 取消所有任务按钮
        self.view.btn_taskCancel.clicked.connect(self._cancel_all_tasks)

        # 重启视频处理线程按钮
        self.view.btn_rebootVision.clicked.connect(lambda: self.vision.reboot())

        # 小车旋转按钮
        self.view.btn_carRotate.clicked.connect(self._car_rotate)

        # 小车旋转到位标志变量
        gMODEL.isRotateReached.value_changed.connect(lambda value: self.view.entryR_isCarRotateReached.setText(f"{value}"))

        # 摄像头界面1窗口化选择框
        self.view.check_show1.stateChanged.connect(self._update_show1_window)

        # 摄像头界面2窗口化选择框
        self.view.check_show2.stateChanged.connect(self._update_show2_window)

        # 摄像头界面3窗口化选择框
        self.view.check_show3.stateChanged.connect(self._update_show3_window)

        # 关节脉冲运动控制按钮
        self.view.btn_moveJ.clicked.connect(self._move_joint_to_clk)

        # 修改蓝色色环补偿按钮
        self.view.btn_saveblueDelta.clicked.connect(self._save_blueDelta)


    #--------------------------控件事件处理_比赛显示界面-----------------------------#
    @Slot()
    def GOOOOOOOOOOOOOO(self): # GOOOOOOOOOOOO!!!!!!!!!
        """GOOOOOOOOOOOO!!!!!!!!!"""
        gMODEL.mainTaskIsRunning = True
        self.add_log("GOOOOOOOOOOOO!!!!!!!!!")
        print("GOOOOOOOOOOOO!!!!!!!!!")
        gMODEL.QRdata.set(self.vision.QR_DATA)  # 更新QR码数据

    @Slot()
    def _start_reading_QR(self):
        """开始读取二维码"""
        self.vision.start_QR_Read()

    @Slot()
    def _update_table_height(self):
        """更新转盘高度"""
        try:
            tableHeight = float(self.view.entry_tableHeight.text())
            if 75 <= tableHeight <= 105:
                gMODEL.catchHeight = tableHeight + 33  # 设置抓取高度
                self.add_log(f"转盘高度已更新: {tableHeight:.1f} mm, 抓取高度为 {gMODEL.catchHeight:.1f} mm")
            else:
                self.show_error_signal.emit("转盘高度必须在75到105毫米之间")
        except ValueError:
            self.show_error_signal.emit("请输入有效的数字作为转盘高度")

    # @Slot() 可被外部调用
    def _move_to_start_position(self):
        """机械臂运动至出发姿态"""
        self.STM32.Servo_Move(1, 100)
        time.sleep(0.005)
        ZDT.Emm_V5_Pos_Control(4,1,1800,190,912,1,0)
        time.sleep(0.005)
        self.arm.MoveJ_AccDec([11202, 904, -1017], [[6400,220], [1500,120], [1500,100]])
        self.add_log("已运动至出发姿态")

    # @Slot() 可被外部调用
    def _move_to_debug_position(self):
        """机械臂运动至调试姿态"""
        self.STM32.Servo_Move(1, 215)
        time.sleep(0.005)
        ZDT.Emm_V5_Pos_Control(4,1,1600,160,MP.J4clk_normal,1,0)
        time.sleep(0.005)
        self.arm.MoveJ_AccDec([0, 331, 4], [[6400,220], [1500,120], [1500,120]])
        self.arm.new_target = [0, 175, 199]
        self.add_log("已运动至调试姿态")
    #------------------------------------------------------------------------------#

    # ============================控件事件处理_非比赛显示界面============================#
    @Slot(float)
    def update_ui_xPos(self, value):
        # 更新与xPos相关的UI元素
        self.view.entry_xPos.blockSignals(True) # 防止重复设置引发死循环
        self.view.entry_xPos.setText(f"{value:.1f}")
        self.view.entry_xPos.blockSignals(False)

        self.view.slider_xPos.blockSignals(True)
        self.view.slider_xPos.setValue(int(value * 10))  # 滑动条值为-2000 ~ 2000
        self.view.slider_xPos.blockSignals(False)

        self.view.label_slider_xPos.setText(f"{value:.1f}")

    @Slot(float)
    def update_ui_yPos(self, value):
        # 更新与yPos相关的UI元素
        self.view.entry_yPos.blockSignals(True)
        self.view.entry_yPos.setText(f"{value:.1f}")
        self.view.entry_yPos.blockSignals(False)

        self.view.slider_yPos.blockSignals(True)
        self.view.slider_yPos.setValue(int(value * 10))
        self.view.slider_yPos.blockSignals(False)

        self.view.label_slider_yPos.setText(f"{value:.1f}")

    @Slot(float)
    def update_ui_zPos(self, value):
        # 更新与zPos相关的UI元素
        self.view.entry_zPos.blockSignals(True)
        self.view.entry_zPos.setText(f"{value:.1f}")
        self.view.entry_zPos.blockSignals(False)

        self.view.slider_zPos.blockSignals(True)
        self.view.slider_zPos.setValue(int(value * 10))
        self.view.slider_zPos.blockSignals(False)

        self.view.label_slider_zPos.setText(f"{value:.1f}")

    @Slot(float)
    def update_ui_speedX(self, value):
        # 更新与speedX相关的UI元素
        self.view.slider_speedX.blockSignals(True)
        self.view.slider_speedX.setValue(int(value * 10)) # 滑动条值为-20000 ~ 20000
        self.view.slider_speedX.blockSignals(False)

        self.view.label_slider_speedX.setText(f"{value:.1f}")

    @Slot(float)
    def update_ui_speedY(self, value):
        # 更新与speedY相关的UI元素
        self.view.slider_speedY.blockSignals(True)
        self.view.slider_speedY.setValue(int(value * 10)) # 滑动条值为-20000 ~ 20000
        self.view.slider_speedY.blockSignals(False)

        self.view.label_slider_speedY.setText(f"{value:.1f}")

    @Slot(float)
    def update_ui_alpha0(self, value):
        # 更新与alpha0相关的UI元素
        self.view.entryR_alpha0.setText(f"{value:.1f}")

    @Slot(int)
    def update_ui_armDirection(self, value):
        # 更新与armDirection相关的UI元素
        self.view.cBox_armDir.blockSignals(True)
        self.view.cBox_armDir.setCurrentIndex(value - 1)
        self.view.cBox_armDir.blockSignals(False)

    def update_ui_HSV(self, value, entry: QLineEdit, slider: QSlider, channel: str, Min_or_Max: str):
        # 更新UI控件
        entry.blockSignals(True)
        entry.setText(f"{value}")
        entry.blockSignals(False)
        
        slider.blockSignals(True)
        slider.setValue(value)
        slider.blockSignals(False)
        
        # 确定阈值数组和索引
        color = self.view.cBox_targetColor.currentText()
        index = ["h", "s", "v"].index(channel)
        
        is_min = Min_or_Max == "Min"
        # 获取目标数组
        target_array = self.get_target_hsv_array(color, is_min)

        # 更新数组值
        if target_array is not None:
            target_array[index] = value

    def get_target_hsv_array(self, color, is_min):
        """根据颜色选择返回对应的阈值数组"""
        prefix = "lower" if is_min else "upper"
        
        if color == "红色":
            red_type = self.view.cBox_Red1or2.currentText()
            if red_type == "红色1":
                return getattr(self.vision, f"{prefix}_red1")
            elif red_type == "红色2":
                return getattr(self.vision, f"{prefix}_red2")
        
        elif color == "绿色":
            obj_type = self.view.cBox_targetObject.currentText()
            if obj_type == "物料":
                return getattr(self.vision, f"{prefix}_green0")
            elif obj_type == "色环":
                return getattr(self.vision, f"{prefix}_green1")
        
        elif color == "蓝色":
            return getattr(self.vision, f"{prefix}_blue")
        
        return None

    @Slot(bool)
    def on_realTime_switch(self, state):
        """处理实时控制开关"""
        if state == True:
            self.view.slider_xPos.valueChanged.connect(self._update_targetPosition)
            self.view.slider_yPos.valueChanged.connect(self._update_targetPosition)
            self.view.slider_zPos.valueChanged.connect(self._update_targetPosition)
            self.add_log("实时控制已启用: 滑动条将直接更新目标位置")
        else:
            self.view.slider_xPos.valueChanged.disconnect(self._update_targetPosition)
            self.view.slider_yPos.valueChanged.disconnect(self._update_targetPosition)
            self.view.slider_zPos.valueChanged.disconnect(self._update_targetPosition)
            self.add_log("实时控制已禁用: 滑动条将不再直接更新目标位置")

    @Slot()
    def _update_targetPosition(self): # 更新目标位置, 触发直线移动
        """更新目标位置, 触发直线移动"""
        x = gMODEL.xPos.get()
        y = gMODEL.yPos.get()
        z = gMODEL.zPos.get()

        if not gMODEL.speedmodeEnabled:         # 位置模式
            self.arm.trigger_MoveL(x, y, z)
        else:                                   # 速度模式
            self.arm.trigger_MoveZ_inSpeed(z)   # 速度模式中解耦移动z轴
        print(f"接收到的坐标:X = {x}, Y = {y}, Z = {z}")
        self.add_log(f"目标位置更新: X={x:.1f}, Y={y:.1f}, Z={z:.1f}")

    @Slot(int)
    def _update_stepSize(self, value): # 更新插补步进距离大小
        """更新插补步进距离大小"""
        step_size = value / 1000.0

        self.view.label_slider_stepSize.setText(f"{step_size:.3f}")

        if not gMODEL.speedmodeEnabled:         # 位置模式
            self.arm.trigger_Update_StepSize(step_size, mode="POSITION")
        else:                                   # 速度模式
            self.arm.trigger_Update_StepSize(step_size, mode="SPEED")
        print(f"接收到的步进值：{step_size}")

    @Slot()
    def _update_speed(self): # 更新速度, 触发速度模式直线移动
        """更新速度"""
        speedX = gMODEL.speedX.get()
        speedY = gMODEL.speedY.get()

        if not gMODEL.speedmodeEnabled:         # 位置模式
            print("当前为位置模式，速度更新无效")
        else:                                   # 速度模式
            self.arm.trigger_MoveL_Speed(speedX, speedY)
            print(f"接收到的速度： X = {speedX}, Y = {speedY}")

    @Slot(bool)
    def on_speedMode_switch(self, state):
        """处理速度模式开关"""
        gMODEL.speedmodeEnabled = state
        if state:
            self.add_log("速度模式已启用: 机械臂将以速度模式运行")
        else:
            self.add_log("速度模式已禁用: 机械臂将以位置模式运行")

    @Slot(bool)
    def on_trackingMode_switch(self, state):
        """处理视觉跟踪开关"""
        gMODEL.trackingEnabled = state
        if state:
            self.add_log("视觉跟踪已启用")
        else:
            self.add_log("视觉跟踪已禁用")

        self.view.sw_tracking_2.blockSignals(True)
        if state:
            self.view.sw_tracking_2.setOn()
        else:
            self.view.sw_tracking_2.setOff()
        self.view.sw_tracking_2.blockSignals(False)

    @Slot(bool)
    def on_trackingMode_switch_2(self, state):
        """处理视觉跟踪开关"""
        gMODEL.trackingEnabled = state
        if state:
            self.add_log("视觉跟踪已启用")
        else:
            self.add_log("视觉跟踪已禁用")

        self.view.sw_tracking.blockSignals(True)
        if state:
            self.view.sw_tracking.setOn()
        else:
            self.view.sw_tracking.setOff()
        self.view.sw_tracking.blockSignals(False)

    @Slot(bool)
    def on_disEnJ13_switch(self, state):
        """处理机械臂J1 ~ J3失能开关"""
        if state:
            ZDT.Emm_V5_En_Control(1, 0, False)
            time.sleep(0.0025)
            ZDT.Emm_V5_En_Control(2, 0, False)
            time.sleep(0.0025)
            ZDT.Emm_V5_En_Control(3, 0, False)
            self.add_log("机械臂J1 ~ J3已失能")
        else:
            ZDT.Emm_V5_En_Control(1, 1, False)
            time.sleep(0.0025)
            ZDT.Emm_V5_En_Control(2, 1, False)
            time.sleep(0.0025)
            ZDT.Emm_V5_En_Control(3, 1, False)
            self.add_log("机械臂J1 ~ J3已启用")

    @Slot(bool)
    def on_disEnJ14_switch(self, state):
        """处理机械臂J1 ~ J4失能开关"""
        if state:
            ZDT.Emm_V5_En_Control(1, 0, False)
            time.sleep(0.0025)
            ZDT.Emm_V5_En_Control(2, 0, False)
            time.sleep(0.0025)
            ZDT.Emm_V5_En_Control(3, 0, False)
            time.sleep(0.0025)
            ZDT.Emm_V5_En_Control(4, 0, False)
            self.add_log("机械臂J1 ~ J4已失能")
        else:
            ZDT.Emm_V5_En_Control(1, 1, False)
            time.sleep(0.0025)
            ZDT.Emm_V5_En_Control(2, 1, False)
            time.sleep(0.0025)
            ZDT.Emm_V5_En_Control(3, 1, False)
            time.sleep(0.0025)
            ZDT.Emm_V5_En_Control(4, 1, False)
            self.add_log("机械臂J1 ~ J4已启用")

    @Slot()
    def _read_joint_clkPos(self):
        """读取各关节位置脉冲"""
        ZDT.Emm_V5_Read_Sys_Params(1, 'S_CLK')
        time.sleep(0.003)
        ZDT.Emm_V5_Read_Sys_Params(2, 'S_CLK')
        time.sleep(0.003)
        ZDT.Emm_V5_Read_Sys_Params(3, 'S_CLK')
        time.sleep(0.003)
        ZDT.Emm_V5_Read_Sys_Params(4, 'S_CLK')

        time.sleep(0.08) # 等待数据返回
        # 更新UI
        clk1 = ZDT.Motor_CLK[0]
        clk2 = ZDT.Motor_CLK[1]
        clk3 = ZDT.Motor_CLK[2]
        clk4 = ZDT.Motor_CLK[3]
        self.view.update_joint_positions(clk1, clk2, clk3, clk4)
        self.add_log("已读取各关节位置脉冲")

    @Slot(int)
    def _update_armDirection(self, index):
        """更新机械臂朝向"""
        gMODEL.armDirection.set(index + 1)

        self.arm.set_armDir(index + 1)
        self.add_log(f"注意！！！机械臂朝向已设置为: {index + 1}，请谨慎进行相对点位移动！")

    @Slot(bool)
    def on_tiltEnd_switch(self, state):
        """处理末端倾斜定位模式开关"""
        if state:
            ZDT.Emm_V5_Pos_Control(4,1,800,100,MP.J4clk_tilted,1,0)
            time.sleep(0.005)
            self.STM32.Servo2_Ctrl(1)
            self.add_log("末端倾斜")
        else:
            ZDT.Emm_V5_Pos_Control(4,1,800,100,MP.J4clk_normal,1,0)
            time.sleep(0.005)
            self.STM32.Servo2_Ctrl(0)
            self.add_log("末端水平")

    @Slot(int)
    def _choose_velAccParams(self, index):
        """选择速度加速度参数"""
        if index == 0:
            self.add_log("已选择全部默认参数")
        else:
            self.add_log("注意！！！已取消选择全部默认参数，此时不建议进行点位移动！")

    @Slot()
    def _rel_move_to_Zero(self): # 相对移动到零点位置
        """相对移动到零点位置"""
        ZDT.Emm_V5_Pos_Control(4,1,1200,120,MP.J4clk_normal,1,0)
        time.sleep(0.005)
        nowParamName = self.view.cBox_chooseVA.currentText()
        if nowParamName == "ALLDEFAULT":
            self.arm.MoveP_AccDec(0, 175, 199, MP.VelAcc.ZERO)
        else:
            self.arm.MoveP_AccDec(0, 175, 199, getattr(MP.VelAcc, nowParamName))
        self.add_log(f"相对移动到零点：{self.arm.new_target}")

    @Slot()
    def _rel_move_to_TrackBegin(self): # 相对移动到物料跟踪起点
        """相对移动到物料跟踪起点"""
        ZDT.Emm_V5_Pos_Control(4,1,1200,120,MP.J4clk_normal,1,0)
        time.sleep(0.005)
        nowParamName = self.view.cBox_chooseVA.currentText()
        if nowParamName == "ALLDEFAULT":
            self.arm.MoveP_AccDec(0, 370, 230, MP.VelAcc.ToTrackBegin)
        else:
            self.arm.MoveP_AccDec(0, 370, 230, getattr(MP.VelAcc, nowParamName))
        self.add_log(f"相对移动到物料跟踪起点：{self.arm.new_target}")

    @Slot()
    def _rel_move_to_PosFix(self): # 相对移动到小车定位点
        """相对移动到小车定位点"""
        ZDT.Emm_V5_Pos_Control(4,1,1200,120,MP.J4clk_normal,1,0)
        time.sleep(0.005)
        nowParamName = self.view.cBox_chooseVA.currentText()
        if nowParamName == "ALLDEFAULT":
            self.arm.MoveP_AccDec(0, 298, 155, MP.VelAcc.ToTrackBegin)
        else:
            self.arm.MoveP_AccDec(0, 298, 155, getattr(MP.VelAcc, nowParamName))
        self.add_log(f"相对移动到小车定位点：{self.arm.new_target}")

    @Slot()
    def _abs_move_to_TrayHover(self): # 绝对移动到托盘悬停点
        """绝对移动到托盘悬停点"""
        nowParamName = self.view.cBox_chooseVA.currentText()
        if nowParamName == "ALLDEFAULT":
            self.arm.MoveJ_AccDec(MP.Pos_TrayHover, MP.VelAcc.ToTrayHover_Normal)
        else:
            self.arm.MoveJ_AccDec(MP.Pos_TrayHover, getattr(MP.VelAcc, nowParamName))
        self.add_log(f"绝对移动到托盘悬停点（单位为关节脉冲）：{MP.Pos_TrayHover}")

    @Slot()
    def _abs_move_to_TrayPlace(self): # 绝对移动到托盘放置点
        """绝对移动到托盘放置点"""
        nowParamName = self.view.cBox_chooseVA.currentText()
        if nowParamName == "ALLDEFAULT":
            self.arm.MoveJ_AccDec(MP.Pos_TrayPlace, MP.VelAcc.Dir1toTrayPlace_Fast)
        else:
            self.arm.MoveJ_AccDec(MP.Pos_TrayPlace, getattr(MP.VelAcc, nowParamName))
        self.add_log(f"绝对移动到托盘放置点（单位为关节脉冲）：{MP.Pos_TrayPlace}")

    @Slot()
    def _abs_move_to_TrayGrab(self): # 绝对移动到托盘抓取点
        """绝对移动到托盘抓取点"""
        nowParamName = self.view.cBox_chooseVA.currentText()
        if nowParamName == "ALLDEFAULT":
            self.arm.MoveJ_AccDec(MP.Pos_TrayGrab, MP.VelAcc.ToTrayGrab)
        else:
            self.arm.MoveJ_AccDec(MP.Pos_TrayGrab, getattr(MP.VelAcc, nowParamName))
        self.add_log(f"绝对移动到托盘抓取点（单位为关节脉冲）：{MP.Pos_TrayGrab}")

    @Slot()
    def _special_move_posFix2trayHover(self): # 特殊点位移动：小车定位点到托盘悬停点
        """特殊点位移动：小车定位点到托盘悬停点"""
        ZDT.Emm_V5_Pos_Control(3, 0, 2500, 210, 2800, 1, 0)
        time.sleep(0.2)
        self.arm.MoveJ_AccDec(MP.Pos_TrayHover, MP.VelAcc.ArmFix_ToTrayHover)
        self.add_log(f"特殊点位移动：小车定位点到托盘悬停点（单位为关节脉冲）：{MP.Pos_TrayHover}")

    @Slot(str)
    def _modify_velAccParams(self, param_name):
        """修改速度加速度参数"""
        nowParam = MP.VelAccParams[param_name]
        self.view.entry_m1Vel.setText(f"{nowParam[0][0]}")
        self.view.entry_m1Acc.setText(f"{nowParam[0][1]}")
        self.view.entry_m2Vel.setText(f"{nowParam[1][0]}")
        self.view.entry_m2Acc.setText(f"{nowParam[1][1]}")
        self.view.entry_m3Vel.setText(f"{nowParam[2][0]}")
        self.view.entry_m3Acc.setText(f"{nowParam[2][1]}")

    @Slot()
    def _save_velAccParams(self):
        """保存速度加速度参数"""
        m1Vel = int(self.view.entry_m1Vel.text())
        m1Acc = int(self.view.entry_m1Acc.text())
        m2Vel = int(self.view.entry_m2Vel.text())
        m2Acc = int(self.view.entry_m2Acc.text())
        m3Vel = int(self.view.entry_m3Vel.text())
        m3Acc = int(self.view.entry_m3Acc.text())

        nowParamName = self.view.cBox_modifyVA.currentText()

        # 更新参数 - 使用setattr动态设置类属性
        new_params = [[m1Vel, m1Acc], [m2Vel, m2Acc], [m3Vel, m3Acc]]
        setattr(MP.VelAcc, nowParamName, new_params)
        self.add_log(f"已保存速度加速度参数{nowParamName}:  {MP.VelAccParams[nowParamName]}")

    @Slot()
    def _move_servo_to_angle(self):
        """指定舵机运动角度"""
        num = int(self.view.cBox_chooseServo.currentText())
        angle = float(self.view.entry_servoDegree.text())

        if angle < 0 or angle > 270:
            self.show_error("舵机角度必须在0到270之间")
            return

        self.STM32.Servo_Move(num, angle)
        self.add_log(f"已将舵机{num}移动到角度: {angle}°")

    @Slot(bool)
    def on_servo1_switch(self, state): # 切换舵机1位置
        """切换舵机1位置"""
        self.STM32.Gripper_En(state)
        gMODEL.servo1_state = state
        status = "闭合" if state else "松开"
        self.add_log(f"夹爪舵机1: {status}")

    @Slot(bool)
    def on_servo2_switch(self, state): # 切换舵机2位置
        """切换舵机2位置"""
        self.STM32.Servo2_Ctrl(state)
        gMODEL.servo2_state = state
        status = "朝前" if state else "朝下"
        self.add_log(f"摄像头舵机2: {status}")

    @Slot(str)
    def _set_servo3_pos(self, posName): # 设置舵机3位置
        """设置舵机3位置"""
        posNum = int(posName[0])
        self.STM32.Servo3_Ctrl(posNum)
        gMODEL.servo3_state = posNum
        self.add_log("舵机3位置更新")

    @Slot(str)
    def _set_servo4_pos(self, posName): # 设置舵机4位置
        """设置舵机4位置"""
        posNum = int(posName[0])
        self.STM32.Servo4_Ctrl(posNum)
        gMODEL.servo4_state = posNum
        self.add_log("舵机4位置更新")

    @Slot()
    def _update_pid1_params(self):
        """更新PID1参数"""
        kp = float(self.view.entry_pid1_Kp.text())
        ki = float(self.view.entry_pid1_Ki.text())
        kd = float(self.view.entry_pid1_Kd.text())

        self.pid1.update_params(kp, ki, kd)
        self.add_log(f"PID1参数已更新: Kp={kp}, Ki={ki}, Kd={kd}")

    @Slot()
    def _update_pid2_params(self):
        """更新PID2参数"""
        kp = float(self.view.entry_pid2_Kp.text())
        ki = float(self.view.entry_pid2_Ki.text())
        kd = float(self.view.entry_pid2_Kd.text())

        self.pid2.update_params(kp, ki, kd)
        self.add_log(f"PID2参数已更新: Kp={kp}, Ki={ki}, Kd={kd}")

    def update_which_HSV(self):
        """更新当前选择的HSV参数并刷新UI"""
        # 获取当前选择的HSV范围
        nowHSV = self.get_current_hsv_range()
        
        # 定义UI控件映射 [min_entry, min_slider, max_entry, max_slider]
        ui_mapping = [
            (self.view.entry_hMin, self.view.slider_hMin, 
            self.view.entry_hMax, self.view.slider_hMax, 0),  # H通道
            (self.view.entry_sMin, self.view.slider_sMin,
            self.view.entry_sMax, self.view.slider_sMax, 1),  # S通道
            (self.view.entry_vMin, self.view.slider_vMin,
            self.view.entry_vMax, self.view.slider_vMax, 2)   # V通道
        ]
        
        # 更新所有UI控件
        for min_entry, min_slider, max_entry, max_slider, idx in ui_mapping:
            # 更新最小值控件
            self.update_ui_control(min_entry, min_slider, nowHSV[0][idx])
            
            # 更新最大值控件
            self.update_ui_control(max_entry, max_slider, nowHSV[1][idx])

    def get_current_hsv_range(self):
        """获取当前选择的HSV范围"""
        targetColor = self.view.cBox_targetColor.currentText()
        red1_or_red2 = self.view.cBox_Red1or2.currentText()
        targetObject = self.view.cBox_targetObject.currentText()

        if targetColor == "红色":
            if red1_or_red2 == "红色1":
                return [self.vision.lower_red1.tolist(), self.vision.upper_red1.tolist()]
            elif red1_or_red2 == "红色2":
                return [self.vision.lower_red2.tolist(), self.vision.upper_red2.tolist()]
        elif targetColor == "绿色":
            if targetObject == "物料":
                return [self.vision.lower_green0.tolist(), self.vision.upper_green0.tolist()]
            elif targetObject == "色环":
                return [self.vision.lower_green1.tolist(), self.vision.upper_green1.tolist()]
        elif targetColor == "蓝色":
            return [self.vision.lower_blue.tolist(), self.vision.upper_blue.tolist()]
        
        # 默认返回空列表（可根据需要调整）
        return [[0, 0, 0], [0, 0, 0]]

    def update_ui_control(self, entry: QLineEdit, slider: QSlider, value: int):
        """更新单个UI控件(文本框和滑块)"""
        # 更新文本框
        entry.blockSignals(True)
        entry.setText(f"{value}")
        entry.blockSignals(False)
        
        # 更新滑块
        slider.blockSignals(True)
        slider.setValue(value)
        slider.blockSignals(False)

    @Slot(str)
    def _update_target_color(self, color):
        """更新目标颜色"""
        # 只有为红色时才启用红色1或红色2选项
        if color == "红色":
            self.view.cBox_Red1or2.setEnabled(True)
        else:
            self.view.cBox_Red1or2.setEnabled(False)

        self.vision.choose_colour = ["红色", "绿色", "蓝色"].index(color) + 1  # 更新视觉处理的颜色选择

        self.update_which_HSV()  # 更新HSV参数

        self.add_log(f"目标颜色已更新: {color}")

    @Slot(str)
    def _update_red1or2(self, red_type):
        """更新红色1或红色2选项"""
        self.update_which_HSV()  # 更新HSV参数

        self.add_log(f"当前红色: {red_type}")

    @Slot(str)
    def _update_target_object(self, obj):
        """更新目标物体"""
        self.vision.Detect_Mode = ["物料", "色环"].index(obj)  # 更新视觉处理的目标物体选择
        self.update_which_HSV()  # 更新HSV参数

        self.add_log(f"目标物体已更新: {obj}")

    @Slot()
    def _reset_now_HSV_params(self):
        """重置当前HSV参数"""
        default_hsv = {
            "lower_red1": np.array([0, 130, 130]),
            "upper_red1": np.array([10, 255, 255]),
            "lower_red2": np.array([150, 30, 70]),
            "upper_red2": np.array([180, 255, 255]),

            "lower_green0": np.array([40, 50, 50]),
            "upper_green0": np.array([81, 255, 255]),
            "lower_green1": np.array([40, 30, 50]),
            "upper_green1": np.array([90, 255, 255]),

            "lower_blue": np.array([100, 100, 60]),
            "upper_blue": np.array([255, 255, 255])
        }

        nowColor = self.view.cBox_targetColor.currentText()
        nowRed1or2 = self.view.cBox_Red1or2.currentText()
        nowObject = self.view.cBox_targetObject.currentText()

        if nowColor == "红色":
            if nowRed1or2 == "红色1":
                self.vision.lower_red1 = default_hsv["lower_red1"]
                self.vision.upper_red1 = default_hsv["upper_red1"]
            elif nowRed1or2 == "红色2":
                self.vision.lower_red2 = default_hsv["lower_red2"]
                self.vision.upper_red2 = default_hsv["upper_red2"]
        elif nowColor == "绿色":
            if nowObject == "物料":
                self.vision.lower_green0 = default_hsv["lower_green0"]
                self.vision.upper_green0 = default_hsv["upper_green0"]
            elif nowObject == "色环":
                self.vision.lower_green1 = default_hsv["lower_green1"]
                self.vision.upper_green1 = default_hsv["upper_green1"]
        elif nowColor == "蓝色":
            self.vision.lower_blue = default_hsv["lower_blue"]
            self.vision.upper_blue = default_hsv["upper_blue"]

        self.update_which_HSV()  # 更新UI显示

        self.add_log("当前HSV参数已重置为默认值")

    @Slot()
    def _car_rel_move(self):
        """小车相对移动"""
        X = int(self.view.entry_carRelX.text())
        Y = int(self.view.entry_carRelY.text())
        T = int(self.view.entry_carRelT.text())
        
        self.STM32.CarMove_Rel([X, Y, T])
        self.add_log(f"小车相对移动: X={X}, Y={Y}, T={T}")

    @Slot(bool)
    def on_carPosFix_switch(self, state):
        """小车视觉定位修正开关"""
        self.vision.poscali_enable_flag = state
        self.STM32.Start_or_Stop_PosCali(state, self.arm.armDir)
        self.add_log(f"小车视觉定位修正{'已启用' if state else '已禁用'}, 机械臂朝向: {self.arm.armDir}")

    @Slot(str)
    def _update_color_order(self, color_order_text):
        """更新颜色顺序"""
        if gMODEL.subTaskNum == 0: # 任务未运行时才允许更新颜色顺序
            order = [int(char) for char in color_order_text]
            self.vision.read_colour_order[0] = order
            self.vision.choose_colour = order[0]            
            self.add_log(f"颜色顺序已更新: {color_order_text}")
        else:
            self.show_error("任务运行中，无法修改颜色顺序！请先停止任务。")

    @Slot()
    def _cancel_reached(self):
        """取消J1J2到位返回功能"""
        ZDT.Emm_V5_Reached_En(1, 0)
        time.sleep(0.03)
        ZDT.Emm_V5_Reached_En(2, 0)
        self.add_log("已取消J1J2到位返回功能")

    @Slot()
    def _start_subtask_test(self):
        """开始子任务测试"""
        gMODEL.taskLock = 1
        subTaskText = self.view.cBox_subTaskNum.currentText()
        num = int(subTaskText.split(" ")[0])  # 获取子任务编号
        gMODEL.subTaskNum = num  # 更新全局模型中的子任务编号
        self.add_log("子任务测试已开始")

    @Slot()
    def _cancel_all_tasks(self):
        """取消所有任务"""
        gMODEL.mainTaskIsRunning = False  # 停止主任务
        gMODEL.subTaskNum = 0  # 重置子任务编号
        gMODEL.taskLock = 0  # 解锁任务
        self.add_log("所有任务已取消")

    @Slot()
    def _car_rotate(self):
        """小车旋转"""
        angle = int(self.view.entry_carRotateA.text())
        speed = int(self.view.entry_carRotateS.text())

        self.STM32.Car_Rotate(angle, speed)
        self.add_log("小车旋转指令已发送")

    @Slot(int)
    def _update_show1_window(self, state):
        """更新是否显示frame1窗口"""
        self.show1_window = state == 2
        if self.show1_window:
            if self.video_window1 is None:
                self.video_window1 = VideoWindow("Camera Feed")
            self.video_window1.show()
            self.add_log("已启用Camera Feed窗口")
        else:
            if self.video_window1 is not None:
                self.video_window1.close()
                self.video_window1 = None
            self.add_log("已禁用Camera Feed窗口")

    @Slot(int)
    def _update_show2_window(self, state):
        """更新是否显示frame2窗口"""
        self.show2_window = state == 2
        if self.show2_window:
            if self.video_window2 is None:
                self.video_window2 = VideoWindow("Gauss_Mask")
            self.video_window2.show()
            self.add_log("已启用Gauss_Mask窗口")
        else:
            if self.video_window2 is not None:
                self.video_window2.close()
                self.video_window2 = None
            self.add_log("已禁用Gauss_Mask窗口")

    @Slot(int)
    def _update_show3_window(self, state):
        """更新是否显示frame3窗口"""
        self.show3_window = state == 2
        if self.show3_window:
            if self.video_window3 is None:
                self.video_window3 = VideoWindow("QR_Code")
            self.video_window3.show()
            self.add_log("已启用QR_Code窗口")
        else:
            if self.video_window3 is not None:
                self.video_window3.close()
                self.video_window3 = None
            self.add_log("已禁用QR_Code窗口")

    @Slot()
    def _move_joint_to_clk(self):
        """移动关节到指定脉冲"""
        addr = self.view.cBox_chooseJ.currentIndex() + 1  # 获取关节编号
        vel = int(self.view.entry_J_vel.text())
        acc = int(self.view.entry_J_acc.text())
        clk = int(self.view.entry_J_clk.text())
        dir = 0 if clk >= 0 else 1  # 确定方向

        ZDT.Emm_V5_Pos_Control(addr, dir, vel, acc, abs(clk), 1, 0)  # 发送关节移动指令
        self.add_log(f"关节移动指令已发送: 关节={addr}, 方向={dir}, 速度={vel}, 加速度={acc}, 脉冲={abs(clk)}")

    @Slot()
    def _save_blueDelta(self):
        """修改蓝色色环补偿"""
        self.vision.blueCircleDelta_x = int(self.view.entry_blueDeltaX.text())
        self.vision.blueCircleDelta_y = int(self.view.entry_blueDeltaY.text())
        self.add_log(f"蓝色色环补偿参数已保存: {self.vision.blueCircleDelta_x}, {self.vision.blueCircleDelta_y}")

    # ===================================================================#

    ##############################################################图像操作
    def check_video_frames(self):
        """定时检查并处理视频帧"""
        frame_data = self.vision.get_frame()
        if frame_data is not None:
            self.update_image_signal.emit(frame_data)

    @Slot(dict)
    def handle_image_update(self, frame_data):
        """处理图像更新 - 在主线程执行"""
        frame = frame_data["frame"]
        mask = frame_data["mask"]
        qr_frame = frame_data["qr_frame"]
        
        # 显示在PySide6窗口
        if self.show1_window and self.video_window1 is not None:
            self.video_window1.update_frame(frame)
        if self.show2_window and self.video_window2 is not None:
            self.video_window2.update_frame(mask)
        if self.show3_window and self.video_window3 is not None:
            self.video_window3.update_frame(qr_frame)

        # 同时更新到Qt UI中
        self.view.update_video_frame_1(frame)
        self.view.update_video_frame_2(mask)
        self.view.update_video_frame_3(qr_frame)
    ##############################################################

    def update_connection_status(self, status):
        """更新连接状态"""
        gMODEL.connection_status = status
        self.update_status_signal.emit("connection", status)

    def update_system_status(self, status):
        """更新系统状态"""
        gMODEL.system_status = status
        self.update_status_signal.emit("system", status)

    def add_log(self, message):
        """添加日志消息"""
        self.log_message_signal.emit(message)

    def show_error(self, message):
        """显示错误消息"""
        self.show_error_signal.emit(message)

    def handle_status_update(self, status_type, value):
        """处理状态更新信号"""
        if status_type == "connection":
            self.view.update_connection_status(value)
        elif status_type == "system":
            self.view.update_system_status(value)

    def shutdown(self):
        """清理资源"""
        # 停止机械臂插补线程
        self.arm.stop()

        # 停止视频定时器
        self.video_timer.stop()
        
        # 关闭所有视频窗口
        if self.video_window1 is not None:
            self.video_window1.close()
            self.video_window1 = None
        if self.video_window2 is not None:
            self.video_window2.close()
            self.video_window2 = None
        if self.video_window3 is not None:
            self.video_window3.close()
            self.video_window3 = None
        
        # 停止视频处理
        if self.vision:
            self.vision.stop()

        # 添加日志
        self.add_log("系统安全关闭")