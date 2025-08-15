import sys
from PySide6 import QtWidgets, QtCore, QtGui
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QApplication, QMainWindow

from HMI.Ui_hmi_view import Ui_MainWindow

import numpy as np
import cv2

##################################################################################################### 单独显示UI界面
if __name__ == '__main__':
    class MainWindow(QMainWindow, Ui_MainWindow):
        def __init__(self):
            super(MainWindow, self).__init__()
            self.setupUi(self)  # 设置UI
    
    app = QApplication(sys.argv)  # 实例化QApplication类
    mainWindow = MainWindow()  # 创建MainWindow实例
    mainWindow.show()  # 显示窗口
    sys.exit(app.exec())  # 进入程序的主循环，遇到退出情况，终止程序
#####################################################################################################

class MainWindow(QMainWindow, Ui_MainWindow):
    # 定义主窗口关闭信号
    window_closing_signal = Signal()
    
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)

        # 初始设置为全屏
        self.showFullScreen()

    def closeEvent(self, event):
        """重写closeEvent方法，在主窗口关闭时发出信号"""
        self.window_closing_signal.emit()
        event.accept()  # 接受关闭事件

    def my_showNormal(self):
        """显示窗口为正常大小"""
        self.showNormal()
        self.resize(1080, 720)  # 设置窗口大小

    def update_connection_status(self, status):
        """更新连接状态"""
        color = "green" if "已连接" in status else "red"
        self.connectionLabel.setText(f"状态: {status}")
        self.connectionLabel.setStyleSheet(f"color: {color};")

    def update_system_status(self, status):
        """更新系统状态"""
        self.sysLabel.setText(f"系统: {status}")

    def add_log_message(self, message):
        """添加日志消息"""
        timestamp = QtCore.QDateTime.currentDateTime().toString("hh:mm:ss")
        self.logText.append(f"[{timestamp}] {message}")
        self.logText_2.append(f"[{timestamp}] {message}")
        self.logText_3.append(f"[{timestamp}] {message}")

    def show_error_message(self, message):
        """显示错误消息"""
        self.logText.append(f"错误： {message}")
        self.logText_2.append(f"错误： {message}")
        self.logText_3.append(f"错误： {message}")

    def update_video_frame_1(self, frame):
        """在1号视频流框中显示OpenCV图像"""
        if frame is None or frame.size == 0:
            return
            
        # 将OpenCV图像转换为Qt格式
        if len(frame.shape) == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            height, width = frame.shape[:2]
            pixmap = QImage(frame, width, height, QImage.Format_RGB888)
        elif len(frame.shape) == 2:  # 灰度图，单通道
            height, width = frame.shape[:2]
            pixmap = QImage(frame, width, height, QImage.Format_Grayscale8)
        else:
            print("未知图像格式")
            return
        
        pixmap = QPixmap.fromImage(pixmap)
        # 获取视频流和label窗口的长宽比值的最大值，适应label窗口播放，不然显示不全
        ratio = max(width / self.frameLabel.width(), height / self.frameLabel.height())
        pixmap.setDevicePixelRatio(ratio)
        # 视频流置于label中间部分播放
        self.frameLabel.setAlignment(Qt.AlignCenter)
        self.frameLabel.setPixmap(pixmap)

    def update_video_frame_2(self, frame):
        """在2号视频流框中显示OpenCV图像"""
        if frame is None or frame.size == 0:
            return
            
        # 将OpenCV图像转换为Qt格式
        if len(frame.shape) == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            height, width = frame.shape[:2]
            pixmap = QImage(frame, width, height, QImage.Format_RGB888)
        elif len(frame.shape) == 2:  # 灰度图，单通道
            height, width = frame.shape[:2]
            pixmap = QImage(frame, width, height, QImage.Format_Grayscale8)
        else:
            print("未知图像格式")
            return
        
        pixmap = QPixmap.fromImage(pixmap)
        # 获取视频流和label窗口的长宽比值的最大值，适应label窗口播放，不然显示不全
        ratio = max(width / self.frameLabel_2.width(), height / self.frameLabel_2.height())
        pixmap.setDevicePixelRatio(ratio)
        # 视频流置于label中间部分播放
        self.frameLabel_2.setAlignment(Qt.AlignCenter)
        self.frameLabel_2.setPixmap(pixmap)

    def update_video_frame_3(self, frame):
        """在3号视频流框中显示OpenCV图像"""
        if frame is None or frame.size == 0:
            return

        # 将OpenCV图像转换为Qt格式
        if len(frame.shape) == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            height, width = frame.shape[:2]
            pixmap = QImage(frame, width, height, QImage.Format_RGB888)
        elif len(frame.shape) == 2:  # 灰度图，单通道
            height, width = frame.shape[:2]
            pixmap = QImage(frame, width, height, QImage.Format_Grayscale8)
        else:
            print("未知图像格式")
            return
        
        pixmap = QPixmap.fromImage(pixmap)
        # 获取视频流和label窗口的长宽比值的最大值，适应label窗口播放，不然显示不全
        ratio = max(width / self.frameLabel_3.width(), height / self.frameLabel_3.height())
        pixmap.setDevicePixelRatio(ratio)
        # 视频流置于label中间部分播放
        self.frameLabel_3.setAlignment(Qt.AlignCenter)
        self.frameLabel_3.setPixmap(pixmap)

    def update_video_frame_4(self, frame):
        """在4号视频流框中显示OpenCV图像"""
        if frame is None or frame.size == 0:
            return

        # 将OpenCV图像转换为Qt格式
        if len(frame.shape) == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            height, width = frame.shape[:2]
            pixmap = QImage(frame, width, height, QImage.Format_RGB888)
        elif len(frame.shape) == 2:  # 灰度图，单通道
            height, width = frame.shape[:2]
            pixmap = QImage(frame, width, height, QImage.Format_Grayscale8)
        else:
            print("未知图像格式")
            return
        
        pixmap = QPixmap.fromImage(pixmap)
        # 获取视频流和label窗口的长宽比值的最大值，适应label窗口播放，不然显示不全
        ratio = max(width / self.frameLabel_4.width(), height / self.frameLabel_4.height())
        pixmap.setDevicePixelRatio(ratio)
        # 视频流置于label中间部分播放
        self.frameLabel_4.setAlignment(Qt.AlignCenter)
        self.frameLabel_4.setPixmap(pixmap)

    def update_joint_positions(self, clk1, clk2, clk3, clk4):
        """更新读取到的关节位置"""
        self.entryR_j1Clk.setText(f"{clk1}")
        self.entryR_j2Clk.setText(f"{clk2}")
        self.entryR_j3Clk.setText(f"{clk3}")
        self.entryR_j4Clk.setText(f"{clk4}")