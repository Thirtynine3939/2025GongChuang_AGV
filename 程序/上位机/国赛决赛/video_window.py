from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PySide6.QtCore import Qt
from PySide6.QtGui import QPixmap, QImage
import cv2
import numpy as np

class VideoWindow(QWidget):
    """独立的视频显示窗口"""
    
    def __init__(self, window_title="Video", parent=None):
        super().__init__(parent)
        self.setWindowTitle(window_title)
        
        # 设置默认窗口大小为480x360，并允许调整大小
        self.resize(480, 360)
        self.setMinimumSize(240, 180)  # 设置最小尺寸，保持4:3比例
        
        # 设置窗口属性
        self.setWindowFlags(Qt.Window | Qt.WindowStaysOnTopHint)
        
        # 创建布局和控件
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("border: 1px solid gray;")
        self.video_label.setScaledContents(True)  # 允许内容缩放
        self.video_label.setMinimumSize(240, 180)
        
        layout.addWidget(self.video_label)
        self.setLayout(layout)
        
        # 设置默认图像
        self.set_default_image()
    
    def set_default_image(self):
        """设置默认显示图像"""
        # 创建一个黑色图像，尺寸为360x480（4:3比例）
        default_image = np.zeros((360, 480, 3), dtype=np.uint8)
        cv2.putText(default_image, "No Video Signal", (120, 180), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        self.update_frame(default_image)
    
    def update_frame(self, frame):
        """更新显示的帧"""
        if frame is None:
            self.set_default_image()
            return
            
        # 确保frame是numpy数组
        if not isinstance(frame, np.ndarray):
            return
            
        # 获取当前视频标签的大小
        label_size = self.video_label.size()
        target_width = label_size.width()
        target_height = label_size.height()
        
        # 如果标签太小，使用默认尺寸
        if target_width < 100 or target_height < 100:
            target_width = 480
            target_height = 360
            
        # 调整图像大小以适应标签
        if frame.shape[:2] != (target_height, target_width):
            frame = cv2.resize(frame, (target_width, target_height))
        
        # 转换颜色格式 (BGR -> RGB)
        if len(frame.shape) == 3 and frame.shape[2] == 3:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        else:
            # 如果是灰度图像，转换为RGB
            if len(frame.shape) == 2:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
            else:
                frame_rgb = frame
        
        # 转换为QImage
        height, width, channel = frame_rgb.shape
        bytes_per_line = 3 * width
        q_image = QImage(frame_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)
        
        # 转换为QPixmap并显示
        pixmap = QPixmap.fromImage(q_image)
        self.video_label.setPixmap(pixmap)
    
    def resizeEvent(self, event):
        """窗口大小改变事件，保持4:3比例"""
        super().resizeEvent(event)
        
        # 获取当前窗口大小
        current_size = event.size()
        width = current_size.width()
        height = current_size.height()
        
        # 计算4:3比例下的新尺寸
        target_ratio = 4.0 / 3.0
        current_ratio = width / height
        
        if abs(current_ratio - target_ratio) > 0.01:  # 如果比例偏差超过0.01
            if current_ratio > target_ratio:
                # 宽度过大，按高度调整宽度
                new_width = int(height * target_ratio)
                new_height = height
            else:
                # 高度过大，按宽度调整高度
                new_width = width
                new_height = int(width / target_ratio)
            
            # 限制最小尺寸
            if new_width < 240:
                new_width = 240
                new_height = 180
            elif new_height < 180:
                new_width = 240
                new_height = 180
            
            # 设置新的窗口大小
            self.resize(new_width, new_height)
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        # 可以在这里添加关闭时的清理代码
        event.accept()
