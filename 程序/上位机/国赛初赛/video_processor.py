import cv2
import numpy as np
import time
import queue
from queue import Queue
import threading
import math
from pyzbar.pyzbar import decode # 二维码检测
from stm32 import STM32Controller
from robocal import RoboArm
from model import GLOBAL_MODEL as gMODEL
from HMI.hmi_view import MainWindow

class VideoProcessor:
    """视频处理类，封装物料检测、二维码识别等功能"""

    def __init__(self, STM32: STM32Controller, arm: RoboArm, view: MainWindow,
                 frame_queue_size=30, camera_index=2, qr_camera_index=0):
        """
        初始化视频处理器
        
        :param frame_queue_size: 帧队列大小 (默认为30)
        :param camera_index: 主摄像头索引 (默认为2)
        :param qr_camera_index: 扫码摄像头索引 (默认为0)
        """

        self.STM32 = STM32
        self.arm = arm
        self.view = view

        # 配置参数
        self.frame_queue = Queue(maxsize=frame_queue_size)
        self.camera_index = camera_index
        self.qr_camera_index = qr_camera_index
        
        # 状态标志
        self.exit_flag = False
        self.running = False
        self.thread = None
        
        # 颜色阈值参数
        self.lower_red1 = np.array([0, 130, 130])   # 红色的下界
        self.upper_red1 = np.array([10, 255, 255])  # 红色的上界
        self.lower_red2 = np.array([150, 30, 70])   # 红色的下界（另一种红色范围）
        self.upper_red2 = np.array([180, 255, 255]) # 红色的上界（另一种红色范围）

        self.lower_green0 = np.array([40, 50, 50])  # 绿色（用于物料）
        self.upper_green0 = np.array([81, 255, 255])
        self.lower_green1 = np.array([40, 30, 50])  # 绿色（用于色环）
        self.upper_green1 = np.array([90, 255, 255])

        self.lower_blue = np.array([100, 100, 60])  # 蓝色
        self.upper_blue = np.array([132, 255, 255])
        
        # 形态学处理核
        self.kernel = np.ones((5, 5), np.uint8)
        self.min_contour_area = 2000
        
        # 位置跟踪
        self.relative_x, self.relative_y = 0, 0
        self.distance = 0.0
        self.last_valid_x = 0.0
        self.last_valid_y = 0.0
        self.last_detected_time = None
        
        # 检测模式
        self.choose_colour = 1
        self.Detect_Mode = 0  # 0:检测物料  1:检测色环
        self.poscali_enable_flag = 0

        # 色环相关
        self.mask_circle_area = None
        self.mask_circle_area_show = 0

        
        # 二维码相关
        self.QR_Camera_En = True
        self.QR_Read_Flag_Start = False
        self.QR_Read_Flag_Success = False
        self.QR_Read_Flag_Timeout = False
        self.QR_DATA = '待读取'
        # if self.QR_Camera_En:
        #     self.QR_DATA = '待读取'
        self.last_QR_DATA = self.QR_DATA
        self.read_colour_order = [[1, 2, 3], [1, 2, 3]]
        
        # 准心位置
        self.center_x, self.center_y = 306, 233
        self.blueCircleDelta_x = -1  # 蓝色色环补偿
        self.blueCircleDelta_y = 2  # 蓝色色环补偿
        
        # QR 检测器
        self.qr_detector = cv2.QRCodeDetector()
        
        # 缓存旋转矩阵
        self.rotate_cache = {"alpha": None, "matrix": None}

        # FPS计算相关变量
        self.FPS = 0
        self.frame_count = 0
        self.start_time = 0

        # 是否启用show
        self.show = False

    def set_centerX(self, value):
        """设置画面中心X坐标"""
        self.center_x = value

    def set_centerY(self, value):
        """设置画面中心Y坐标"""
        self.center_y = value

    def start(self):
        """启动视频处理线程"""
        if self.running:
            return
            
        self.running = True
        self.exit_flag = False
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def stop(self):
        """停止视频处理"""
        self.running = False
        self.exit_flag = True
        # 只有不是当前线程时才 join
        if self.thread and self.thread.is_alive() and threading.current_thread() != self.thread:
            self.thread.join(timeout=1.0)

    def start_QR_Read(self):
        """启动二维码检测"""
        self.QR_Read_Flag_Start = True

    def reboot(self):
        """重启视频处理器"""
        print("test")
        self.stop()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.start()
        print("视频处理器已重启")
        self.view.add_log_message("视频处理器已重启")

    def run(self):
        """线程主函数"""
        # 打开摄像头
        cap = cv2.VideoCapture(self.camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not cap.isOpened():
            print(f"无法打开检测用摄像头")
            return
        print(f"检测用摄像头已启动")
        # self.view.add_log_message("检测用摄像头已启动")
        
        if self.QR_Camera_En:
            cap_scan = cv2.VideoCapture(self.qr_camera_index)
            cap_scan.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap_scan.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

            if not cap_scan.isOpened():
                print(f"无法打开扫码用摄像头")
                return
            print(f"扫码用摄像头已启动")
            # self.view.add_log_message("扫码用摄像头已启动")
            gMODEL.QRdata.set("准备出发")

        # 初始化FPS计算
        self.start_time = time.time()
        self.frame_count = 0
        
        # QR检测状态
        detecting = False
        start_time_QR_Read = 0

        # 串口发送频率控制参数
        prev_send_time = time.time()
        send_interval = 0.01  # 发送数据的时间间隔（秒）

        # 初始化成功参数，用于重启线程
        self.QR_Read_Flag_Success = False
        
        while not self.exit_flag:
            # 更新FPS计数
            self.frame_count += 1
            
            # 计算FPS (每秒更新一次)
            curr_time = time.time()
            elapsed_time = curr_time - self.start_time
            if elapsed_time > 1.0:  # 每1秒更新一次FPS
                self.FPS = self.frame_count / elapsed_time
                self.frame_count = 0
                self.start_time = curr_time
            
            # 读取摄像头帧
            ret, frame = cap.read()
            if not ret:
                print("无法读取主摄像头数据")
                continue
            
            # QR检测处理
            if self.QR_Camera_En and not self.QR_Read_Flag_Success:
                ret_scan, FrameScan = cap_scan.read()
                if not ret_scan:
                    print("无法读取扫码摄像头数据")
                    continue
                
                if self.QR_Read_Flag_Start:
                    self.QR_Read_Flag_Start = False
                    detecting = True
                    start_time_QR_Read = time.time()
                    print("开始检测二维码，请将二维码对准摄像头...")
                    self.QR_DATA = '开始读取'
                    gMODEL.QRdata.set(self.QR_DATA) # 更新二维码数据UI显示
                
                if detecting:
                    elapsed = time.time() - start_time_QR_Read
                    if elapsed > 3:  # 超时处理
                        print("检测超时")
                        self.QR_DATA = '超时'
                        gMODEL.QRdata.set(self.QR_DATA) # 更新二维码数据UI显示
                        detecting = False
                        self.QR_Read_Flag_Timeout = True
                    else:
                        success, data = self.process_qr_code(FrameScan)
                        if success:
                            print("二维码内容:", data)
                            self.QR_DATA = data
                            gMODEL.QRdata.set(self.QR_DATA) # 更新二维码数据UI显示
                            self.read_colour_order = [[int(c) for c in part] for part in data.split('+')]
                            print(self.read_colour_order)
                            self.QR_Read_Flag_Success = True
                            cap_scan.release()
                            print("扫码摄像头已关闭")
                            detecting = False
            
            # 物料/色环检测
            mask, frame = self.process_main_frame(frame, curr_time)
            
            # 串口发送
            if curr_time - prev_send_time >= send_interval and self.poscali_enable_flag:
                self.STM32.Circle_Centre_Send(self.relative_x, self.relative_y)
                prev_send_time = curr_time
            
            # 将帧放入队列
            if self.QR_Camera_En and not self.QR_Read_Flag_Success:
                self.put_frame_to_queue(frame, mask, FrameScan)
            else:
                self.put_frame_to_queue(frame, mask)

            # 将处理后的帧显示在主界面
            if self.show:
                self.put_frame_to_show(frame, mask)
            
        # 释放资源
        cap.release()
        if self.QR_Camera_En and not self.QR_Read_Flag_Success:
            cap_scan.release()

    def process_qr_code(self, frame):
        """处理二维码检测"""
        # 先尝试OpenCV快速识别
        isSuccess, data = False, None
        data, bbox, _ = self.qr_detector.detectAndDecode(frame)
        if data:
            return True, data
        
        # 如果快速识别失败，使用更复杂的处理方法
        qr_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thd = cv2.threshold(qr_gray, 50, 255, cv2.THRESH_BINARY_INV)
        kernel_QR = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 21))
        closed = cv2.morphologyEx(thd, cv2.MORPH_CLOSE, kernel_QR)
        
        contours, _ = cv2.findContours(closed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, None
        
        # 按轮廓面积降序排列
        sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
        
        for contour in sorted_contours:
            # 忽略过小轮廓
            if cv2.contourArea(contour) < frame.shape[0] * frame.shape[1] * 0.01:
                continue
            
            # 计算凸包
            hull = cv2.convexHull(contour)
            
            # 多边形逼近
            epsilon = 0.02 * cv2.arcLength(hull, True)
            approx = cv2.approxPolyDP(hull, epsilon, True)
            
            if len(approx) == 4:
                # 顶点排序处理
                vertices = np.array([np.squeeze(p) for p in approx])
                try:
                    ori_points = np.float32(self.sort_vertices_clockwise(vertices))
                except:
                    continue
                
                # 透视变换
                dst_points = np.float32([[25, 25], [475, 25], [475, 475], [25, 475]])
                M = cv2.getPerspectiveTransform(ori_points, dst_points)
                transformed = cv2.warpPerspective(qr_gray, M, (500, 500))
                
                # 解码处理
                barcodes = decode(transformed) # 使用zbar识别二维码
                if barcodes:
                    for barcode in barcodes:
                        return True, barcode.data.decode("utf-8")
        
        return False, None
    
    def process_main_frame(self, frame, curr_time):
        """处理主摄像头帧"""
        if self.Detect_Mode == 0:  # 物料检测
            mask = self.hsv_binary_one(frame, self.choose_colour, 0)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
            self.detect_material(frame, mask, curr_time)
        elif self.Detect_Mode == 1:  # 色环检测
            mask = self.hsv_binary_one(frame, self.choose_colour, 1)
            self.detect_color_circle(frame, curr_time)
        
        # 绘制准心
        self.draw_crosshair(frame)
        
        # 显示信息
        self.display_info(frame, self.FPS)
        
        return mask, frame
    
    def detect_material(self, frame, mask, curr_time):
        """检测物料"""
        # 寻找最大轮廓的质心     ################################################################待研究：圆度
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cx, cy = None, None
        
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(max_contour)
            
            if area > self.min_contour_area:
                M = cv2.moments(max_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    
                    # 计算相对坐标
                    relative_x_unrotated = cx - self.center_x
                    relative_y_unrotated = -(cy - self.center_y)
                    
                    # 旋转点
                    point_rotated = self.rotate_point(relative_x_unrotated, relative_y_unrotated, -self.arm.Alpha)

                    self.relative_x = point_rotated[0]
                    self.relative_y = point_rotated[1]
                    
                    # 更新有效坐标
                    self.last_valid_x = self.relative_x
                    self.last_valid_y = self.relative_y
                    self.last_detected_time = curr_time
        
        # 未检测到质心时的处理
        if cx is None:
            self.handle_no_detection(curr_time)

    def detect_color_circle(self, frame, curr_time):
        """检测色环"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gauss = cv2.GaussianBlur(gray, (3, 3), 0)
        
        # 检测圆形
        # minDist:两圆最小距离（同心圆无效） param1: Canny参数  param2: 圆可信度  minRadius: 最小半径  maxRadius: 最大半径
        circles = cv2.HoughCircles(
            gauss, cv2.HOUGH_GRADIENT_ALT, 0.5, minDist=100, 
            param1=100, param2=0.9, minRadius=20, maxRadius=180
        )
        
        color_circle_target = []
        
        if circles is not None:
            circles = circles.reshape((-1, 3))
            circles = self.delete_superfluous_circle(circles)
            squares = self.circle_square(frame, circles)
            
            if squares:
                for square in squares:
                    if self.is_color(square[0], self.choose_colour):
                        target_x = square[1][0]
                        target_y = square[1][1]
                        color_circle_target.append([self.choose_colour, target_x, target_y])
                
                if color_circle_target:
                    # 选择Y值最小的圆
                    max_y = max(item[2] for item in color_circle_target)
                    result = next(item for item in color_circle_target if item[2] == max_y)
                    if self.choose_colour == 3:  # 蓝色色环
                        target_x = result[1] + self.blueCircleDelta_x
                        target_y = result[2] + self.blueCircleDelta_y
                    else:
                        target_x = result[1]
                        target_y = result[2]
                    cv2.circle(frame, (int(target_x), int(target_y)), 4, (0, 0, 255), -1)
                    
                    # 计算相对坐标
                    relative_x_unrotated = target_x - self.center_x
                    relative_y_unrotated = -(target_y - self.center_y)
                    
                    # 旋转点
                    point_rotated = self.rotate_point(relative_x_unrotated, relative_y_unrotated, -self.arm.Alpha)

                    self.relative_x = point_rotated[0]
                    self.relative_y = point_rotated[1]
                    
                    # 更新有效坐标
                    self.last_valid_x = self.relative_x
                    self.last_valid_y = self.relative_y
                    self.last_detected_time = curr_time
        
        # 未检测到圆形时的处理
        if not color_circle_target:
            self.handle_no_detection(curr_time)

    def hsv_binary_one(self, img, color, sta, erode_dilate=False):
        """根据颜色创建HSV二值掩码"""
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        if color == 1:  # 红色
            mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
        elif color == 2:  # 绿色
            mask = cv2.inRange(hsv, 
                              self.lower_green0 if sta == 0 else self.lower_green1,
                              self.upper_green0 if sta == 0 else self.upper_green1)
        elif color == 3:  # 蓝色
            mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        
        if erode_dilate:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        
        return mask
    
    def is_color(self, img, color, min_area=150):
        """检测图像中是否包含指定颜色"""
        mask = self.hsv_binary_one(img, color, 1)
        self.mask_circle_area = cv2.countNonZero(mask)
        if self.mask_circle_area != 0:
            self.mask_circle_area_show = self.mask_circle_area
        return self.mask_circle_area > min_area
    
    def rotate_point(self, x, y, rotate_alpha):
        """旋转点坐标"""
        if self.rotate_cache["alpha"] != rotate_alpha:
            self.rotate_cache["matrix"] = np.array([
                [np.cos(rotate_alpha), -np.sin(rotate_alpha)],
                [np.sin(rotate_alpha),  np.cos(rotate_alpha)]
            ])
            self.rotate_cache["alpha"] = rotate_alpha
        
        rotated_point = np.dot(self.rotate_cache["matrix"], np.array([x, y]))
        return rotated_point

    def handle_no_detection(self, curr_time):
        """处理未检测到目标的情况"""
        if self.last_detected_time and (curr_time - self.last_detected_time <= 1.0): # 超时1秒
            self.relative_x = self.last_valid_x
            self.relative_y = self.last_valid_y
        else:
            self.relative_x = 0.0
            self.relative_y = 0.0

    def draw_crosshair(self, frame):
        """绘制旋转准心"""
        horizontal_line = np.array([[self.center_x-50, self.center_y], [self.center_x+50, self.center_y]])  # 水平线
        vertical_line = np.array([[self.center_x, self.center_y-50], [self.center_x, self.center_y+50]])    # 垂直线
        
        # 创建旋转矩阵
        M = cv2.getRotationMatrix2D((self.center_x, self.center_y), self.arm.Alpha0, 1.0)

        # 旋转并绘制十字准心
        rotated_horizontal_line = cv2.transform(np.array([horizontal_line]), M)[0]
        rotated_vertical_line = cv2.transform(np.array([vertical_line]), M)[0]
        
        cv2.line(frame, tuple(rotated_horizontal_line[0]), tuple(rotated_horizontal_line[1]), (0, 255, 0), 2)
        cv2.line(frame, tuple(rotated_vertical_line[0]), tuple(rotated_vertical_line[1]), (0, 255, 0), 2)

    def display_info(self, frame, fps):
        """在画面上显示信息"""
        # 显示相对坐标
        cv2.putText(frame, f'Relative X: {self.relative_x:.2f}', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, f'Relative Y: {self.relative_y:.2f}', (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        
        # 显示中心坐标
        cv2.putText(frame, f'Center X: {self.center_x}', (280, 390), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, f'Center Y: {self.center_y}', (280, 435), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2, cv2.LINE_AA)

        # 显示距离
        self.distance = math.sqrt(self.relative_x**2 + self.relative_y**2)
        cv2.putText(frame, f'Distance: {self.distance:.2f}', (10, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        
        # 显示FPS
        cv2.putText(frame, f'FPS: {fps:.2f}', (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        
        # 显示色环面积
        if self.mask_circle_area_show is not None:
            cv2.putText(frame, f'circle_area: {self.mask_circle_area_show:.2f}', (10, 140), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
            
    def put_frame_to_show(self, frame, mask):
        cv2.imshow("Processed Frame", frame)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # 按'q'退出
            self.stop()

    def put_frame_to_queue(self, frame, mask, qr_frame=None):
        """将处理后的帧放入队列"""
        frame_data = {
            "frame": frame,
            "mask": mask,
            "qr_frame": qr_frame,
            "relative_x": self.relative_x,
            "relative_y": self.relative_y,
        }
        
        # 放入队列（非阻塞方式）
        try:
            self.frame_queue.put_nowait(frame_data)
        except queue.Full:
            # 队列满时丢弃最旧的一帧，放入新帧
            try:
                self.frame_queue.get_nowait()  # 丢弃一帧
                self.frame_queue.put_nowait(frame_data)
            except queue.Empty:
                pass

    def get_frame(self):
        """从队列获取一帧图像（非阻塞）"""
        try:
            return self.frame_queue.get_nowait()
        except queue.Empty:
            return None
        
    # ================== 辅助函数 ==================
    
    @staticmethod
    def sort_vertices_clockwise(vertices):
        """按顺时针方向排序顶点"""
        center = np.mean(vertices, axis=0)
        vectors = vertices - center
        angles = np.arctan2(vectors[:, 1], vectors[:, 0])
        sorted_indices = np.argsort(angles)
        return vertices[sorted_indices]
    
    @staticmethod
    def delete_superfluous_circle(circles):
        """过滤出同心圆的最大圆"""
        max_concentric_circles = []
        if circles is not None:
            while circles.shape[0] != 0:
                # 寻找与第一个圆同心的其他圆
                concentric_circles_index = np.where(
                    (circles[:, 0] == circles[0, 0]) & 
                    (circles[:, 1] == circles[0, 1])
                )
                concentric_circles = circles[concentric_circles_index, :]
                # 最大半径的索引
                max_concentric_circle_index = np.argmax(concentric_circles[0, :, 2])
                max_concentric_circles.append(concentric_circles[0, max_concentric_circle_index, :])
                # 在原数组中删除其他圆
                circles = np.delete(circles, concentric_circles_index, 0)
        return np.array(max_concentric_circles)
    
    @staticmethod
    def circle_square(img, circles):
        """获取圆形区域的方形ROI"""
        squares = []
        if circles is not None:
            for circle in circles:
                x, y, r = circle
                x1, y1 = max(0, int(x - r)), max(0, int(y - r))
                x2, y2 = min(img.shape[1], int(x + r)), min(img.shape[0], int(y + r))
                
                try:
                    roi = img[y1:y2, x1:x2]
                    squares.append([roi, (x, y, r)])
                except:
                    continue
        return squares
    

if __name__ == "__main__":
    # 示例用法
    STM32 = STM32Controller()  
    arm = RoboArm()

    processor = VideoProcessor(STM32, arm, 30, 0, 1)
    processor.show = True  # 启用显示
    processor.start()
    
    try:
        while processor.running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        processor.stop()