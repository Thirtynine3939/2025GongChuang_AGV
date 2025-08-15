import sys
import time
import threading
import cv2
import numpy as np
from PySide6.QtWidgets import QApplication
from presenter import AppPresenter
from task import TaskProcessor

# ================================== #

# 默认运行环境为linux，若想在win运行，请去model.py文件中将platform改为0

# win端运行需要串口com1和com3，可以使用相关软件配置虚拟串口后再运行此程序。

# ================================== #

# 全局状态
exit_program = False

def main():
    global exit_program
    
    # 创建Qt应用（必须在主线程）
    app = QApplication(sys.argv)

    # 初始化Presenter
    presenter = AppPresenter()

    # 初始化task
    task_processor = TaskProcessor(presenter)

    # 启动视频处理线程
    presenter.vision.start()
    presenter.add_log("视频处理线程已启动")

    # 启动机械臂插补线程
    presenter.arm.start()
    presenter.add_log("机械臂插补线程已启动")

    # 启动后台任务线程
    task_processor.start_background_tasks()
    presenter.add_log("后台任务线程已启动")

    # 启动主任务线程
    task_processor.start_main_tasks()
    presenter.add_log("主任务线程已启动")

    # 显示主窗口
    presenter.view.show()

    # 设置退出处理
    def on_main_window_closing():
        global exit_program
        exit_program = True

        # 停止主任务线程
        task_processor.stop_main_tasks()

        # 停止后台任务线程
        task_processor.stop_background_tasks()

        # 清理资源
        presenter.shutdown()
        
        # 关闭所有OpenCV窗口
        cv2.destroyAllWindows()

        print("程序已退出")
        
        # 退出应用程序
        app.quit()

    # 连接主窗口关闭信号到退出处理函数
    presenter.view.window_closing_signal.connect(on_main_window_closing)

    # 启动Qt事件循环
    sys.exit(app.exec())


if __name__ == "__main__":
    main()