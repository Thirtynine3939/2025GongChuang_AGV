class PositionalPID:
    """
    位置式PID控制器
    """
    def __init__(self, kp: float, ki: float, kd: float):
        """
        初始化位置式PID控制器
        :param kp: 比例系数
        :param ki: 积分系数
        :param kd: 微分系数
        """
        self.kp = kp  # 比例系数
        self.ki = ki  # 积分系数
        self.kd = kd  # 微分系数

        # 误差值存储
        self.error_sum = 0.0  # 累积误差
        self.last_error = 0.0  # 上次误差

    def update_params(self, kp: float = None, ki: float = None, kd: float = None):
        """
        更新PID参数
        :param kp: 新的比例系数（可选）
        :param ki: 新的积分系数（可选）
        :param kd: 新的微分系数（可选）
        """
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        print(f"PID参数更新: kp={self.kp}, ki={self.ki}, kd={self.kd}")

    def reset(self):
        """
        清零累计误差和上次误差
        """
        self.error_sum = 0.0
        self.last_error = 0.0
        print("PID误差数据已清零")

    def compute(self, target: float, feedback: float) -> float:
        """
        计算位置式PID输出
        :param target: 目标值
        :param feedback: 当前反馈值
        :return: PID输出值
        """
        # 计算当前误差
        error = feedback - target

        # 累积误差（积分项）
        self.error_sum += error

        # 积分限幅
        if self.error_sum > 5000.0:
            self.error_sum = 5000.0
        elif self.error_sum < -5000.0:
            self.error_sum = -5000.0

        # 计算PID输出
        output = (
            self.kp * error +
            self.ki * self.error_sum +
            self.kd * (error - self.last_error)
        )

        # 更新上次误差
        self.last_error = error

        if output > 2000.0:
            output = 2000.0
        elif output < -2000.0:
            output = -2000.0
        
        if -1.5 <= error <= 1.5:
            return 0
        else:
            return output

# 示例代码：
if __name__ == "__main__":
    pid1 = PositionalPID(kp=1.0, ki=0.1, kd=0.05)
    pid2 = PositionalPID(kp=0.8, ki=0.05, kd=0.02)

    target_values = {"pid1": 10.0, "pid2": 15.0}
    feedback_values = {"pid1": 8.0, "pid2": 12.0}

    for i in range(5):
        # 计算PID1输出
        output1 = pid1.compute(target_values["pid1"], feedback_values["pid1"])
        feedback_values["pid1"] += output1
        print(f"Step {i+1}, PID1 Output: {output1:.4f}, Feedback1: {feedback_values['pid1']:.4f}")

        # 计算PID2输出
        output2 = pid2.compute(target_values["pid2"], feedback_values["pid2"])
        feedback_values["pid2"] += output2
        print(f"Step {i+1}, PID2 Output: {output2:.4f}, Feedback2: {feedback_values['pid2']:.4f}")

        # 模拟实时更新PID参数
        if i == 2:  # 在第三步更新PID参数
            pid1.update_params(kp=1.5, ki=0.2, kd=0.1)
            pid2.update_params(kp=1.0, ki=0.1, kd=0.05)
