#!/usr/bin/env python3
#-*- coding:utf-8 -*-
class Pidcal:
    def __init__(self):
        self.error_sum = 0
        self.error_old = 0
        self.x = 0

        # 초기 PID 값
        self.kp = 0.003
        self.ki = 0.000005
        self.kd = 0.005

        # Twiddle용 파라미터
        self.p = [self.kp, self.ki, self.kd]
        self.dp = [self.kp/10, self.ki/10, self.kd/10]

    def cal_error(self, setpoint=440):
        return abs(setpoint - self.x)

    def twiddle(self, setpoint=440):
        best_err = self.cal_error(setpoint)
        threshold = 1e-30

        while sum(self.dp) > threshold:
            for i in range(len(self.p)):
                self.p[i] += self.dp[i]
                err = self.cal_error(setpoint)

                if err < best_err:
                    best_err = err
                    self.dp[i] *= 1.1
                else:
                    self.p[i] -= 2 * self.dp[i]
                    err = self.cal_error(setpoint)

                    if err < best_err:
                        best_err = err
                        self.dp[i] *= 1.05
                    else:
                        self.p[i] += self.dp[i]
                        self.dp[i] *= 0.95

        # 튜닝된 값 저장
        self.kp, self.ki, self.kd = self.p
        print(f"Twiddle 완료 → kp: {self.kp:.6f}, ki: {self.ki:.6f}, kd: {self.kd:.6f}")


    # setpoint is the center and the x_current is where the car is
    # width = 640, so 320 is the center but 318 is more accurate in real
    def pid_control(self, x_current, setpoint=320): #change just curve_count
        error = setpoint - x_current
        self.error_sum += error
        d_error = error - self.error_old

        p = self.kp * error
        i = self.ki * self.error_sum
        d = self.kd * d_error

        self.error_old = error
        pid_output = p + i + d
        pid_output = max(min(pid_output, 1.0), -1.0)  # -1 ~ 1 사이로 제한
        return pid_output