import pigpio
import time
import os
import cv2
import numpy as np


show = True
# HSV参数
lower = np.array([0, 0, 120])
upper = np.array([105, 85, 255])
# 扫描参数
y_scan = 150
x_middle = 140
wide_scan = 5
wide_need = 2

# 人行道配置
sidewalk = True
sidewalk_len = 50
motor_before_speed = 12000
motor_stop_pwm = 8000

# 初始化pwm输出
motor_pin = 20
motor_speed = 13300  # 最低可动11500

servo_pin = 21
servo_middle = 68  # 经验证中间值是68
servo_min = 60  # 最右边占空比(千分之)
servo_max = 72  # 最左边占空比(千分之)

pid_left = 0.05  # 左转是pwm增加方向
pid_right = 0.10

pi = pigpio.pi()
pi.set_mode(motor_pin, pigpio.OUTPUT)
pi.set_PWM_frequency(motor_pin, 200)
pi.set_PWM_range(motor_pin, 40000)
pi.set_PWM_dutycycle(motor_pin, 10000)
time.sleep(1)

pi2 = pigpio.pi()
pi2.set_mode(servo_pin, pigpio.OUTPUT)
pi2.set_PWM_frequency(servo_pin, 50)
pi2.set_PWM_range(servo_pin, 1000)
pi2.set_PWM_dutycycle(servo_pin, servo_middle)
time.sleep(2)

cap = cv2.VideoCapture(-1)
while True:
    try:
        # RUN!!!
        ret, frame = cap.read()
        if frame is None:
            cap.release()
            pi.set_PWM_dutycycle(motor_pin, 10000)
            cap = cv2.VideoCapture(-1)
            ret, frame = cap.read()
            continue
        pi.set_PWM_dutycycle(motor_pin, motor_speed)
        frame = cv2.resize(frame, (320, 180))
        # HSV+inRange
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_hsv = cv2.inRange(frame_hsv, lower, upper)
        # OPEN
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        # frame_hsv = cv2.morphologyEx(frame_hsv, cv2.MORPH_OPEN, kernel)
        # scan
        y_line = frame_hsv[y_scan]
        left_line_x = x_middle
        right_line_x = x_middle
        # left_scaner
        while left_line_x > wide_scan + 1:
            wide_range = y_line[left_line_x - wide_scan:left_line_x]
            if list(wide_range).count(255) > wide_need:
                break
            left_line_x = left_line_x - 1
        while right_line_x < 320 - wide_scan - 1:
            wide_range = y_line[right_line_x:right_line_x + wide_scan]
            if list(wide_range).count(255) > wide_need:
                break
            right_line_x = right_line_x + 1
        loss = (right_line_x + left_line_x) / 2 - x_middle
        print(loss)

        if loss > 0:  # 右转情况
            pi2.set_PWM_dutycycle(servo_pin, servo_middle - pid_right * loss)
        else:  # 左转loss<=0,pwm++
            pi2.set_PWM_dutycycle(servo_pin, servo_middle - pid_left * loss)
        
        if sidewalk:
            pi.set_PWM_dutycycle(motor_pin, motor_before_speed)
        else:
            pi.set_PWM_dutycycle(motor_pin, motor_speed)
        print(sidewalk)
        # 人行道检测
        if sidewalk:
            white_count = right_line_x - left_line_x
            print(white_count)
            if white_count<sidewalk_len:
                sidewalk = False
                pi.set_PWM_dutycycle(motor_pin, motor_stop_pwm)
                time.sleep(3)

        # show
        if show:
            cv2.line(frame, (left_line_x, y_scan), (right_line_x, y_scan), (0, 255, 0), 5)
            cv2.imshow("frame", frame)
            cv2.imshow("frame_hsv", frame_hsv)
            if cv2.waitKey(1) == 27:
                break
        
    except KeyboardInterrupt:
        break
# 停止
# os.system("sudo killall pigpiod")
pi.set_PWM_dutycycle(motor_pin, 10000)
