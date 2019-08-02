import cv2
import numpy as np
import serial

ser = serial.Serial("/dev/ttyAMA0", 115200)

class Ball_detect():
    def __init__(self):
        self.point_flag = False
        self.ban_flag = False
        self.x_ball = 0
        self.y_ball = 0


    def detect(self, frame, contours):
        # 板的长度和面积的最大值和最小值
        BanarclengthMin = 1000
        BanarclengthMax = 2000
        BansquareMin = 60000
        BansquareMax = 150000

        # 球的长度和面积的最大值和最小值
        BallarclengthMin = 50
        BallarclengthMax = 120
        BallsquareMin = 350
        BallsquareMax = 1000

        x_ball = 0
        y_ball = 0

        # print(len(contours))
        for cnt in range(len(contours)):
            # cv2.drawContours(frame, contours, cnt, (0, 255, 0), 2)
            square = cv2.contourArea(contours[cnt])  # 面积
            arcLength = cv2.arcLength(contours[cnt], True)  # 长度
            # print(square, arcLength)
            # 板定位
            if BanarclengthMin < arcLength < BanarclengthMax and BansquareMin < square < BansquareMax:
                approx = cv2.approxPolyDP(contours[cnt], arcLength * 0.02, 1)
                cv2.polylines(frame, [approx], True, (0, 0, 255), 2)  # 拟合
                mm = cv2.moments(approx)  # 中心求解
                cx = int(mm['m10'] / mm['m00'])
                cy = int(mm['m01'] / mm['m00'])
                cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)
                # print("板的中心坐标是 (%d,%d)" % (np.int(cx), np.int(cy)))
                # cv2.drawContours(image, contours, cnt, (0, 255, 0), 2)
                self.ban_flag = True

            # 球定位
            elif BallarclengthMin < arcLength < BallarclengthMax and BallsquareMin < square < BallsquareMax:
                # cv2.drawContours(frame, contours, cnt, (0, 255, 0), 2)
                epsilon = 0.01 * arcLength  # 轮廓逼近，主要是对闭合轮廓进行检测，即水管的漏油点
                approx = cv2.approxPolyDP(contours[cnt], epsilon, True)  # 进行图像拟合
                rrt = cv2.fitEllipse(contours[cnt])  # 椭圆拟合
                cv2.ellipse(frame, rrt, (255, 0, 0), 2, cv2.LINE_AA)  # 画出拟合椭圆
                x, y = rrt[0]  # 求出椭圆中心
                # print("球的坐标是 (%d,%d)" % (np.int(x), np.int(y)))
                cv2.circle(frame, (np.int(x), np.int(y)), 4, (255, 0, 0), -1, 8, 0)  # 画出椭圆中心
                self.point_flag = True
                x_ball = int(-2.381 * x + 1013)
                y_ball = int(-2.15 * y + 744.2)
                # print("球的坐标是 (%d,%d)" % (x_ball, y_ball))
        if self.ban_flag == True and self.point_flag == True and 0 < x_ball < 650 and 0 < y_ball < 650:
            # print("------------------  Successful positioning!  ------------------")
            # print("球的坐标是 (%d,%d)" % (x_ball, y_ball))
            self.ban_flag = False
            self.point_flag_flag = False
            return [x_ball, y_ball]
        else:
            # print("#################  Unsuccessful positioning!  #################")
            return [0]

    def cameraAnalysis(self, frame):
        frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_CUBIC)  # 因为摄像头问题，对图像进行了大小修改
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 转为灰度图
        # cv2.imshow("gray", gray)
        ret, binary = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY)  # 二值化
        # cv2.imshow("binary", binary)
        blurred = cv2.GaussianBlur(binary, (3, 3), 0)  # 高斯滤波处理
        # cv2.imshow("blur", blurred)                                               # 显示滤波图像
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))  # 建立7 * 7的卷积核
        closed = cv2.morphologyEx(blurred, cv2.MORPH_RECT, kernel)  # 去除噪点
        # cv2.imshow("closed", closed)
        ret, binary = cv2.threshold(closed, 30, 255, cv2.THRESH_BINARY)  # 再次二值化
        # cv2.imshow("binary", binary)
        image, contours, hierarchy = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)  # 寻找轮廓
        if len(contours) != 0:
            ball_position = self.detect(frame, contours)
            cv2.imshow("frame", frame)
            # print(ball_position)
            return ball_position
        else:
            return [0]


if __name__ == "__main__":
    cameraCapture  = cv2.VideoCapture(0)
    gunqiu = Ball_detect()
    cnt = 0
    while True:
        cnt = cnt + 1               # 消除开启摄像头时初始化的干扰
        ret, frame = cameraCapture.read()
        position = gunqiu.cameraAnalysis(frame)
        if cnt > 30:
            if len(position) != 1:
                print("------------------  Successful positioning!  ------------------")
                print(position)
                check_communication = "py{}{}".format(position[0], position[1])
                print(check_communication)
                ser.write(check_communication.encode("utf-8"))
            else:
                print("#################  Unsuccessful positioning!  #################")
                check_communication = "pn000000"
                ser.write(check_communication.encode("utf-8"))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cameraCapture.release()
    cv2.destroyAllWindows()
