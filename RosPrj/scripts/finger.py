import numpy as np
import cv2
import mediapipe as mp
import math
import time
import utlis
import rospy
import embedded_robot_arm.msg as arm_msg

# 初始化MediaPipe手势识别
mp_hands = mp.solutions.hands
hands = mp.solutions.hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
    )
fps = 0
fps_update_interval = 0.5  # 更新FPS显示的间隔（秒）
last_update_time = time.time()
frame_count = 0
draw = mp.solutions.drawing_utils

x_index = 90
y_index = 90
y_bias = 100
z_index = 100
z_bias = 50

# 捕捉视频设备
video_capture = cv2.VideoCapture(0)

# 计算两点间的2D距离
def calculate_2d_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

if __name__ == '__main__':
    rospy.init_node('midfinger', anonymous=True)
    cmd_pub = rospy.Publisher('/robot_cmd', arm_msg.cmd, queue_size=10)
    rate = rospy.Rate(5)
    cmd_msg = arm_msg.cmd()
    cmd_msg.cmd = 'moveJ'
    cmd_msg.mode = 0
    cmd_msg.data = [0, 0, 0, 0, 0, 0]
    
    try:
        while True:
            # 读取视频帧
            _, img = video_capture.read()
            
            img = cv2.resize(img, (640, 480))

            color_image = img

            color_image = np.asanyarray(color_image)

            # 处理图像并识别手势
            img = cv2.flip(color_image, 1)
            height, width, _ = img.shape
            results = hands.process(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            if results.multi_hand_landmarks:
                for hand in results.multi_hand_landmarks:
                    thumb_tip = hand.landmark[mp_hands.HandLandmark.THUMB_TIP]
                    index_tip = hand.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                    thumb_x, thumb_y = int(thumb_tip.x * width), int(thumb_tip.y * height)
                    index_x, index_y = int(index_tip.x * width), int(index_tip.y * height)
                    distance_2d = calculate_2d_distance(thumb_x, thumb_y, index_x, index_y)
                    #计算指尖距离

                    midfing_tip = hand.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
                    midfing_dip = hand.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_DIP]
                    mtip_x, mtip_y = int(midfing_tip.x * width), int(midfing_tip.y * height)
                    mdip_x, mdip_y = int(midfing_dip.x * width), int(midfing_dip.y * height)
                    mlength=int(calculate_2d_distance(mtip_x, mtip_y,mdip_x, mdip_y))
                    # 计算食指第一节长度


                    if distance_2d <= 40:  # 设定的2D阈值
                        midpoint_x=int((thumb_x + index_x)/2)
                        midpoint_y=int((thumb_y + index_y) / 2)
                        cv2.putText(img, f'Midpoint 2D Coordinates: {midpoint_x} , {midpoint_y}', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.putText(img, f'Middle finger 2Dlength: {mlength}', (10, 70),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.circle(img, (thumb_x, thumb_y), 10, (255, 0, 0), -1)
                        cv2.circle(img, (index_x, index_y), 10, (0, 255, 0), -1)
                        cv2.line(img, (mtip_x, mtip_y), (mdip_x, mdip_y), (0, 255, 255), 5)

                        #归一
                        x=(midpoint_x/640-0.5)*2 * x_index
                        y=(480-midpoint_y)/480 * y_index + y_bias
                        
                        if mlength < 10:
                            mlength = 10
                        elif mlength > 60:
                            mlength = 60
                            
                        z = (mlength - 10) / 50 * z_index + z_bias
                        
                        cmd_msg.data = [z, -x , y, math.pi, 0, 0]
                        
                        cmd_pub.publish(cmd_msg)


                draw.draw_landmarks(img, hand, mp_hands.HAND_CONNECTIONS)
            current_time = time.time()
            frame_count += 1
            if current_time - last_update_time > fps_update_interval:
                fps = frame_count / (current_time - last_update_time)
                frame_count = 0
                last_update_time = current_time

            # 在图像左上角显示FPS
            cv2.putText(img, f'FPS: {fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('RealSense 3D Hands', img)


            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cv2.destroyAllWindows()



