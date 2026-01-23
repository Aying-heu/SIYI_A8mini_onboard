import cv2
from pyzbar.pyzbar import decode
import numpy as np

# 视频流地址（替换为你的实际地址）
video_url = "rtsp://192.168.1.25:8554/main.264"
cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)
import time
idx=0
while True:
    ret, frame = cap.read()
    if not ret:
        print("无法获取视频帧，退出...")
        break
    # 显示处理后的视频
    cv2.imshow('12', frame)
    # 按'q'退出
    key=cv2.waitKey(1)
    if key == ord('q'):
        break
    if key == ord('s'):
        cv2.imwrite(str(idx)+'.png',frame)
        idx+=1

# 释放资源
cap.release()
cv2.destroyAllWindows()