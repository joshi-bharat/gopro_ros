#!/usr/bin/env python

import cv2
import argparse
# cap = cv2.VideoCapture(
#     'udp://172.22.186.53:8554?fifo_size=50000000&buffer_size=10000000', cv2.CAP_FFMPEG)

cap = cv2.VideoCapture('/dev/video2', cv2.CAP_V4L2)

if not cap.isOpened():
    print('VideoCapture not opened')
    exit(-1)

while True:
    ret, frame = cap.read()

    if not ret:
        print('frame empty')
        break

    cv2.imshow('image', frame)

    if cv2.waitKey(1) & 0XFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
