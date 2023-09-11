import  cv2

cap = cv2.VideoCapture('udp://@0.0.0.0:8554?overrun_nonfatal=1&fifo_size=50000000&buffer_size=425984', cv2.CAP_FFMPEG)

if not cap.isOpened():
    print('VideoCapture not opened')
    exit(-1)

while True:
    ret, frame = cap.read()

    if not ret:
        print('frame empty')
        break

    cv2.imshow('image', frame)

    if cv2.waitKey(1)&0XFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()