import cv2
cv2.namedWindow("webcam test")
cam_url = 'rtsp://192.168.0.11/stream1'
cap = cv2.VideoCapture(cam_url)

if cap.isOpened():
    rval, frame = cap.read()
else:   
    cap.open(cam_url)
    rval = False

while rval:
    frame = cv2.resize(frame, (1920, 1080))  # resize the frame
    cv2.imshow("webcam test", frame)
    rval, frame = cap.read()
    key = cv2.waitKey(1)
    if key == 27:  # exit on ESC
        break
cap.release()
cv2.destroyAllWindows()
