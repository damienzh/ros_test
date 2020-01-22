import cv2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    ret, frame = cap.read()
    if ret:
        frame_small = cv2.resize(frame, (480, 360))
        cv2.imshow('cam_original', frame)
        cv2.imshow('small', frame_small)
    k = cv2.waitKey(1)
    if k == ord('q'):
        break

cv2.destroyAllWindows()
cap.release()
