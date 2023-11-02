import cv2
import mediapipe as mp
import numpy as np
import time
import HandTrackingModule as htm
import math
from cvzone.SerialModule import SerialObject

arduino = SerialObject()

cap = cv2.VideoCapture(0)
cTime = 0
pTime = 0

detector = htm.handDetector(detectionCon=0.7)
vol = 0
volBar = 350
servAngle = 0

while True:
    succes, img = cap.read()
    img = detector.findHands(img)
    lmList = detector.findPosition(img, draw=True)
    if len(lmList) != 0:
        # print(lmList[4])

        x1, y1 = lmList[4][1], lmList[4][2]
        x2, y2 = lmList[8][1], lmList[8][2]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        cv2.circle(img, (x1, y1), 7, (0, 0, 255), cv2.FILLED)
        cv2.circle(img, (x2, y2), 7, (0, 0, 255), cv2.FILLED)
        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 3)
        cv2.circle(img, (cx, cy), 7, (0, 0, 255), cv2.FILLED)

        length = math.hypot(x2 - x1, y2 - y1)

        volBar = np.interp(length, [30, 200], [350, 150])
        servAngle = int(np.interp(length, [30, 200], [0, 180]))  # print(length, vol)
        arduino.sendData([servAngle])
        print(int(length), servAngle)
        if length <= 20:
            cv2.circle(img, (cx, cy), 7, (0, 255, 0), cv2.FILLED)
    cv2.rectangle(img, (50, 150), (80, 350), (0, 155, 0), 4)
    cv2.rectangle(img, (50, int(volBar)), (80, 350), (0, 155, 0), cv2.FILLED)
    cv2.putText(
        img,
        f"{servAngle}",
        (48, 390),
        cv2.FONT_HERSHEY_DUPLEX,
        0.7,
        (255, 0, 0),
        3,
    )
    cv2.putText(
        img,
        "Sudut",
        (31, 130),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 0, 0),
        3,
    )

    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    cv2.putText(
        img, f"FPS: {int(fps)}", (10, 50), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 0, 0), 3
    )
    cv2.imshow("img", img)
    if cv2.waitKey(1) == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()
