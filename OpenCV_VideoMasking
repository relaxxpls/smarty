import cv2
import numpy as np

input_vid = cv2.VideoCapture(0)
ret = True


def nothing(x):
    pass


while ret:
    ret, frame = input_vid.read()
    cv2.imshow("Original Video Input", frame)

    img_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    cv2.namedWindow("Choose the object")

    cv2.createTrackbar("Hue Start", "Choose the object", 0, 179, nothing)
    cv2.createTrackbar("Hue End", "Choose the object", 179, 179, nothing)
    cv2.createTrackbar("Saturation Start", "Choose the object", 0, 255, nothing)
    cv2.createTrackbar("Saturation End", "Choose the object", 255, 255, nothing)
    cv2.createTrackbar("Value Start", "Choose the object", 0, 255, nothing)
    cv2.createTrackbar("Value End", "Choose the object", 255, 255, nothing)

    hs = cv2.getTrackbarPos("Hue Start", "Choose the object")
    ss = cv2.getTrackbarPos("Saturation Start", "Choose the object")
    vs = cv2.getTrackbarPos("Value Start", "Choose the object")
    lower_range = np.array([hs, ss, vs])

    he = cv2.getTrackbarPos("Hue End", "Choose the object")
    se = cv2.getTrackbarPos("Saturation End", "Choose the object")
    ve = cv2.getTrackbarPos("Value End", "Choose the object")
    upper_range = np.array([he, se, ve])

    mask = cv2.inRange(img_HSV, lower_range, upper_range)
    cv2.imshow("Select your Object (mask)", mask)

    if cv2.waitKey(1) & 0xff == ord('q'):
        break

input_vid.release()
cv2.destroyAllWindows()
