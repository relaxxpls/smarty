import cv2
import numpy as np
import imutils
import gpiozero

input_led = gpiozero.LED(17)  # RPi initialisations
input_led.off()
robot = gpiozero.Robot(left=(4, 14), right=(17, 18))
# speed = 1   # later dynamically define speed depending on distance

def nothing(n):
    pass

def select_obj():
    cv2.namedWindow("Choose the object")
    cv2.createTrackbar("Hue Start", "Choose the object", 0, 179, nothing)
    cv2.createTrackbar("Hue End", "Choose the object", 179, 179, nothing)
    cv2.createTrackbar("Saturation Start", "Choose the object", 0, 255, nothing)
    cv2.createTrackbar("Saturation End", "Choose the object", 255, 255, nothing)
    cv2.createTrackbar("Value Start", "Choose the object", 0, 255, nothing)
    cv2.createTrackbar("Value End", "Choose the object", 255, 255, nothing)
    cv2.createTrackbar("2 = Selection Made\n1 = Previous Selection\n0 = Quit", "Choose the object", 0, 2, nothing)
    lower_range = None
    upper_range = None
    key = 0

    input_vid = cv2.VideoCapture(0)
    ret = True
    while ret:
        ret, frame = input_vid.read()
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        img_hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        hs = cv2.getTrackbarPos("Hue Start", "Choose the object")
        ss = cv2.getTrackbarPos("Saturation Start", "Choose the object")
        vs = cv2.getTrackbarPos("Value Start", "Choose the object")
        lower_range = np.array([hs, ss, vs], dtype=np.uint8)

        he = cv2.getTrackbarPos("Hue End", "Choose the object")
        se = cv2.getTrackbarPos("Saturation End", "Choose the object")
        ve = cv2.getTrackbarPos("Value End", "Choose the object")
        upper_range = np.array([he, se, ve], dtype=np.uint8)
        key = cv2.getTrackbarPos("2 = Selection Made\n1 = Previous Selection\n0 = Quit", "Choose the object")

        mask = cv2.inRange(img_hsv, lower_range, upper_range)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        color_mask = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow("Select your Object (mask)", color_mask)

        if cv2.waitKey(1) & 0xff == ord('q'):
            break
    return lower_range, upper_range, key

def detect_obj(lower_range, upper_range):
    input_vid = cv2.VideoCapture(0)
    width = 1280  # Screen Dimensions : 1280W * 960H
    ret = True
    while ret:
        ret, frame = input_vid.read()
        frame = imutils.resize(frame, width)           # resize if needed
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        img_hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(img_hsv, lower_range, upper_range)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contour = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour = imutils.grab_contours(contour)
        frame_marked = frame.copy()
        if len(contour) > 0:
            largest_contour = max(contour, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            m = cv2.moments(largest_contour)
            centre = (int(m['m10']/m['m00']), int(m['m01']/m['m00']))
            if radius > 10:
                cv2.circle(frame_marked, (int(x), int(y)), int(radius), (255, 255, 255), 2)
                cv2.circle(frame_marked, centre, 5, (150, 150, 150), -1)

                input_led.on()      # If object detected, we will Switch on the Signal LED
                # Left-Right Movement
                if width//3 < x < 2*width//3:
                    print("I See You!")
                elif x < width//3:
                    print("Turning Left")
                    robot.left()
                elif x > 2*width//3:
                    print("Turning Right")
                    robot.right()
                else:
                    print("Subject Lost! (You Can Run, But You Can't Hide!")
                # front-back movement will be defined here

            else:
                input_led.off()

        cv2.imshow("Marked Frame!", frame_marked)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break
    input_vid.release()
    cv2.destroyAllWindows()

def main():
    lr, ur, key = select_obj()
    if key == 2:
        with open("hsv_settings.txt", 'w') as file:
            file.write(str(lr[0])+" "+str(lr[1])+" "+str(lr[2])+"\n"+str(ur[0])+" "+str(ur[1])+" "+str(ur[2]))
        detect_obj(lr, ur)
    elif key == 1:
        with open("hsv_settings.txt", 'r') as file:
            line = file.readlines()
            str_parts = line[0].split()
            lr = np.array([str_parts[0], str_parts[1], str_parts[2]], dtype=np.uint8)
            str_parts = line[1].split()
            ur = np.array([str_parts[0], str_parts[1], str_parts[2]], dtype=np.uint8)
        detect_obj(lr, ur)
    else:
        pass

if __name__ == "__main__":
    main()
