import importlib.machinery

import cv2
import numpy as np
from CaptureCamera import Detect
from Fusion import Fusion


class CameraHandler:
    def capture(self):
        # Create a VideoCapture object and read from input file
        # If the input is the camera, pass 0 instead of the video file name
        # PN-VY-P1_20190502T180850494.jpg
        # PN-VY-P1_20190502T181751863.jpg
        # PN-VY-P1_20190502T181758246.jpg
        # cap = cv2.VideoCapture(0)
        img = cv2.imread('CaptureCamera/PN-VY-P1_20190502T180847300.jpg')
        # img = cv2.imread('PN-VY-P1_20190502T180850494.jpg')
        # img = cv2.imread('PN-VY-P1_20190502T181751863.jpg')
        # img = cv2.imread('PN-VY-P1_20190502T181758246.jpg')

        [image, pick] = Detect.detectPedestrian(img)
        fusion = Fusion.Fusion(pick, img.shape)
        aa = fusion.fuse()

        for o in aa:
            for oo in o:
                if oo.detected is not True:
                    continue

                cv2.rectangle(
                    image,
                    (oo.rect[0], oo.rect[1]),
                    (oo.rect[2], oo.rect[3]),
                    (0, 255, 0),
                    2)

                cv2.putText(image,
                            'Distance: ' + str(oo.distance) + 'm',
                            (oo.rect[0], oo.rect[1] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.2,
                            (255, 0, 0),
                            1,
                            cv2.LINE_AA)

                cv2.putText(image,
                            'Velocity: ' + str(oo.velocity) + 'km/s',
                            (oo.rect[0], oo.rect[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.2,
                            (255, 0, 0),
                            1,
                            cv2.LINE_AA)

        cv2.imshow('image', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        exit(0)

        # Check if camera opened successfully
        if (cap.isOpened() == False):
            print("Error opening video stream or file")

        # Read until video is completed
        while (cap.isOpened()):
            # Capture frame-by-frame
            ret, frame = cap.read()
            if ret == True:
                [image, pick] = Detect.detectPedestrian(frame)

                for (xA, yA, xB, yB) in pick:
                    cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
                    cv2.putText(image, 'TEST', (xA - 5, yA - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1,
                                cv2.LINE_AA)

                # Display the resulting frame
                cv2.imshow('Frame', image)

                # Press Q on keyboard to  exit
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    break

            # Break the loop
            else:
                break

        # When everything done, release the video capture object
        cap.release()

        # Closes all the frames
        cv2.destroyAllWindows()
