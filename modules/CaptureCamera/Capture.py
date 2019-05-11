import importlib.machinery

import cv2
import numpy as np
from modules.CaptureCamera import Detect
from modules.Fusion import Fusion


class CameraHandler:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.logger = None
        self.state = 'run'

    def releaseAndClose(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

    def captureFrame(self):
        ret, frame = self.cap.read()
        if ret:
            [image, pick] = Detect.detectPedestrian(frame)

            for (xA, yA, xB, yB) in pick:
                cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
                cv2.putText(image, 'TEST', (xA - 5, yA - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1,
                            cv2.LINE_AA)

            # Display the resulting frame
            cv2.imshow('Frame', image)
        return 'ok'

    def setLogger(self, logger):
        self.logger = logger

    def setState(self, state):
        self.state = state

    def capture(self):

        # Check if camera opened successfully
        if not self.cap.isOpened():
            print("Error opening video stream or file")

        # Read until video is completed
        while self.cap.isOpened():
            # Capture frame-by-frame
            ret, frame = self.cap.read()
            if ret:
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
        self.cap.release()

        # Closes all the frames
        cv2.destroyAllWindows()
