import importlib.machinery

import cv2

from modules.Logger.Logger import Logger


class CameraHandler:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.logger = None
        self.state = 'run'

    def releaseAndClose(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

    def setLogger(self, logger):
        self.logger = logger

    def setState(self, state):
        self.state = state

    def captureFrame(self):
        return self.cap.read()

    @staticmethod
    def insertCountDataToImage(frame, count):
        cv2.putText(frame,
                    'People detected - fusion: ' + str(count[0]),
                    (10, frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA)
        cv2.putText(frame,
                    'People detected - camera ' + str(count[1]),
                    (10, frame.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA)
        cv2.putText(frame,
                    'People detected - radar ' + str(count[2]),
                    (10, frame.shape[0] - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA)

    @staticmethod
    def insertDataToImage(frame, data):
        # print('t')
        data.print()
        cv2.rectangle(
            frame,
            (data.rect[0], data.rect[1]),
            (data.rect[2], data.rect[3]),
            (0, 255, 0),
            2)

        cv2.putText(frame,
                    'Distance: ' + str(round(data.distance, 4)) + 'm',
                    (data.rect[0], data.rect[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.2,
                    (255, 0, 0),
                    1,
                    cv2.LINE_AA)

        cv2.putText(frame,
                    'Velocity: ' + str(round(data.velocity, 4)) + 'm/s',
                    (data.rect[0], data.rect[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.2,
                    (255, 0, 0),
                    1,
                    cv2.LINE_AA)
