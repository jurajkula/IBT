import os
from threading import Lock

import imutils
from imutils.object_detection import non_max_suppression

from modules.Camera import Detect
from modules.Camera.CameraHandler import CameraHandler
from modules.Config import Config
from modules.Fusion import Fusion
from modules.Radar.RadarHandler import RadarHandler, np
import cv2
from modules.Logger.Logger import Logger
import time


class Manager:
    def __init__(self, config: Config):
        self.state = 'load3'
        self.radarHandler, self.cameraHandler = self.createHandlers()
        self.logger = Logger(True)
        self.radarData = []
        self.config = config
        self.lock = Lock()

    @staticmethod
    def createHandlers():
        return RadarHandler(), CameraHandler()

    def setState(self, state):
        self.state = state

    def configureRadar(self):
        if self.state != 'load3':
            self.radarHandler.set_ports('/dev/ttyACM1', '/dev/ttyACM0')\
                .set_config_file(self.config.configRadar)\
                .send_config()
        self.radarHandler.setState(self.state)
        self.radarHandler.setLogger(Logger(True))
        self.radarHandler.setRadarData(self.radarData)
        self.radarHandler.lock = self.lock

    def configureCamera(self):
        self.cameraHandler.setState(self.state)
        self.cameraHandler.setLogger(Logger(True))

    def runner(self):
        fusion = Fusion.Fusion(self.config)
        self.radarHandler.start()

        # c = 1
        # oldFusion = None
        #
        # location = './data/UnicamSaver/20190510/04'
        #
        # for file in sorted(os.listdir(location)):
        #
        #     print(file)
        #     img = cv2.imread(location + '/' + file)
        #     frame = imutils.resize(img, width=min(800, img.shape[1]))
        #     pick = Detect.detectPedestrian(frame, self.config.winStride, self.config.scale)
        #     fused = fusion.fuse(pick,
        #                         [self.cameraHandler.cap.get(4), self.cameraHandler.cap.get(4)],
        #                         self.radarData)
        #     if fused is None:
        #         cv2.imshow('Frame', frame)
        #         cv2.waitKey(25)
        #         time.sleep(3.6)
        #         continue
        #
        #     for o in fused:
        #         for oo in o:
        #             if oo.detected is not True:
        #                 continue
        #             self.cameraHandler.insertDataToImage(frame, oo)
        #
        #     cv2.imshow('Frame', frame)
        #     cv2.waitKey(25)
        #     time.sleep(3.6)
        #
        # while self.cameraHandler.cap.isOpened():
        #     # Capture frame-by-frame
        #     ret, frame = self.cameraHandler.captureFrame()
        #     frame = imutils.resize(frame, width=min(600, frame.shape[1]))
        #
        #     if ret:
        #         if (c % self.config.oldDetection == 0) & (self.config.oldDetection > 0):
        #             oldFusion = None
        #
        #         if c % 3 == 0:
        #             pick = Detect.detectPedestrian(frame, self.config.winStride, self.config.scale)
        #             c = 0
        #
        #             fused = fusion.fuse(pick,
        #                                 [self.cameraHandler.cap.get(3), self.cameraHandler.cap.get(4)],
        #                                 self.radarData)
        #
        #             if fused is not None:
        #                 oldFusion = fused
        #
        #         if oldFusion is None:
        #             cv2.imshow('Frame', frame)
        #             if cv2.waitKey(25) == ord('q'):
        #                 self.radarHandler.setState('cancel')
        #                 break
        #             c += 1
        #             continue
        #
        #         for o in oldFusion:
        #             for oo in o:
        #                 if oo.detected is not True:
        #                     continue
        #                 self.cameraHandler.insertDataToImage(frame,oo)
        #
        #         # Display the resulting frame
        #         cv2.imshow('Frame', frame)
        #
        #         # Press Q on keyboard to  exit
        #         if cv2.waitKey(25) == ord('q'):
        #             break
        #         c += 1
        #
        #     # Break the loop
        #     else:
        #         break

        self.radarHandler.join()
        # self.cameraHandler.releaseAndClose()

        # counter = 0
        # while self.radarHandler.is_alive():
        #     if counter == 100:
        #         exit(-5)
        #     counter += 1
