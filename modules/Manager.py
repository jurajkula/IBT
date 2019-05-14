import os
import time
from os import mkdir
from os.path import isdir
from threading import Lock

import cv2
import imutils

from modules.Camera import Detect
from modules.Camera.CameraHandler import CameraHandler
from modules.Config import Config
from modules.Fusion import Fusion
from modules.Logger.Logger import Logger
from modules.Radar.RadarHandler import RadarHandler


class Manager:
    def __init__(self, config: Config):
        self.state = config.mode
        self.radarHandler, self.cameraHandler = self.createHandlers()
        self.logger = Logger(config.debug)
        self.radarData = []
        self.config = config
        self.lock = Lock()
        self.temp = None

    @staticmethod
    def createHandlers():
        return RadarHandler(), CameraHandler()

    def setState(self, state):
        self.state = state

    def configureRadar(self):
        # if self.state != 'load3':
        # self.radarHandler.set_ports('/dev/ttyACM1', '/dev/ttyACM0') \
        #     .set_config_file(self.config.configRadar) \
        # .send_config()
        self.radarHandler.setState(self.state)
        self.radarHandler.setLogger(self.logger)
        self.radarHandler.setRadarData(self.radarData)
        self.radarHandler.lock = self.lock
        self.radarHandler.dataRadarPath = self.temp + '/radar'

    def configureCamera(self):
        self.cameraHandler.setState(self.state)
        self.cameraHandler.setLogger(self.logger)

    def runner(self):
        if self.state == 'save':
            ids = 0
            while isdir('./data/records/record-' + str(ids)):
                ids += 1
            self.temp = './data/records/record-' + str(ids)
            mkdir(self.temp)
            mkdir(self.temp + '/radar')
            mkdir(self.temp + '/camera')
            exit()
        fusion = Fusion.Fusion(self.config)
        self.radarHandler.start()

        c = 0
        oldFusion = None

        location = './data/UnicamSaver/20190510/04'
        sleeeep = 1
        for file in sorted(os.listdir(location)):

            print(file)
            img = cv2.imread(location + '/' + file)
            # tms = time.time() * 1000
            # img = cv2.imread('./data/me1.png')
            frame = imutils.resize(img, width=min(self.config.imageSize, img.shape[1]))
            pick = Detect.detectPedestrian(frame, self.config.winStride, self.config.scale)
            fused = fusion.fuse(pick,
                                [frame.shape[0], frame.shape[1]],
                                self.radarData)

            # print(self.radarData)
            self.cameraHandler.insertCountDataToImage(frame, [fused[1] if fused is not None else 0, len(pick),
                                                              len(self.radarData)])

            if fused is None:
                cv2.imshow('Frame', frame)
                # print(time.time() * 1000 - tms)
                cv2.waitKey(25)
                time.sleep(sleeeep)
                # exit()
                continue

            for o in fused[0]:
                for oo in o:
                    if oo.detected is not True:
                        continue
                    self.cameraHandler.insertDataToImage(frame, oo)

            cv2.imshow('Frame', frame)
            # print(time.time() * 1000 - tms)
            cv2.waitKey(25)
            # exit()
            time.sleep(sleeeep)

        # while self.cameraHandler.cap.isOpened():
        #     fusedCount = 0
        #     pick = [0]
        #
        #     # Capture frame-by-frame
        #     ret, frame = self.cameraHandler.captureFrame()
        #     frame = imutils.resize(frame, width=min(600, frame.shape[1]))
        #
        #     if ret:
        #         # if (c % self.config.oldDetection == 0) & (self.config.oldDetection > 0):
        #         #     oldFusion = None
        #
        #         if c % 3 == 0:
        #             pick = Detect.detectPedestrian(frame, self.config.winStride, self.config.scale)
        #             c = 0
        #             print(pick)
        #
        #             fused = fusion.fuse(pick,
        #                                 [self.cameraHandler.cap.get(3), self.cameraHandler.cap.get(4)],
        #                                 self.radarData)
        #
        #             if fused is not None:
        #                 oldFusion = fused[0]
        #                 fusedCount = fused[1]
        #
        #         self.cameraHandler.insertCountDataToImage(frame, [fusedCount, len(pick), len(self.radarData)])
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
        self.cameraHandler.releaseAndClose()

        # counter = 0
        # while self.radarHandler.is_alive():
        #     if counter == 100:
        #         exit(-5)
        #     counter += 1
