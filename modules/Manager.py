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
        self.radarHandler, self.cameraHandler = self.createHandlers()
        self.logger = Logger(config.debug)
        self.radarData = []
        self.config = config
        self.lockRadarData = Lock()
        self.lockRadarTimestamp = Lock()
        self.temp = None
        self.state = self.setState()
        self.radarTimestamp = [0]

    @staticmethod
    def createHandlers():
        return RadarHandler(), CameraHandler()

    def setState(self):
        self.state = self.config.mode

        path = './data/records/record-'
        if self.state == 'save':
            ids = 0
            while isdir(path + str(ids)):
                ids += 1
            self.temp = path + str(ids)
            mkdir(self.temp)
            mkdir(self.temp + '/radar')
            mkdir(self.temp + '/camera')

        if self.state == 'load':
            if isdir(path + str(self.config.loadId)):
                self.temp = path + str(self.config.loadId)
            else:
                exit(-10)

        return self.state

    def configureRadar(self):
        self.radarHandler.setLogger(self.logger)
        if self.state != 'load':
            self.radarHandler.set_ports('/dev/ttyACM1', '/dev/ttyACM0') \
                .set_config_file(self.config.configRadar) \
                .send_config()
        if self.state != 'run':
            self.radarHandler.dataRadarPath = self.temp + '/radar'
        self.radarHandler.setState(self.state)

        self.radarHandler.setRadarData(self.radarData)
        self.radarHandler.lockRadarData = self.lockRadarData
        self.radarHandler.lockRadarTimestamp = self.lockRadarTimestamp
        self.radarHandler.timestamp = self.radarTimestamp

    def configureCamera(self):
        self.cameraHandler.setState(self.state)
        self.cameraHandler.setLogger(self.logger)

    def fpsFromCamera(self):
        frames = 120
        i = 0
        start = time.time()
        while i < 120:
            ret, frame = self.cameraHandler.captureFrame()
            i += 1

        seconds = time.time() - start
        return frames / seconds

    def runner(self):
        fusion = Fusion.Fusion(self.config)

        if self.state != 'load':
            fps = int(self.fpsFromCamera() + 0.5)
        self.radarHandler.start()

        if self.state == 'save':
            c = 0

            while self.cameraHandler.cap.isOpened():
                ret, frame = self.cameraHandler.captureFrame()

                if ret:
                    if c % fps == 0:
                        timestamp = int(time.time())

                        filename = self.temp + '/camera/img-' + str(timestamp) + '.png'
                        cv2.imwrite(filename, frame)
                        c = 0

                    cv2.imshow('frame', frame)
                    c += 1
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self.radarHandler.setState('cancel')
                        break
                else:
                    break
            exit(0)

        oldFusion = None

        if self.state == 'load':
            for file in sorted(os.listdir(self.temp + '/camera/')):
                # print(file)
                fusedCount = 0
                pick = [0]
                img = cv2.imread(self.temp + '/camera/' + file)
                frame = imutils.resize(img, width=min(self.config.imageSize, img.shape[1]))

                timestamp = int(file.split('-')[1].split('.')[0]) * 1000

                while True:
                    self.lockRadarTimestamp.acquire()
                    try:
                        timestampRadar = self.radarTimestamp[0]
                    finally:
                        self.lockRadarTimestamp.release()

                    if (timestampRadar - 50 < timestamp) & (timestamp < timestampRadar + 50):
                        pick = Detect.detectPedestrian(frame, self.config.winStride, self.config.scale)

                        self.lockRadarData.acquire()
                        try:
                            fused = fusion.fuse(pick,
                                                [frame.shape[0], frame.shape[1]],
                                                self.radarData)
                        finally:
                            self.lockRadarData.release()

                        if fused is not None:
                            oldFusion = fused[0]
                            fusedCount = fused[1]
                        break

                    if timestampRadar + 50 > timestamp:
                        break
                    time.sleep(0.1)

                self.lockRadarData.acquire()
                try:
                    self.cameraHandler.insertCountDataToImage(frame, [fusedCount, len(pick), len(self.radarData)])
                finally:
                    self.lockRadarData.release()

                if oldFusion is None:
                    cv2.imshow('Frame', frame)
                    if cv2.waitKey(25) == ord('q'):
                        self.radarHandler.setState('cancel')
                        break
                    continue

                for o in oldFusion:
                    for oo in o:
                        if oo.detected is not True:
                            continue
                        self.cameraHandler.insertDataToImage(frame, oo)

                cv2.imshow('Frame', frame)
                # print(time.time() * 1000 - tms)
                if cv2.waitKey(25) == ord('q'):
                    self.radarHandler.setState('cancel')
                    break

            self.radarHandler.setState('cancel')
            while self.radarHandler.is_alive():
                time.sleep(0.4)
            print('5')
            try:
                self.cameraHandler.releaseAndClose()
                exit(0)
            except RuntimeError:
                pass

        c = 0

        # location = './data/UnicamSaver/20190510/04'
        # sleeeep = 1.8
        # for file in sorted(os.listdir(location)):
        #
        #     # print(file)
        #     img = cv2.imread(location + '/' + file)
        #     # tms = time.time() * 1000
        #     # img = cv2.imread('./data/me1.png')
        #     frame = imutils.resize(img, width=min(self.config.imageSize, img.shape[1]))
        #     pick = Detect.detectPedestrian(frame, self.config.winStride, self.config.scale)
        #     fused = fusion.fuse(pick,
        #                         [frame.shape[0], frame.shape[1]],
        #                         self.radarData)
        #
        #     # print(self.radarData)
        #     self.cameraHandler.insertCountDataToImage(frame, [fused[1] if fused is not None else 0, len(pick),
        #                                                       len(self.radarData)])
        #
        #     if fused is None:
        #         cv2.imshow('Frame', frame)
        #         # print(time.time() * 1000 - tms)
        #         cv2.waitKey(25)
        #         # exit()
        #         continue
        #
        #     for o in fused[0]:
        #         for oo in o:
        #             if oo.detected is not True:
        #                 continue
        #             self.cameraHandler.insertDataToImage(frame, oo)
        #
        #     cv2.imshow('Frame', frame)
        #     # print(time.time() * 1000 - tms)
        #     cv2.waitKey(25)
        #     # exit()
        #     time.sleep(sleeeep)

        while self.cameraHandler.cap.isOpened():
            time.sleep(0.001)
            fusedCount = 0
            pick = [0]

            # Capture frame-by-frame
            ret, frame = self.cameraHandler.captureFrame()
            frame = imutils.resize(frame, width=min(600, frame.shape[1]))

            if ret:
                # if (c % self.config.oldDetection == 0) & (self.config.oldDetection > 0):
                #     oldFusion = None

                if c % int(fps / 4) == 0:
                    timestamp = time.time() * 1000
                    self.lockRadarTimestamp.acquire()
                    try:
                        timestampRadar = self.radarTimestamp[0]
                    finally:
                        self.lockRadarTimestamp.release()

                    if timestampRadar - 50 < timestamp < timestampRadar + 50:

                        pick = Detect.detectPedestrian(frame, self.config.winStride, self.config.scale)

                        self.lockRadarData.acquire()
                        try:
                            fused = fusion.fuse(pick,
                                                [self.cameraHandler.cap.get(3), self.cameraHandler.cap.get(4)],
                                                self.radarData)
                        finally:
                            self.lockRadarData.release()

                        if fused is not None:
                            oldFusion = fused[0]
                            fusedCount = fused[1]
                    c = 0

                self.lockRadarData.acquire()
                try:
                    self.cameraHandler.insertCountDataToImage(frame, [fusedCount, len(pick), len(self.radarData)])
                finally:
                    self.lockRadarData.release()

                if oldFusion is None:
                    cv2.imshow('Frame', frame)
                    if cv2.waitKey(25) == ord('q'):
                        self.radarHandler.setState('cancel')
                        break
                    c += 1
                    continue

                for o in oldFusion:
                    for oo in o:
                        if oo.detected is not True:
                            continue
                        self.cameraHandler.insertDataToImage(frame,oo)

                # Display the resulting frame
                cv2.imshow('Frame', frame)

                # Press Q on keyboard to  exit
                if cv2.waitKey(25) == ord('q'):
                    break
                c += 1

            # Break the loop
            else:
                break

        self.radarHandler.join()
        self.cameraHandler.releaseAndClose()

        # counter = 0
        # while self.radarHandler.is_alive():
        #     if counter == 100:
        #         exit(-5)
        #     counter += 1
