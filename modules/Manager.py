import imutils

from modules.Camera import Detect
from modules.Camera.CameraHandler import CameraHandler
from modules.Config import Config
from modules.Fusion import Fusion
from modules.Radar.RadarHandler import RadarHandler
import cv2
from modules.Logger.Logger import Logger


class Manager:
    def __init__(self, config: Config):
        self.state = 'load'
        self.radarHandler, self.cameraHandler = self.createHandlers()
        self.logger = Logger(True)
        self.radarData = []
        self.config = config

    @staticmethod
    def createHandlers():
        return RadarHandler(), CameraHandler()

    def setState(self, state):
        self.state = state

    def configureRadar(self):
        if self.state != 'load':
            self.radarHandler.set_ports('/dev/ttyACM1', '/dev/ttyACM0')\
                .set_config_file(self.config.configRadar)\
                .send_config()
        self.radarHandler.setState(self.state)
        self.radarHandler.setLogger(Logger(True))
        self.radarHandler.setRadarData(self.radarData)

    def configureCamera(self):
        self.cameraHandler.setState(self.state)
        self.cameraHandler.setLogger(Logger(True))

    def runner(self):
        fusion = Fusion.Fusion()

        self.radarHandler.start()

        c = 1
        oldFusion = None

        while self.cameraHandler.cap.isOpened():
            # Capture frame-by-frame
            ret, frame = self.cameraHandler.captureFrame()
            frame = imutils.resize(frame, width=min(600, frame.shape[1]))

            if ret:
                if (c % self.config.oldDetection == 0) & (self.config.oldDetection > 0):
                    oldFusion = None

                if c % 3 == 0:
                    pick = Detect.detectPedestrian(frame)
                    c = 0

                    fused = fusion.fuse(pick,
                                        [self.cameraHandler.cap.get(3), self.cameraHandler.cap.get(4)],
                                        self.radarData)

                    if fused is not None:
                        oldFusion = fused

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

        counter = 0
        while self.radarHandler.is_alive():
            if counter == 100:
                exit(-5)
            counter += 1
