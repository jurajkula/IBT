import imutils

from modules.CaptureCamera import Detect
from modules.CaptureCamera.Capture import CameraHandler
from modules.Fusion import Fusion
from modules.Radar.radar_handler import RadarHandler
import cv2
from modules.Logger.Logger import Logger


class Manager:
    def __init__(self):
        self.state = 'load'
        self.radarHandler, self.cameraHandler = self.createHandlers()
        self.logger = Logger(True)
        self.radarData = []

    @staticmethod
    def createHandlers():
        return RadarHandler(), CameraHandler()

    def setState(self, state):
        self.state = state

    def configureRadar(self):
        if self.state != 'load':
            self.radarHandler.set_ports('/dev/ttyACM1', '/dev/ttyACM0')\
                .set_config_file("./mmw_pplcount_demo_default.cfg")\
                .send_config()
        self.radarHandler.setState(self.state)
        self.radarHandler.setLogger(Logger(True))
        self.radarHandler.setRadarData(self.radarData)

    def configureCamera(self):
        self.cameraHandler.setState(self.state)
        self.cameraHandler.setLogger(Logger(True))

    def aa(self):
        return self.cameraHandler.cap.read()

    def runner(self):
        fusion = Fusion.Fusion()

        self.radarHandler.start()

        c = 1
        oldFusion = None

        oldFusionFrames = 5

        while self.cameraHandler.cap.isOpened():
            # Capture frame-by-frame
            ret, frame = self.aa()
            frame = imutils.resize(frame, width=min(600, frame.shape[1]))

            if ret:
                if (c % oldFusionFrames == 0) & (oldFusionFrames > 0):
                    oldFusion = None

                if c % 3 == 0:
                    pick = Detect.detectPedestrian(frame)
                    c = 0
                    print(self.radarData)

                    fused = fusion.fuse(pick,
                                        [self.cameraHandler.cap.get(3), self.cameraHandler.cap.get(4)],
                                        self.radarData)

                    if fused is not None:
                        oldFusion = fused

                if oldFusion is None:
                    cv2.imshow('Frame', frame)
                    if cv2.waitKey(25) & 0xFF == ord('q'):
                        break
                    c += 1
                    continue

                for o in oldFusion:
                    for oo in o:
                        if oo.detected is not True:
                            continue

                        cv2.rectangle(
                            frame,
                            (oo.rect[0], oo.rect[1]),
                            (oo.rect[2], oo.rect[3]),
                            (0, 255, 0),
                            2)

                        cv2.putText(frame,
                                    'Distance: ' + str(round(oo.distance, 4)) + 'm',
                                    (oo.rect[0], oo.rect[1] - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.2,
                                    (255, 0, 0),
                                    1,
                                    cv2.LINE_AA)

                        cv2.putText(frame,
                                    'Velocity: ' + str(round(oo.velocity, 4)) + 'm/s',
                                    (oo.rect[0], oo.rect[1] - 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.2,
                                    (255, 0, 0),
                                    1,
                                    cv2.LINE_AA)

                # Display the resulting frame
                cv2.imshow('Frame', frame)

                # Press Q on keyboard to  exit
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    break
                c += 1

            # Break the loop
            else:
                break

        self.radarHandler.join()
