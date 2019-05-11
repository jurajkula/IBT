from Manager import *
from Radar.radar_handler import RadarHandler
from CaptureCamera.Capture import *

manager = Manager()
manager.configureRadar()
manager.configureCamera()
try:
    manager.runner()
except:
    exit()
