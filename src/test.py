from Radar.radar_handler import RadarHandler
from CaptureCamera.Capture import *

isRadar = True
dataPort = 0
controlPort = 0
config = 0
dataFile = 0

radarHandler = RadarHandler()
cameraHandler = CameraHandler()

if isRadar:
    radarHandler.set_ports()\
        .set_config_file("./mmw_pplcount_demo_default.cfg")\
        .send_config()


radarHandler.setState('run')

radarHandler.run()

# cameraHandler.capture()