from Radar.radar_handler import RadarHandler

isRadar = False
dataPort = 0
controlPort = 0
config = 0
dataFile = 0

radarHandler = RadarHandler()

if isRadar:
    radarHandler.set_ports()\
        .set_config_file("./mmw_pplcount_demo_default.cfg")\
        .send_config()
else:
    radarHandler.set_data_file("./data.out")

radarHandler.run()
