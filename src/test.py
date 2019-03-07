from Radar.radar_handler import RadarHandler

isRadar = False
dataPort = 0
controlPort = 0
config = 0
dataFile = 0

radarHandler = RadarHandler()
#radarHandler.set_ports()
radarHandler.set_config_file("./mmw_pplcount_demo_default.cfg")
radarHandler.send_config()
radarHandler.set_data_file("./data.out")
radarHandler.run()
