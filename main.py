from modules.Config import Config
from modules.Manager import Manager

config = Config.Config()
config.loadConfigFiles()

manager = Manager(config)
manager.configureRadar()
manager.configureCamera()
try:
    manager.runner()
except:
    exit()
