from modules.Manager import Manager

manager = Manager()
manager.configureRadar()
manager.configureCamera()
try:
    manager.runner()
except:
    exit()
