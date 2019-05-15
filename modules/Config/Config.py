import sys
from modules.errors import ERROR_FILE_NOT_FOUND


class Config:
    def __init__(self, data):
        self.configGlobal = None
        self.configRadar = None
        self.oldDetection = 5
        self.fusionDelimiter = 8
        self.radarX = 12
        self.winStride = 6
        self.scale = 1.2
        self.shift = 6
        self.debug = data['debug']
        self.mode = data['mode']
        self.imageSize = 600
        self.loadId = data['id']

    def loadDataFromRadarConfig(self):
        for line in self.configRadar:
            if line.find('SceneryParam') == -1:
                continue

            data = line.split()
            self.shift = float(data[1]) * (-1)
            self.radarX = abs(float(data[2])) + abs(float(data[1]))
        self.loadConfigFiles()

    def loadDataFromGlobalConfig(self):
        for line in self.configGlobal:
            if line.find('%') != -1:
                continue

            if line.find('FusionDelimiter') != -1:
                self.fusionDelimiter = int(line.split(':')[1].strip())

            if line.find('WinStride') != -1:
                self.winStride = int(line.split(':')[1].strip())

            if line.find('Scale') != -1:
                self.scale = float(line.split(':')[1].strip())

            if line.find('ImageSize') != -1:
                self.imageSize = int(line.split(':')[1].strip())

    def loadConfigFiles(self):
        try:
            self.configGlobal = open('config/global.cfg', 'r')
        except FileNotFoundError:
            print('Cannot open config file: global.cfg', file=sys.stderr)
            exit(ERROR_FILE_NOT_FOUND)

        try:
            self.configRadar = open('config/mmw_pplcount_demo_default.cfg', 'r')
        except FileNotFoundError:
            print('Cannot open config file: mmw_pplcount_demo_default.cfg', file=sys.stderr)
            exit(ERROR_FILE_NOT_FOUND)
