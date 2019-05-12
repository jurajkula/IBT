import sys
from modules.errors import ERROR_FILE_NOT_FOUND


class Config:
    def __init__(self):
        self.configGlobal = None
        self.configRadar = None
        self.oldDetection = 5
        self.fusionDelimiter = 10
        self.radarX = 12
        self.winStride = 4
        self.scale = 1.05
        self.shift = 6

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
