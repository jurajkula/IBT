import sys
from modules.errors import ERROR_FILE_NOT_FOUND


class Config:
    def __init__(self):
        self.configGlobal = None
        self.configRadar = None
        self.oldDetection = 5

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
