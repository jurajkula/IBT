import sys
from modules.constants import *


class Logger:
    def __init__(self, isDebug=False):
        self.isDebug = isDebug

    def log(self, message, state=0):
        if self.isDebug:
            if state == LOGGER_STATE_INFO:
                print(message)

            if state == LOGGER_STATE_WARNING:
                print('W: ' + message)

        if state == LOGGER_STATE_ERROR:
            print('E: ' + message, file=sys.stderr)
