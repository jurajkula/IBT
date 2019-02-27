import io

import serial, time


class RadarHandler:
    configFile: io.TextIOWrapper
    controlPort: serial.Serial
    dataPort: serial.Serial

    def __init__(self):
        self.dataPort = 0
        self.controlPort = 0
        self.dataFile = 0
        self.configFile = 0
        self.defaultRepeating = 5

    def set_ports(self):
        reconnect = self.defaultRepeating

        while reconnect > 0:
            reconnect -= 1
            time.sleep(1)
            if self.set_data_port():
                break

        reconnect = self.defaultRepeating
        while reconnect > 0:
            reconnect -= 1
            time.sleep(1)
            if self.set_control_port():
                break

    def set_data_port(self):
        print('Connecting data port...')
        try:
            self.dataPort = serial.Serial('/dev/ttyACM1', 921600, timeout=0.2)
        except (ValueError, serial.SerialException):
            self.dataPort = 0
            print('Could not connect to data port')
            return False
        return True

    def set_control_port(self):
        print('Connecting control port...')
        try:
            self.controlPort = serial.Serial('/dev/ttyACM0', 115200, timeout=0.2)
        except (ValueError, serial.SerialException):
            self.controlPort = 0
            print('Could not connect to control port')
            return False
        return True

    def set_data_file(self, file):
        self.dataFile = open(file, "rb")

    def set_config_file(self, file):
        self.configFile = open(file, "r")

    def ports_connected(self):
        if self.controlPort == 0:
            return False

        if self.dataPort == 0:
            return False

        return True

    def send_config(self):
        if self.ports_connected() == 0:
            return

        if self.configFile == 0:
            return

        for line in self.configFile:
            if line[0] == '%':
                continue
            line = line.strip()
            print(line)
            line += "\n"
            self.controlPort.write(line.encode())
            time.sleep(1)
            echo = self.controlPort.readline()
            time.sleep(1)
            done = self.controlPort.readline()
            time.sleep(1)
            prompt = self.controlPort.readline()
            time.sleep(1)
            print(echo)
            print(done)
            print(prompt)
