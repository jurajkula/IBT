import io
import json
import threading
import time

import math
import matplotlib.cm as cm
import matplotlib.pyplot as pyplot
import serial

from modules.Radar.RadarStructures import *
from modules.constants import *
from modules.errors import *

syncPatternUINT64 = [int('0102', 16), int('0304', 16), int('0506', 16), int('0708', 16)]
syncPatternUINT64 = np.array(syncPatternUINT64, np.uint16).view(np.uint64)

syncPatternUINT8 = [int('0102', 16), int('0304', 16), int('0506', 16), int('0708', 16)]
syncPatternUINT8 = np.array(syncPatternUINT8, np.uint16).view(np.uint8)


def computeH(s):
    posx = s[0]
    posy = s[1]
    velx = s[2]
    vely = s[3]
    range = math.sqrt(posx * posx + posy * posy)
    if posy == 0:
        azimuth = math.pi / 2
    elif posy > 0:
        azimuth = math.atan(posx / posy)
    else:
        azimuth = math.atan(posx / posy) + math.pi
    doppler = (posx * velx + posy * vely) / range
    return np.array((range, azimuth, doppler)).transpose()


def getDim(G, C, A):
    o, D, V = np.linalg.svd(A / G)

    a = 1 / math.sqrt(D[0])
    b = 1 / math.sqrt(D[1])
    c = 1 / math.sqrt(D[2])

    return max([a, b])


def savePointCloudToJSON(path, detectTime, objects):

    data = [{
        'time': detectTime,
        'objects': objects,
    }]
    with open(path + '/radardata.temp', 'a') as outfile:
        outfile.write(json.dumps(data))
        outfile.write('\n')


def validateChecksum(header):
    h = np.array(header, np.uint8).view(np.uint16)
    a = np.uint32(np.sum(h))

    b = np.array(a, np.uint32)

    dt = np.dtype([('f1', np.uint16), ('f2', np.uint16)])
    b = b.view(dtype=dt)

    c = b['f1'] + b['f2']
    rr = np.invert(c)
    return rr


def loadPointCloudFromJSON(tempFile):
    line = tempFile.readline()

    if not line:
        return None

    data = json.loads(line)[0]
    return data


class RadarHandler (threading.Thread):
    configFile: io.TextIOWrapper
    controlPort: serial.Serial
    dataPort: serial.Serial
    lockRadarData: threading.Lock()
    lockRadarTimestamp: threading.Lock()

    def __init__(self):
        threading.Thread.__init__(self)
        self.dataPort = 0
        self.controlPort = 0
        self.configFile = None
        self.defaultRepeating = 5

        self.frameHeaderLengthInBytes = lengthFromStruct(frameHeaderStructType)
        self.tlvHeaderLengthInBytes = lengthFromStruct(tlvHeaderStruct)
        self.pointLengthInBytes = lengthFromStruct(pointStruct)
        self.targetLengthInBytes = lengthFromStruct(targetStruct)
        self.indexLengthInBytes = 1
        self.lostSync = 0
        self.clutterPoints = np.zeros([2, 1])
        self.count = 1000
        self.outOfSyncBytes = 0
        self.gotHeader = 0
        self.rxHeader = 0
        self.byteCount = 0
        self.targetFrameNum = 0
        self.frameNum = 0
        self.rxData = np.zeros((10000, 1), np.uint8)
        self.offset = 0
        self.numTargets = 0
        self.numInputPoints = 0
        self.trackerRun = 'Target'
        self.mIndex = np.zeros(1)
        self.point3D = np.zeros(3)
        self.numOutputPoints = 0
        self.pointCloud = []
        self.posAll = []

        self.maxNumTracks = 20
        self.maxNumPoints = 250
        self.timestamp = [0]
        self.tempFile = None
        self.state = 'run'
        self.logger = None
        self.radarData = []
        self.lockRadarData = None
        self.lockRadarTimestamp = None
        self.dataRadarPath = ''

    def setRadarData(self, radarData):
        self.radarData = radarData

    def setLogger(self, logger):
        self.logger = logger

    def setState(self, state):
        self.state = state

        # if state == 'save':
        #     if os.path.isfile('data/pointcloud.temp'):
        #         os.remove("data/pointcloud.temp")

        if state == 'load':
            self.setTempFile()

        return self

    def set_ports(self, dataPort, controlPort):
        reconnect = self.defaultRepeating

        while reconnect > 0:
            reconnect -= 1
            time.sleep(1)
            if self.set_data_port(dataPort):
                break

        reconnect = self.defaultRepeating
        while reconnect > 0:
            reconnect -= 1
            time.sleep(1)
            if self.set_control_port(controlPort):
                break

        if not self.ports_connected():
            self.logger.log('Cannot connect to ports.', LOGGER_STATE_ERROR)
            exit(ERROR_PORTS_CONNECTION)
        return self

    def setTempFile(self):
        self.tempFile = open(self.dataRadarPath + '/radardata.temp', 'r')
        # print(self.dataRadarPath)
        return self

    def set_data_port(self, dataPort):
        print('Connecting data port...')
        try:
            self.dataPort = serial.Serial(dataPort, 921600, timeout=0.2)
        except (ValueError, serial.SerialException):
            self.dataPort = 0
            print('Could not connect to data port')
            return False
        return True

    def set_control_port(self, controlPort):
        print('Connecting control port...')
        try:
            self.controlPort = serial.Serial(controlPort, 115200, timeout=0.2)
        except (ValueError, serial.SerialException):
            self.controlPort = 0
            print('Could not connect to control port')
            return False
        return True

    def set_config_file(self, config):
        self.configFile = config
        return self

    def ports_connected(self):
        if self.controlPort == 0:
            return False

        if self.dataPort == 0:
            return False

        return True

    def send_config(self):
        if self.ports_connected() == 0:
            return

        for line in self.configFile:
            if line[0] == '%':
                continue
            line = line.strip()
            line += "\n"
            done = b''
            count = 1

            while done.decode().find('Done') == -1:
                if count > 5:
                    self.logger.log('Cannot send command to radar.', LOGGER_STATE_ERROR)
                    exit(ERROR_RADAR_COMMAND)

                sleepTime = 0.1 * count

                self.controlPort.write(line.encode())
                time.sleep(sleepTime)
                echo = self.controlPort.readline()
                time.sleep(sleepTime)
                done = self.controlPort.readline()
                time.sleep(sleepTime)
                prompt = self.controlPort.readline()
                time.sleep(sleepTime)
                print(echo)
                print(done)
                print(prompt)
                count += 1

    def getData(self, dataType, length):
        data = None
        if self.ports_connected():
            data = self.dataPort.read(length)
            data = np.frombuffer(data, dtype=dataType)

        return data

    def initFrameHeaderStruct(self):
        offset = 0
        for i in frameHeaderStructType:
            for j in range(len(i)):
                i[j] = np.array(self.rxHeader[offset:offset + i[j].itemsize], np.uint8)
                offset += i[j].itemsize

        return offset

    def run(self):
        if self.state == 'load':
            while self.tempFile.readable():
                if self.state == 'cancel':
                    break

                data = loadPointCloudFromJSON(self.tempFile)
                if data is None:
                    exit(0)
                self.timestamp[0] = data['time']
                self.radarData.clear()
                for obj in data['objects']:
                    self.radarData.append(obj)

                time.sleep(0.09)
            exit(0)

        periodOfReset = 0
        while self.state != 'cancel':
            while self.lostSync == 0:
                if self.state == 'cancel':
                    break

                if periodOfReset % 25 == 0:
                    self.dataPort.flush()
                periodOfReset += 1

                if self.gotHeader == 0:
                    self.rxHeader = self.getData(np.uint8, self.frameHeaderLengthInBytes)

                    self.lockRadarTimestamp.acquire()
                    try:
                        self.timestamp[0] = time.time() * 1000
                    finally:
                        self.lockRadarTimestamp.release()

                    self.byteCount = self.rxHeader.size * self.rxHeader.itemsize

                magicBytes = np.array(self.rxHeader[0:8], np.uint8)
                magicBytes = magicBytes.view(np.uint64)

                if magicBytes != syncPatternUINT64:
                    self.lostSync = 1
                    break

                if self.byteCount != self.frameHeaderLengthInBytes:
                    self.lostSync = 1
                    break

                if validateChecksum(self.rxHeader) != 0:
                    self.lostSync = 1
                    break

                self.offset = self.initFrameHeaderStruct()

                if self.gotHeader == 1:
                    if frameHeaderStructType['frameNumber'].astype(np.uint8).view(np.uint32) > self.targetFrameNum:
                        self.targetFrameNum = frameHeaderStructType['frameNumber'].astype(np.uint8).view(np.uint32)
                        self.gotHeader = 0
                    else:
                        self.gotHeader = 0
                        self.lostSync = 1
                        break

                self.targetFrameNum = frameHeaderStructType['frameNumber'].astype(np.uint8).view(np.uint32)

                dataLength = frameHeaderStructType['packetLength'].astype(np.uint8).view(
                    np.uint32).__int__() - self.frameHeaderLengthInBytes

                self.numInputPoints = 0
                self.numTargets = 0
                self.mIndex = np.zeros(1)

                TID = []
                S = []
                EC = []
                G = []

                if dataLength > 0:
                    self.rxData = self.getData(np.uint8, dataLength)

                    self.byteCount = self.rxData.size * self.rxData.itemsize

                    if self.byteCount != float(dataLength):
                        self.lostSync = 1
                        break

                    self.offset = 0

                    self.numInputPoints = 0
                    for nTlv in range(frameHeaderStructType['numTLVs'].astype(np.uint8).view(np.uint16).__int__()):
                        tlvType = np.array(self.rxData[self.offset + 0:self.offset + 4], np.uint8).view(np.uint32)
                        tlvLength = np.array(self.rxData[self.offset + 4:self.offset + 8], np.uint8).view(np.uint32)
                        if tlvLength + self.offset > dataLength:
                            self.lostSync = 1
                            break

                        self.offset += self.tlvHeaderLengthInBytes

                        valueLength = tlvLength.__int__() - self.tlvHeaderLengthInBytes
                        if tlvType.__int__() == 6:
                            self.numInputPoints = int(valueLength / 16)
                            if self.numInputPoints > 0:
                                p = np.array(self.rxData[self.offset: self.offset + valueLength], np.uint8).view(
                                    np.single)

                                self.pointCloud = p.reshape(4, self.numInputPoints)

                                self.posAll = [np.multiply(self.pointCloud[0, :], np.sin(self.pointCloud[1, :])),
                                               np.multiply(self.pointCloud[0, :], np.cos(self.pointCloud[1, :]))]

                                self.numOutputPoints = np.size(self.pointCloud, 1)

                            self.offset += valueLength

                        if tlvType.__int__() == 7:
                            self.numTargets = int(valueLength / self.targetLengthInBytes)
                            TID = np.zeros((1, self.numTargets))[0]
                            S = np.zeros((6, self.numTargets))
                            EC = np.zeros((9, self.numTargets))
                            G = np.zeros((1, self.numTargets))[0]

                            for n in range(0, self.numTargets):
                                TID[n] = np.array(self.rxData[self.offset: self.offset + 4], np.uint8).view(np.uint32)
                                S[:, n] = np.array(self.rxData[self.offset + 4: self.offset + 28], np.uint8).view(
                                    np.single)
                                EC[:, n] = np.array(self.rxData[self.offset + 28: self.offset + 64], np.uint8).view(
                                    np.single)
                                G[n] = np.array(self.rxData[self.offset + 64: self.offset + 68], np.uint8).view(
                                    np.single)
                                self.offset += 68

                        if tlvType.__int__() == 8:
                            numIndices = int(valueLength / self.indexLengthInBytes)

                            self.mIndex = np.array(self.rxData[self.offset: self.offset + numIndices], np.uint8).view(
                                np.uint8)
                            self.offset += valueLength

                if self.numInputPoints == 0:
                    self.numOutputPoints = 0
                    self.pointCloud = np.single(np.zeros((4, 0)))
                    self.posAll = []
                    self.posInRange = []

                if self.numTargets == 0:
                    TID = []
                    S = []
                    EC = []
                    G = []

                if self.trackerRun == 'Target':
                    if self.numTargets == 0:
                        TID = np.zeros((1, 1))
                        S = np.zeros((6, 1))
                        EC = np.zeros((9, 1))
                        G = np.zeros((1, 1))

                if (np.isnan(S) != 0).sum():
                    self.lostSync = 1
                    break

                if (np.isnan(EC) != 0).sum():
                    self.lostSync = 1
                    break

                tNumC = np.size(TID, axis=0)
                peopleCountTotal = tNumC
                self.logger.log('PEOPLE: ' + str(peopleCountTotal))

                if self.mIndex.shape[0]:
                    self.mIndex += 1

                self.lockRadarData.acquire()
                try:
                    self.radarData.clear()

                    objectsList = []

                    for n in range(tNumC):
                        tid = TID[n] + 1
                        if tid > self.maxNumTracks:
                            self.lostSync = 1
                            break

                        vel = math.sqrt(math.pow(S[2, n], 2) + math.pow(S[3, n], 2))

                        rObject = {
                            'distance': S[1, n],
                            'x': S[0, n],
                            'velocity': vel
                        }
                        self.radarData.append(rObject)
                        objectsList.append(rObject)

                finally:
                    self.lockRadarData.release()

                if self.state == 'save':
                    savePointCloudToJSON(self.dataRadarPath, self.timestamp[0], objectsList)

                if self.posAll:
                    self.point3D = np.array((self.posAll[0], self.posAll[1], self.pointCloud[2, :]))

                time.sleep(0.06)

            while self.lostSync:
                n = 0
                for n in range(8):
                    rxByte = self.getData(np.uint8, 1)

                    if rxByte != syncPatternUINT8[n]:
                        self.outOfSyncBytes = self.outOfSyncBytes + 1
                        break

                if n == 7:
                    self.lostSync = 0

                    self.frameNum = self.frameNum + 1
                    if self.frameNum > 10000:
                        self.frameNum = 1

                    header = self.getData(np.uint8, self.frameHeaderLengthInBytes - 8)

                    self.byteCount = header.size * header.itemsize
                    header = header.tolist()
                    self.rxHeader = np.array([np.transpose(syncPatternUINT8)]).tolist()[0]
                    self.rxHeader += header

                    self.byteCount = self.byteCount + 8
                    self.gotHeader = 1
                    self.lockRadarTimestamp.acquire()
                    try:
                        self.timestamp[0] = time.time() * 1000
                    finally:
                        self.lockRadarTimestamp.release()
