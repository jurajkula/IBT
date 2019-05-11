import io
import os
import threading

import math

import matplotlib.pyplot as pyplot
import matplotlib.cm as cm
import numpy as np
import serial
import time
from statistics import mean
from modules.Radar.radar_structures import *
import json
from modules.constants import *

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


def savePointCloudToJSON(detectTime, header, pointCloud):
    header = np.array(header, np.uint8)

    data = [{
        'time': detectTime,
        'header': header.tolist(),
        'pointcloud': pointCloud.tolist()
    }]
    with open('pointcloud.temp', 'a') as outfile:
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
        exit(-1)

    data = json.loads(line)[0]
    return data


class RadarHandler (threading.Thread):
    configFile: io.TextIOWrapper
    controlPort: serial.Serial
    dataPort: serial.Serial

    def __init__(self):
        threading.Thread.__init__(self)
        self.dataPort = 0
        self.controlPort = 0
        self.dataFile = open('data/Radar/20190510/20190510T100300006.bin')
        self.configFile = 0
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
        self.timestamp = 0
        self.tempFile = open('data/pointcloud.temp', 'r')
        self.state = 'run'
        self.logger = None
        self.radarData = []
        self.lock = None

    def setRadarData(self, radarData):
        self.radarData = radarData

    def setLogger(self, logger):
        self.logger = logger

    def setState(self, state):
        self.state = state

        if state == 'save':
            os.remove("data/pointcloud.temp")

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
        return self

    def setTempFile(self):
        self.tempFile = open('data/pointcloud.temp', 'r')
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

    def set_data_file(self, file):
        self.dataFile = open(file, "rb")
        return self

    def set_config_file(self, file):
        self.configFile = open(file, 'r')
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

        if self.configFile == 0:
            return

        for line in self.configFile:
            if line[0] == '%':
                continue
            line = line.strip()
            print(line)
            line += "\n"
            self.controlPort.write(line.encode())
            time.sleep(0.5)
            echo = self.controlPort.readline()
            time.sleep(0.5)
            done = self.controlPort.readline()
            time.sleep(0.5)
            prompt = self.controlPort.readline()
            time.sleep(0.5)
            print(echo)
            print(done)
            print(prompt)

    def getData(self, dataType, length):
        if self.ports_connected():
            data = self.dataPort.read(length)
            data = np.frombuffer(data, dtype=dataType)
        else:
            data = np.fromfile(self.dataFile, dataType, length)
        return data

    def initFrameHeaderStruct(self):
        offset = 0
        for i in frameHeaderStructType:
            for j in range(len(i)):
                i[j] = np.array(self.rxHeader[offset:offset + i[j].itemsize], np.uint8)
                offset += i[j].itemsize

        return offset

    def run(self):
        # self.timestamp = 1557482580
        if self.state == 'load2':
            current = time.time()

        while True:
            while self.lostSync == 0:
                print(self.timestamp)
                if self.state == 'load':
                    data = loadPointCloudFromJSON(self.tempFile)

                # print(self.timestamp)
                if self.gotHeader == 0:
                    if self.state == 'load':
                        self.rxHeader = np.array(data['header'], np.uint8)
                        self.timestamp = data['time']

                    elif self.state == 'load2':
                        self.rxHeader = np.fromfile(self.dataFile, np.uint8, self.frameHeaderLengthInBytes)
                        new = time.time()
                        self.timestamp += (new - current)
                        current = new


                    else:
                        self.rxHeader = self.getData(np.uint8, self.frameHeaderLengthInBytes)
                        self.timestamp = time.time() * 1000

                    self.byteCount = self.rxHeader.size * self.rxHeader.itemsize

                magicBytes = np.array(self.rxHeader[0:8], np.uint8)
                magicBytes = magicBytes.view(np.uint64)

                if magicBytes != syncPatternUINT64:
                    # print('REMOVE\n')
                    self.lostSync = 1
                    break

                if self.byteCount != self.frameHeaderLengthInBytes:
                    self.lostSync = 1
                    break

                if validateChecksum(self.rxHeader) != 0:
                    self.lostSync = 1
                    break

                # print('CONITNUE')
                # exit()
                self.offset = self.initFrameHeaderStruct()

                if self.gotHeader == 1:
                    print(frameHeaderStructType['frameNumber'].astype(np.uint8).view(np.uint32))
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
                    if self.state == 'load':
                        self.rxData = np.array(data['pointcloud'], np.uint8)
                    elif self.state == 'load2':
                        self.rxData = np.fromfile(self.dataFile, np.uint8, dataLength)
                    else:
                        self.rxData = self.getData(np.uint8, dataLength)

                    self.byteCount = self.rxData.size * self.rxData.itemsize

                    if self.byteCount != float(dataLength):
                        self.lostSync = 1
                        break

                    self.offset = 0

                    if self.state == 'save':
                        savePointCloudToJSON(self.timestamp, self.rxHeader, self.rxData)

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

                                snrAll = self.pointCloud[3, :]

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

                pyplot.clf()
                pyplot.axis([-5, 5, -1, 10])

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

                colorIndex = 0
                self.radarData.clear()

                for n in range(tNumC):
                    tid = TID[n] + 1
                    colors = cm.rainbow(np.linspace(0, 1, tNumC))
                    if tid > self.maxNumTracks:
                        self.lostSync = 1
                        break
                    try:
                        if self.mIndex.shape[0] & (self.mIndex.shape[0] == self.point3D.shape[1]):
                            # color
                            ind = (self.mIndex == tid)

                            # for index in range(ind.size):
                            #     if ind[index]:
                            #         # print(self.point3D[0][index], self.point3D[1][index])
                            #         pyplot.scatter(self.point3D[0][index], self.point3D[1][index], color=colors[colorIndex], s=10)

                    except IndexError:
                        self.logger.log('point3D out of range', LOGGER_STATE_WARNING)
                        # print('point3D out of range')

                    pyplot.scatter(S[0, n], S[1, n], color=colors[colorIndex], facecolors='none', s=100)

                    vel = math.sqrt(math.pow(S[2, n], 2) + math.pow(S[3, n], 2))

                    rObject = {
                        'distance': S[1, n],
                        'x': S[0, n],
                        'velocity': vel
                    }

                    # print(S[:, n])
                    self.radarData.append(rObject)

                    colorIndex += 1

                # print(detectionObjects)
                if self.posAll:
                    self.point3D = np.array((self.posAll[0], self.posAll[1], self.pointCloud[2, :]))

                pyplot.pause(0.05)
                pyplot.draw()

            while self.lostSync:
                self.lostSync = 0
                break
                # print('here')
                n = 0
                for n in range(8):
                    if self.state == 'load2':
                        rxByte = np.fromfile(self.dataFile, np.uint8, 1)
                    else:
                        rxByte = self.getData(np.uint8, 1)

                    if rxByte != syncPatternUINT8[n]:
                        self.outOfSyncBytes = self.outOfSyncBytes + 1
                        break

                if n == 7:
                    self.lostSync = 0

                    self.frameNum = self.frameNum + 1
                    if self.frameNum > 10000:
                        self.frameNum = 1

                    if self.state == 'load2':
                        header = np.fromfile(self.dataFile, np.uint8, self.frameHeaderLengthInBytes - 8)
                    else:
                        header = self.getData(np.uint8, self.frameHeaderLengthInBytes - 8)

                    self.byteCount = header.size * header.itemsize
                    header = header.tolist()
                    self.rxHeader = np.array([np.transpose(syncPatternUINT8)]).tolist()[0]
                    self.rxHeader += header

                    self.byteCount = self.byteCount + 8
                    self.gotHeader = 1
                    self.timestamp = time.time() * 1000

                    # data = self.loadPointCloudFromJSON()
