import io
import math

import matplotlib.pyplot as pyplot
import numpy as np
import serial
import time


def lengthFromStruct(S):
    length = 0
    for item in S[0]:
        length += item.itemsize
    return length


frameHeaderStructType = np.zeros(1, dtype=[('sync', np.uint64, 8),
                                           ('version', np.uint32, 4),
                                           ('platform', np.uint32, 4),
                                           ('timestamp', np.uint32, 4),
                                           ('packetLength', np.uint32, 4),
                                           ('frameNumber', np.uint32, 4),
                                           ('subframeNumber', np.uint32, 4),
                                           ('chirpMargin', np.uint32, 4),
                                           ('frameMargin', np.uint32, 4),
                                           ('uartSentTime', np.uint32, 4),
                                           ('trackProcessTime', np.uint32, 4),
                                           ('numTLVs', np.uint16, 2),
                                           ('checksum', np.uint16, 2)
                                           ])

tlvHeaderStruct = np.zeros(1, dtype=[('type', np.uint32, 4),
                                     ('length', np.uint32, 4)
                                     ])

pointStruct = np.zeros(1, dtype=[('range', np.float, 4),
                                 ('angle', np.float, 4),
                                 ('doppler', np.float, 4),
                                 ('snr', np.float, 4)
                                 ])

targetStruct = np.zeros(1, dtype=[('tid', np.uint32, 4),
                                  ('posX', np.float, 4),
                                  ('posY', np.float, 4),
                                  ('velX', np.float, 4),
                                  ('velY', np.float, 4),
                                  ('accX', np.float, 4),
                                  ('accY', np.float, 4),
                                  ('EC', np.float, 9 * 4),
                                  ('G', np.float, 4)
                                  ])

syncPatternUINT64 = [int('0102', 16), int('0304', 16), int('0506', 16), int('0708', 16)]
syncPatternUINT64 = np.array(syncPatternUINT64, np.uint16).view(np.uint64)

syncPatternUINT8 = [int('0102', 16), int('0304', 16), int('0506', 16), int('0708', 16)]
syncPatternUINT8 = np.array(syncPatternUINT8, np.uint16).view(np.uint8)


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

    @staticmethod
    def validateChecksum(header):
        h = np.array(header, np.uint8).view(np.uint16)
        a = np.uint32(np.sum(h))

        b = np.array(a, np.uint32)

        dt = np.dtype([('f1', np.uint16), ('f2', np.uint16)])
        b = b.view(dtype=dt)

        c = b['f1'] + b['f2']
        rr = np.invert(c)
        return rr

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

    def computeH(self, s):
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

    def getDim(self, G, C, A):
        o, D, V = np.linalg.svd(A / G)

        a = 1 / math.sqrt(D[0])
        b = 1 / math.sqrt(D[1])
        c = 1 / math.sqrt(D[2])

        return max([a, b])

    def run(self):
        while True:
            while self.lostSync == 0:
                frameStart = time.time()
                timestamp = frameStart
                # packetLength = dataPort.in_waiting

                if self.gotHeader == 0:
                    self.rxHeader = self.getData(np.uint8, self.frameHeaderLengthInBytes)
                    self.byteCount = self.rxHeader.size * self.rxHeader.itemsize

                start = (time.time() - frameStart) * 1000

                magicBytes = np.array(self.rxHeader[0:8], np.uint8)
                magicBytes = magicBytes.view(np.uint64)

                if magicBytes != syncPatternUINT64:
                    # print('REMOVE\n')
                    self.lostSync = 1
                    break

                if self.byteCount != self.frameHeaderLengthInBytes:
                    self.lostSync = 1
                    break

                if self.validateChecksum(self.rxHeader) != 0:
                    self.lostSync = 1
                    break

                # print('CONITNUE')
                # exit()
                self.offset = self.initFrameHeaderStruct()

                if self.gotHeader == 1:
                    # print(frameHeaderStructType['frameNumber'].astype(np.uint8).view(np.uint32))
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

                if dataLength > 0:
                    self.rxData = self.getData(np.uint8, dataLength)
                    self.byteCount = self.rxData.size * self.rxData.itemsize
                    # print(rxData.size)
                    # print(dataLength)
                    if self.byteCount != float(dataLength):
                        self.lostSync = 1
                        break

                    self.offset = 0

                    for nTlv in range(frameHeaderStructType['numTLVs'].astype(np.uint8).view(np.uint16).__int__()):
                        tlvType = np.array(self.rxData[self.offset + 0:self.offset + 4], np.uint8).view(np.uint32)
                        tlvLength = np.array(self.rxData[self.offset + 4:self.offset + 8], np.uint8).view(np.uint32)
                        # print(tlvLength)
                        if tlvLength + self.offset > dataLength:
                            self.lostSync = 1
                            break

                        self.offset += self.tlvHeaderLengthInBytes

                        valueLength = tlvLength.__int__() - self.tlvHeaderLengthInBytes
                        # print(tlvHeaderLengthInBytes)
                        if tlvType.__int__() == 6:
                            self.numInputPoints = valueLength / self.pointLengthInBytes
                            # print(valueLength)
                            # print(pointLengthInBytes)
                            if self.numInputPoints > 0:
                                p = np.array(self.rxData[self.offset: self.offset + valueLength], np.uint8).view(
                                    np.single)
                                # print(p)
                                ppp = int(p.size / 4)
                                # TODO ppp prepracovanie
                                self.pointCloud = p.reshape(4, ppp)

                                # Convert degrees to radians
                                self.pointCloud[1, :] = self.pointCloud[1, :] * np.pi / 180
                                # posAll = [np.dot(pointCloud[0, :], np.sin(pointCloud[1, :])),
                                #           np.dot(pointCloud[0, :], np.cos(pointCloud[1, :]))]
                                self.posAll = [self.pointCloud[0, :] * np.sin(self.pointCloud[1, :]),
                                               self.pointCloud[0, :] * np.cos(self.pointCloud[1, :])]
                                snrAll = self.pointCloud[3, :]

                                # remove out of range, todo

                                staticInd = (self.pointCloud[2, :] == 0)
                                clutterInd = np.in1d(np.transpose(self.pointCloud[0:1, :]),
                                                     np.transpose(self.clutterPoints))
                                clutterInd = np.transpose(clutterInd) & staticInd

                                self.clutterPoints = self.pointCloud[0:2, staticInd]
                                self.pointCloud = self.pointCloud[0:3, ~clutterInd]
                                # print(self.pointCloud)
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
                pyplot.axis([-4, 4, 0, 10])
                # print(self.posAll)
                # print(snrAll)

                # if snrAll.all() * 10 > 0:
                # pyplot.scatter(self.posAll[0], self.posAll[1])
                # else:
                # self.lostSync = 1
                # break

                # pyplot.pause(0.05)
                # pyplot.draw()

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
                print('PEOPLE:', peopleCountTotal)

                if self.mIndex.shape[0]:
                    self.mIndex += 1

                # if np.size(self.point3D, axis=0):
                #     print(self.point3D[0], self.point3D[1])
                #     pyplot.scatter(self.point3D[0][0], self.point3D[1])

                for n in range(tNumC):
                    tid = TID[n] + 1

                    if tid > self.maxNumTracks:
                        self.lostSync = 1
                        break
                    try:
                        if self.mIndex.shape[0] & (self.mIndex.shape[0] == self.point3D.shape[1]):
                            # color
                            ind = (self.mIndex == tid)

                            for index in range(ind.size):
                                if ind[index]:
                                    pyplot.scatter(self.point3D[0, index], self.point3D[1, index])

                    except IndexError:
                        print('point3D out of range')

                    centroid = self.computeH(S[:, n])
                    ec = np.reshape(EC[:, n], (3, 3))

                    if (ec != 0).sum() > 1:
                        dim = self.getDim(1, centroid, ec)

                if self.posAll:
                    self.point3D = np.array((self.posAll[0], self.posAll[1], self.pointCloud[2, :]))

                pyplot.pause(0.05)
                pyplot.draw()
                # line = dataPort.read(65536)
                # dataFile.write(line)
                ##print(line)
                # time.sleep(1)
                # count -= 1
                break

                # if self.targetFrameNum:
                lostSyncTime = time.time()

            while self.lostSync:
                # print('here')
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
