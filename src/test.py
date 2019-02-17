import serial, time
from struct import *
import numpy as np

dataPort = serial.Serial('/dev/ttyACM1', 921600, timeout=0.2)
controlPort = serial.Serial('/dev/ttyACM0', 115200, timeout=0.2)
config = open("./mmw_pplcount_demo_default.cfg", "r")
dataFile = open("./data.out", "rb")


def lengthFromStruct(S):
    length = 0
    for item in S[0]:
        length += item.itemsize
    return length


# def validateChecksum1(header):
#     print(header)
#     h = np.array(header, np.uint8).view(np.uint16)
#     print(h)
#     a = [np.uint32(np.sum(h))]
#     print(a)
#     b = np.array(a, np.uint32).astype(np.uint16)
#     b = np.uint16(np.sum(b))
#     print(b)
#
#     rr = np.invert(b)
#     print(rr)
#     return rr

def validateChecksum(header):
    h = np.array(header, np.uint8).view(np.uint16)
    a = np.uint32(np.sum(h))

    b = np.array(a, np.uint32)

    dt = np.dtype([('f1', np.uint16), ('f2', np.uint16)])
    b = b.view(dtype=dt)

    c = b['f1'] + b['f2']
    rr = np.invert(c)
    return rr

def sendData(data):
    data += "\n"
    controlPort.write(data.encode())
    time.sleep(1)
    echo = controlPort.readline()
    time.sleep(1)
    done = controlPort.readline()
    time.sleep(1)
    prompt = controlPort.readline()
    time.sleep(1)
    print(echo)
    print(done)
    print(prompt)


for line in config:
    if line[0] == '%':
        continue
    line = line.strip()
    print(line)
    #sendData(line)

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

frameHeaderLengthInBytes = lengthFromStruct(frameHeaderStructType)
tlvHeaderLengthInBytes = lengthFromStruct(tlvHeaderStruct)
pointLengthInBytes = lengthFromStruct(pointStruct)
targetLengthInBytes = lengthFromStruct(targetStruct)
indexLengthInBytes = 1
lostSync = 0
clutterPoints = np.zeros([2, 1])
count = 1000
outOfSyncBytes = 0
gotHeader = 0
rxHeader = 0
byteCount = 0
targetFrameNum = 0
frameNum = 0

while True:
    while lostSync == 0:
        frameStart = time.time()
        timestamp = frameStart
        # packetLength = dataPort.in_waiting

        if gotHeader == 0:
            rxHeader = np.fromfile(dataFile, np.uint8, frameHeaderLengthInBytes)
            #rxHeader = dataPort.read(frameHeaderLengthInBytes)
            #rxHeader = np.frombuffer(rxHeader, dtype=np.uint8)
            byteCount = rxHeader.size * rxHeader.itemsize
            print('HLAVICKA')
            print(rxHeader)
            print(byteCount)
            print('END')

        start = (time.time() - frameStart) * 1000

        magicBytes = np.array(rxHeader[0:8], np.uint8)
        magicBytes = magicBytes.view(np.uint64)

        if magicBytes != syncPatternUINT64:
            print('REMOVE\n')
            lostSync = 1
            break

        if byteCount != frameHeaderLengthInBytes:
            lostSync = 1
            break

        if validateChecksum(rxHeader) != 0:
            lostSync = 1
            break

        print('CONITNUE')
        #exit()
        offset = 0
        frameHeaderStructType['sync'] = np.array(rxHeader[offset:offset + frameHeaderStructType['sync'].itemsize],
                                                 np.uint8)
        offset += frameHeaderStructType['sync'].itemsize
        frameHeaderStructType['version'] = np.array(rxHeader[offset:offset + frameHeaderStructType['version'].itemsize],
                                                    np.uint8)
        offset += frameHeaderStructType['version'].itemsize
        frameHeaderStructType['platform'] = np.array(
            rxHeader[offset:offset + frameHeaderStructType['platform'].itemsize], np.uint8)
        offset += frameHeaderStructType['platform'].itemsize
        frameHeaderStructType['timestamp'] = np.array(
            rxHeader[offset:offset + frameHeaderStructType['timestamp'].itemsize], np.uint8)
        offset += frameHeaderStructType['timestamp'].itemsize
        frameHeaderStructType['packetLength'] = np.array(
            rxHeader[offset:offset + frameHeaderStructType['packetLength'].itemsize], np.uint8)
        offset += frameHeaderStructType['packetLength'].itemsize
        frameHeaderStructType['frameNumber'] = np.array(
            rxHeader[offset:offset + frameHeaderStructType['frameNumber'].itemsize], np.uint8)
        offset += frameHeaderStructType['frameNumber'].itemsize
        frameHeaderStructType['subframeNumber'] = np.array(
            rxHeader[offset:offset + frameHeaderStructType['subframeNumber'].itemsize], np.uint8)
        offset += frameHeaderStructType['subframeNumber'].itemsize
        frameHeaderStructType['chirpMargin'] = np.array(
            rxHeader[offset:offset + frameHeaderStructType['chirpMargin'].itemsize], np.uint8)
        offset += frameHeaderStructType['chirpMargin'].itemsize
        frameHeaderStructType['frameMargin'] = np.array(
            rxHeader[offset:offset + frameHeaderStructType['frameMargin'].itemsize], np.uint8)
        offset += frameHeaderStructType['frameMargin'].itemsize
        frameHeaderStructType['uartSentTime'] = np.array(
            rxHeader[offset:offset + frameHeaderStructType['uartSentTime'].itemsize], np.uint8)
        offset += frameHeaderStructType['uartSentTime'].itemsize
        frameHeaderStructType['trackProcessTime'] = np.array(
            rxHeader[offset:offset + frameHeaderStructType['trackProcessTime'].itemsize], np.uint8)
        offset += frameHeaderStructType['trackProcessTime'].itemsize
        frameHeaderStructType['numTLVs'] = np.array(rxHeader[offset:offset + frameHeaderStructType['numTLVs'].itemsize],
                                                    np.uint8)
        offset += frameHeaderStructType['numTLVs'].itemsize
        frameHeaderStructType['checksum'] = np.array(
            rxHeader[offset:offset + frameHeaderStructType['checksum'].itemsize], np.uint8)
        offset += frameHeaderStructType['checksum'].itemsize

        print(frameHeaderStructType['numTLVs'].astype(np.uint8).view(np.uint16))
        print(frameHeaderStructType['version'].astype(np.uint8).view(np.uint32))
        print(frameHeaderStructType['packetLength'].astype(np.uint8).view(np.uint32))
        print(frameHeaderStructType.view())
        print(frameHeaderLengthInBytes)

        if gotHeader == 1:
            print(frameHeaderStructType['frameNumber'].astype(np.uint8).view(np.uint32))
            if frameHeaderStructType['frameNumber'].astype(np.uint8).view(np.uint32) > targetFrameNum:
                targetFrameNum = frameHeaderStructType['frameNumber']
                gotHeader = 0
            else:
                gotHeader = 0
                lostSync = 1
                break

        targetFrameNum = frameHeaderStructType['frameNumber']

        dataLength = frameHeaderStructType['packetLength'].astype(np.uint8).view(
            np.uint32).__int__() - frameHeaderLengthInBytes

        if dataLength > 0:
            rxData = np.fromfile(dataFile, np.uint8, dataLength)
            byteCount = rxData.size * rxData.itemsize
            print(rxData.size)
            print(dataLength)
            if byteCount != float(dataLength):
                lostSync = 1
                break

            offset = 0

            for nTlv in range(frameHeaderStructType['numTLVs'].astype(np.uint8).view(np.uint16).__int__()):
                tlvType = np.array(rxData[offset + 0:offset + 4], np.uint8).view(np.uint32)
                tlvLength = np.array(rxData[offset + 4:offset + 8], np.uint8).view(np.uint32)
                print(tlvLength)
                if tlvLength + offset > dataLength:
                    lostSync = 1
                    break

                offset += tlvHeaderLengthInBytes

                valueLength = tlvLength.__int__() - tlvHeaderLengthInBytes
                print(tlvHeaderLengthInBytes)
                if tlvType.__int__() == 6:
                    numInputPoints = valueLength / pointLengthInBytes
                    print(valueLength)
                    print(pointLengthInBytes)
                    if numInputPoints > 0:
                        p = np.array(rxData[offset: offset + valueLength], np.uint8).view(np.single)
                        print(p)
                        pointCloud = p.reshape(4, numInputPoints.__int__())
                        pointCloud[1, :] = pointCloud[1, :] * np.pi / 180

                        posAll = [np.dot(pointCloud[0, :], np.sin(pointCloud[1, :])),
                                  np.dot(pointCloud[0, :], np.cos(pointCloud[1, :]))]
                        snrAll = pointCloud[3, :]

                        # remove out of range, todo

                        staticInd = (pointCloud[2, :] == 0)
                        clutterInd = np.in1d(np.transpose(pointCloud[0:1, :]), np.transpose(clutterPoints))
                        clutterInd = np.transpose(clutterInd) & staticInd

                        clutterPoints = pointCloud[0:1, staticInd]
                        pointCloud = pointCloud[0:2, ~clutterInd]
        # line = dataPort.read(65536)
        # dataFile.write(line)
        ##print(line)
        # time.sleep(1)
        # count -= 1
        break

    if targetFrameNum:
        lostSyncTime = time.time()

    while lostSync:
        # print('here')
        n = 0
        for n in range(8):
            rxByte = np.fromfile(dataFile, np.uint8, 1)

            if rxByte != syncPatternUINT8[n]:
                outOfSyncBytes = outOfSyncBytes + 1
                break

        if n == 7:
            lostSync = 0

            frameNum = frameNum + 1
            if frameNum > 10000:
                frameNum = 1

            header = np.fromfile(dataFile, np.uint8, frameHeaderLengthInBytes - 8)

            byteCount = header.size * header.itemsize
            header = header.tolist()
            rxHeader = np.array([np.transpose(syncPatternUINT8)]).tolist()[0]
            rxHeader += header

            byteCount = byteCount + 8
            gotHeader = 1

    # break

# dataPort.close()
# controlPort.close()
