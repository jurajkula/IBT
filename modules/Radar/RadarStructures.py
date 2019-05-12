import numpy as np


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
