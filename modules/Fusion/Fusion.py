import numpy as np

from modules.Config import Config
from modules.Fusion.FusionObject import FusionObject


def createMap(count):
    arr = []
    for i in range(0, count):
        x = []
        arr.append(x)

    return arr


class Fusion:
    def __init__(self, config: Config):
        self.config = config

    def fuse(self, picks, imageShape, radarData):

        if len(picks) == 0:
            return None

        picks = picks[picks[:, 3].argsort()]
        radarData = sorted(radarData, key=lambda k: k['distance'])

        radarX = self.config.radarX / self.config.fusionDelimiter
        X = imageShape[0] / self.config.fusionDelimiter
        a = createMap(self.config.fusionDelimiter)

        for t in picks:
            x = np.mean([t[0], t[2]])
            obj = FusionObject()
            obj.setDetection(True).setRect(t)
            a[int(x / X)].append(obj)

        for obj in radarData:
            iX = int((obj['x'] + self.config.shift) / radarX)
            if len(a[iX]) == 0:
                continue

            i = 0
            while a[iX][i].fused:
                i += 1

            if len(a[iX]) >= i:
                a[iX][i].setDistance(obj['distance']).setVelocity(obj['velocity']).setFusion(True)

        return a
