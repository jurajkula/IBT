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
        # print('FUSION')
        if len(picks) == 0:
            return None
        picks = picks[picks[:, 3].argsort()]
        radarData = sorted(radarData, key=lambda k: k['distance'])
        radarX = self.config.radarX / self.config.fusionDelimiter
        X = imageShape[0] / self.config.fusionDelimiter
        a = createMap(self.config.fusionDelimiter + 1)

        for t in picks:
            x = np.mean([t[0], t[2]])
            obj = FusionObject()
            obj.setDetection(True).setRect(t)
            a[int(x / X)].append(obj)

        fusedCount = 0

        for obj in radarData:
            iX = int((obj['x'] + self.config.shift) / radarX)
            if len(a[iX]) == 0:
                continue
            i = 0
            while a[iX][i].fused:
                i += 1
                if len(a[iX]) == i:
                    break
            if len(a[iX]) > i:
                a[iX][i].setDistance(obj['distance']).setVelocity(obj['velocity']).setFusion(True)
                fusedCount += 1

        for ind in range(len(a)):
            for obj in a[ind]:
                if len(a) == ind + 1:
                    break

                if not obj.fused:
                    continue

                for cobj in a[ind+1]:
                    objD = np.mean(obj.rect[1], obj.rect[2])
                    cobjD = np.mean(cobj.rect[1], cobj.rect[2])

                    if objD < cobjD & obj['distance'] > cobj['distance']:
                        obj.setFusion(False)

                    if objD < cobjD & obj['distance'] > cobj['distance']:
                        cobj.setFusion(False)

        arr = [a, fusedCount]
        return arr
