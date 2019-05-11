import numpy as np

from modules.Radar.RadarObject import RadarObject


def createMap(count):
    arr = []
    for i in range(0, count):
        x = []
        arr.append(x)

    return arr


class Fusion:
    def fuse(self, picks, imageShape, radarData):

        if len(picks) == 0:
            return None

        test = picks

        test = test[test[:, 3].argsort()]
        radarData = sorted(radarData, key=lambda k: k['distance'])

        s = 10

        radarX = 12 / s

        X = imageShape[0] / s

        a = createMap(s)

        for t in test:
            x = np.mean([t[0], t[2]])
            obj = RadarObject()
            obj.setDetection(True).setRect(t)
            a[int(x / X)].append(obj)

        for obj in radarData:
            iX = int((obj['x'] + 6) / radarX)
            if len(a[iX]) == 0:
                continue

            i = 0
            while a[iX][i].fused:
                i += 1

            if len(a[iX]) >= i:
                a[iX][i].setDistance(obj['distance']).setVelocity(obj['velocity']).setFusion(True)

        return a
