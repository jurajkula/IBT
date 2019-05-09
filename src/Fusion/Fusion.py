import numpy as np

from Radar.RadarObject import RadarObject


def createMap(count):
    arr = []
    for i in range(0, count):
        x = []
        for j in range (0, count):
            x.append(RadarObject())
        arr.append(x)

    return arr


class Fusion:
    def __init__(self, picks, imageShape):
        self.objects = [{'distance': 7.2, 'x': 9, 'velocity': 2}, {'distance': 7, 'x': 11, 'velocity': 1}]
        self.picks = picks
        self.shape = imageShape

    def fuse(self):
        # print(self.picks)

        test = self.picks
        test = test[test[:, 0].argsort()]

        print(self.objects)
        self.objects = sorted(self.objects, key=lambda k: k['x'])
        print(self.objects)

        s = 15

        radarX = 20 / s
        radarY = 16 / s

        X = self.shape[1] / s
        Y = self.shape[0] / s

        print(X, Y)
        print(radarX, radarY)

        a = createMap(s)
        b = np.zeros(shape=(s, s))

        for t in test:
            x = np.mean([t[0], t[2]])
            y = np.mean([t[1], t[3]])
            a[int(y / Y)][int(x / X)].setDetection(True).setRect(t)
            # print(json.dumps(data))

        for o in a:
            for oo in o:
                if oo.detected is not True:
                    continue

                oo.print()

        for obj in self.objects:
            iX = int(obj['distance'] / radarY)
            iY = int(obj['x'] / radarX)
            if a[iX][iY].detected:
                a[iX][iY].setDistance(obj['distance']).setVelocity(obj['velocity']).setFusion(True)

        for o in a:
            for oo in o:
                if oo.detected is not True:
                    continue

                oo.print()

        return a
