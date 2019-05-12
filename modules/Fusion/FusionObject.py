class FusionObject:
    def __init__(self):
        self.detected = False
        self.fused = False
        self.distance = 0
        self.velocity = 0
        self.rect = [0, 0, 0, 0]

    def setDetection(self, detection):
        self.detected = detection
        return self

    def setDistance(self, distance):
        self.distance = distance
        return self

    def setVelocity(self, velocity):
        self.velocity = velocity
        return self

    def setRect(self, rect):
        self.rect = rect
        return self

    def setFusion(self, fused):
        self.fused = fused
        return self

    def print(self):
        print([{
            'detection': self.detected,
            'fusion': self.fused,
            'distance': self.distance,
            'velocity': self.velocity,
            'rects': self.rect
        }])