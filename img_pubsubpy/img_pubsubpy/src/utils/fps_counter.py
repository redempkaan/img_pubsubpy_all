import time

class FpsCounter:
    def __init__(self, name="FPS", interval=1.0):
        self.name = name
        self.interval = interval
        self.last_time = time.time()
        self.frames = 0

    def step(self):
        self.frames += 1
        now = time.time()
        if (now - self.last_time) >= self.interval:
            fps = self.frames / (now - self.last_time)
            print(f"[{self.name}] {fps:.2f} FPS")
            self.frames = 0
            self.last_time = now
