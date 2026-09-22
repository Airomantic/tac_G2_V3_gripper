"""Synthetic SDK used only by tests that explicitly add this fixture directory."""

import time
from types import SimpleNamespace

import numpy as np


Mode = SimpleNamespace(STANDARD="standard")
SensorOptions = SimpleNamespace


class Sensor:
    def __init__(self, options):
        self.options = options
        self.sequence = 0
        if options.dev_id == "hang_start":
            time.sleep(30)

    def getEvents(self):
        return None

    def getDevStatus(self):
        return 0

    def wait_for_new(self, sequence, timeout_ms=20):
        time.sleep(0.005)
        if self.options.dev_id == "stall" and self.sequence >= 5:
            return False
        self.sequence += 1
        return True

    def getDeformation2D(self):
        flow = np.zeros((4, 4, 2), np.float32)
        if self.options.dev_id == "bad" and self.sequence > 3:
            flow[0, 0, 0] = np.nan
        return self.sequence, flow

    def getDepth(self):
        return self.sequence, np.zeros((4, 4), np.float32)

    def disconnect(self):
        if self.options.dev_id == "disconnect_error":
            raise RuntimeError("Synthetic disconnect failure")
