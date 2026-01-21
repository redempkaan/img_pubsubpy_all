import time
import threading
from collections import OrderedDict


class FrameCache:
    def __init__(self, max_size=50, ttl_sec=2.0):
        self.cache = OrderedDict()
        self.lock = threading.Lock()
        self.max_size = max_size
        self.ttl = ttl_sec

    def put(self, frame_id, jpeg_bytes):
        with self.lock:
            self.cache[frame_id] = {
                "jpeg": jpeg_bytes,
                "ts": time.monotonic_ns()
            }
            self.cache.move_to_end(frame_id)
            self._cleanup()

    def get(self, frame_id):
        with self.lock:
            entry = self.cache.get(frame_id)
            if entry:
                return entry["jpeg"]
            return None
        
    def get_ts(self, frame_id):
        with self.lock:
            entry = self.cache.get(frame_id)
            if entry:
                return entry["ts"]
            return None

    def _cleanup(self):
        now = time.time()
        to_delete = []

        for fid, entry in self.cache.items():
            if now - entry["ts"] > self.ttl:
                to_delete.append(fid)

        for fid in to_delete:
            self.cache.pop(fid, None)

        while len(self.cache) > self.max_size:
            self.cache.popitem(last=False)
