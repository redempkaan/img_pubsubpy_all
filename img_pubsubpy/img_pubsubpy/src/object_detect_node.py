import rclpy
from rclpy.node import Node
import socket
import netifaces
import requests
import time
import cv2
import os
import struct
import numpy as np
import queue
import threading
from img_pubsubpy.utils.fps_counter import FpsCounter
from img_pubsubpy.utils.edge1_object_detect import process_jpeg, process_and_draw

SIGNAL_SERVER = "server_ip"
MY_NAME = "receiver01"
MY_LISTEN_PORT = 7000
MY_PUBLISH_PORT = 8001
MY_PUB_TOPIC = "/det/object"
MY_SUB_TOPIC = "/img/raw"

resp = requests.get(
    f"{SIGNAL_SERVER}/topic",
    params={"name": MY_PUB_TOPIC}
)

if resp.status_code != 200:
    raise RuntimeError("Signaling server error")

node_info = resp.json()

subs = node_info.get("subscribers", [])
if not subs:
    raise RuntimeError("No subscribers found for topic")

drone = subs[0]

DRONE_IP = drone["ip"]
DRONE_SEND_PORT = drone["port"]

def get_zerotier_ip():
    for iface in netifaces.interfaces():
        if iface.startswith("zt"):
            return netifaces.ifaddresses(iface)[netifaces.AF_INET][0]["addr"]
    return None

class ReceiverNode(Node):
    def __init__(self):
        super().__init__("receiver_node")

        self.my_ip = get_zerotier_ip()
        if not self.my_ip:
            raise RuntimeError("ZeroTier IP not found")

        self.get_logger().info(f"Receiver ZeroTier IP: {self.my_ip}")
        
        self.fps_counter = FpsCounter("EDGE1 RECEIVER")

        requests.post(f"{SIGNAL_SERVER}/register", json={
	    "name": MY_NAME,
	    "ip": self.my_ip,
	    "node_type": "edge",
	    "topics": {
		"subscribe": {
		    "/img/raw": {
		        "port": MY_LISTEN_PORT,
		        "protocol": "tcp",
		        "datatype": "jpeg"
		    }
		},
		"publish": {
		    MY_PUB_TOPIC: {
		        "port": MY_PUBLISH_PORT,
		        "protocol": "tcp",
		        "datatype": "json"
		    }
		}
	    }
	})

        self.get_logger().info("Signaling server register complete")

      
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((self.my_ip, MY_LISTEN_PORT))
        self.server.listen(5)
        
        self.det_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.det_sock.connect((DRONE_IP, DRONE_SEND_PORT))
        self.get_logger().info("Drone send connection success")

        self.get_logger().info(f"Waiting for connection on {self.my_ip}:{MY_LISTEN_PORT}")

        
        self.conn, self.addr = self.server.accept()
        self.get_logger().info(f"Connection established: {self.addr}")
        
        self.frame_queue = queue.Queue(maxsize=2)

        self.recv_thread = threading.Thread(target=self.recv_loop, daemon=True)
        self.proc_thread = threading.Thread(target=self.process_loop, daemon=True)

        self.recv_thread.start()
        self.proc_thread.start()
    
    def recv_exact(self, conn, n):
        data = b""
        while len(data) < n:
            part = conn.recv(n - len(data))
            if  not part:
                return None
            data += part
        return data
    

    def recv_loop(self):
        while True:
            hdr = self.recv_exact(self.conn, 8)
            if not hdr:
                print("Drone disconnected")
                break

            frame_id, length = struct.unpack("!II", hdr)
            self.t_recv_ns = time.monotonic_ns()
            jpeg_bytes = self.recv_exact(self.conn, length)
            if not jpeg_bytes:
                continue

            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass

            self.frame_queue.put((frame_id, jpeg_bytes))
            
    def process_loop(self):
        while True:
            frame_id, jpeg_bytes = self.frame_queue.get()

            temp_img, output_json = process_jpeg(jpeg_bytes)

            json_bytes = output_json.encode("utf-8")
            packet = struct.pack("!II", frame_id, len(json_bytes)) + json_bytes
            self.t_send_ns = time.monotonic_ns()
            print(f"Process Latency=  {(self.t_send_ns - self.t_recv_ns) / 1_000_000}ms")

            self.det_sock.sendall(packet)

            self.fps_counter.step()
            print(f"[EDGE] JSON sent for frame id:{frame_id}")


     


def main():
    rclpy.init()
    node = ReceiverNode()
    rclpy.spin(node)
