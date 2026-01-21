import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import socket
import requests
import netifaces
import time

SIGNAL_SERVER = "http://"
MY_NAME = "publisher01"
MY_PORT = 6000

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("No camera")

def get_zerotier_ip():
    for iface in netifaces.interfaces():
        if iface.startswith("zt"):
            return netifaces.ifaddresses(iface)[netifaces.AF_INET][0]["addr"]
    return None

class SenderNode(Node):
    def __init__(self):
        super().__init__("sender_node")

        # Getting ZeroTier ip
        self.my_ip = get_zerotier_ip()
        if not self.my_ip:
            raise RuntimeError("ZeroTier IP not found")
        self.get_logger().info(f"ZeroTier IP: {self.my_ip}")

        # 2) Registering signaling server
        requests.post(f"{SIGNAL_SERVER}/register", json={
            "name": MY_NAME,
            "ip": self.my_ip,
            "port": MY_PORT
        })
        self.get_logger().info(f"[I] {MY_NAME} has been registered")

        # Waiting for receiver node
        receiver_ip, receiver_port = self.wait_for_node("receiver01")
        receiver_ip2, receiver_port2 = self.wait_for_node("receiver02")

        # 4) Creating a socket and connecting
        self.sock1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock1.connect((receiver_ip, receiver_port))
        self.get_logger().info("Receiver01 connection OK")

        self.sock2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock2.connect((receiver_ip2, receiver_port2))
        self.get_logger().info("Receiver02 connection OK")

        self.pub_raw = self.create_publisher(Image, "/image/raw", 10)

        # Periodical message
        self.timer = self.create_timer(1.0, self.send_img)

    def wait_for_node(self, target_name):
        while True:
            try:
                nodes = requests.get(f"{SIGNAL_SERVER}/nodes").json()
                if target_name in nodes:
                    ip = nodes[target_name]["ip"]
                    port = nodes[target_name]["port"]
                    return ip, port
            except:
                pass
            self.get_logger().info(f"{target_name} waiting...")
            time.sleep(1)

    def send_msg(self):
        msg = "Hello over a channel which is not DDS!"
        self.sock1.sendall(msg.encode())
        self.get_logger().info("Message sent")
        self.sock2.sendall(msg.encode())
        self.get_logger().info("Message sent")

    def send_img(self):
        ret, frame = cap.read()
        if not ret:
            print("Ret error")
        
        ok, buf = cv2.imencode('.jpg', frame)
        if not ok:
            print("Ok error")
        
        data = buf.tobytes()
        packet = struct.pack("!I", len(data)) + data
        self.sock1.sendall(packet)
        print("Package sent to rec01!")
        self.sock2.sendall(packet)
        print("Package sent to rec02!")
        



def main():
    rclpy.init()
    node = SenderNode()
    rclpy.spin(node)
