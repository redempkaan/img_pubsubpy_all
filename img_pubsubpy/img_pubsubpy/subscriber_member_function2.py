import rclpy
from rclpy.node import Node
import socket
import netifaces
import requests
import time
import cv2
import os

SIGNAL_SERVER = "http://"
MY_NAME = "receiver02"
MY_PORT = 6001

SAVE_FOLDER = "received_images"
os.makedirs(SAVE_FOLDER, exist_ok=True)

img_index = 0

def get_zerotier_ip():
    """ZeroTier interface IP adresini döndürür."""
    for iface in netifaces.interfaces():
        if iface.startswith("zt"):
            return netifaces.ifaddresses(iface)[netifaces.AF_INET][0]["addr"]
    return None

class ReceiverNode(Node):
    def __init__(self):
        super().__init__("receiver_node")

        # 1) ZeroTier IP al
        self.my_ip = get_zerotier_ip()
        if not self.my_ip:
            raise RuntimeError("ZeroTier IP bulunamadı! ZeroTier çalışıyor mu?")

        self.get_logger().info(f"Receiver ZeroTier IP: {self.my_ip}")

        # 2) Signaling server'a kaydol
        requests.post(f"{SIGNAL_SERVER}/register", json={
            "name": MY_NAME,
            "ip": self.my_ip,
            "port": MY_PORT
        })

        self.get_logger().info("Signaling server'a kayıt tamamlandı.")

        # 3) TCP server oluştur
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((self.my_ip, MY_PORT))
        self.server.listen(5)

        self.get_logger().info(f"{self.my_ip}:{MY_PORT} üzerinde bağlantı bekleniyor...")

        # Blocking accept (ilk bağlantı)
        self.conn, self.addr = self.server.accept()
        self.get_logger().info(f"Bağlantı sağlandı: {self.addr}")

        # 4) Her 0.5 saniyede mesaj dinle
        self.timer = self.create_timer(0.5, self.get_img)

    def read_msg(self):
        try:
            data = self.conn.recv(1024)
            if data:
                self.get_logger().info(f"Gelen mesaj: {data.decode()}")
        except:
            pass
    
    def recv_exact(conn, n):
        data = b""
        while len(data) < n:
            part = conn.recv(n - len(data))
            if  not part:
                return None
            data += part
        return data
    
    def get_img(self):
        hdr = recv_exact(conn, 4)
        if not hdr:
            print("Drone connection lost")
        
        (length,) = struct.unpack("!I", hdr)

        data = recv_exact(conn, length)
        if not data:
            print("Data read error")
        
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)

        if img is None:
            print("JPEG decode error")

        filename = f"received_images/frame_{img_index}.jpg"
        cv2.imwrite(filename, img)

        print(f"Image saved -> {filename}")
        img_index += 1



def main():
    rclpy.init()
    node = ReceiverNode()
    rclpy.spin(node)
