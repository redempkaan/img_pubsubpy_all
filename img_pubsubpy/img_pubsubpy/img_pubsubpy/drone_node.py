import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path, Odometry
from cv_bridge import CvBridge
import cv2
import socket
import requests
import netifaces
import time
import struct
import threading
import json
import numpy as np
import math
from img_pubsubpy.utils.fps_counter import FpsCounter
from img_pubsubpy.utils.frame_cache import FrameCache

SIGNAL_SERVER = "signaling_server"
MY_NAME = "publisher01"
PUBLISH_PORT = 7000
DETECTION_RECEIVE_PORT = 8001
POSE_RECEIVE_PORT = 8002
BARCODE_RECEIVE_PORT = 8003
FRAME_RATE = 50.0
MAX_CACHE_SIZE = 30
MAX_POSES = 1000
PUB_TOPICS = ["/img/raw"]
SUB_TOPICS = ["/det/object", "/slam/pose", "/barcode/result"]

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("No camera")

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)

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

        self.fps_counter = FpsCounter("DRONE SEND")
        self.fps_counter2 = FpsCounter("DRONE YOLO RECEIVE")
        self.fps_counter3 = FpsCounter("DRONE ORB-SLAM3 RECEIVE")
        self.fps_counter4 = FpsCounter("DRONE BARCODE RECEIVE")
        self.frame_cache = FrameCache()
        
        self.frame_id = 0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        self.edge_index = 0



        # Registering signaling server
        requests.post(f"{SIGNAL_SERVER}/register", json={
        "name": "drone_01",
        "ip": self.my_ip,
        "node_type": "drone",
        "topics": {
            "publish": {
                "/img/raw": {
                    "port": PUBLISH_PORT,
                    "protocol": "tcp",
                    "datatype": "jpeg"
                }
            },
            "subscribe": {
                "/det/object": {
                    "port": DETECTION_RECEIVE_PORT,
                    "protocol": "tcp",
                    "datatype": "json"
                },
                "/slam/pose": {
                    "port": POSE_RECEIVE_PORT,
                    "protocol": "tcp",
                    "datatype": "json"
                },
                "/image/barcode": {
                    "port": BARCODE_RECEIVE_PORT,
                    "protocol": "tcp",
                    "datatype": "json"
                }
            }
        }
    })
        self.get_logger().info(f"[I] {MY_NAME} has been registered")

        
        self.pub_raw = self.create_publisher(Image, "/image/raw", 10)
        self.pub_det = self.create_publisher(String, "/detections", 10)
        self.pub_det_img = self.create_publisher(Image, "/image/detections", 10)
        self.pub_barcode = self.create_publisher(String, "/barcode", 10)
        self.pub_barcode_img = self.create_publisher(Image, "/image/barcode", 10)
        self.pub_pose = self.create_publisher(String, "/slam/pose", 10)
        self.pub_path = self.create_publisher(Path, "/slam/path", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)


        self.recv_thread = threading.Thread(target=self.recv_loop, daemon=True)
        self.recv_thread.start()
        
        self.recv_thread2 = threading.Thread(target=self.recv_loop2, daemon=True)
        self.recv_thread2.start()

        self.recv_thread3 = threading.Thread(target=self.recv_loop3, daemon=True)
        self.recv_thread3.start()
        

        receiver_ip, receiver_port = self.wait_for_edge(PUB_TOPICS[0], self.edge_index)
        self.get_logger().info(f"{receiver_ip} : {receiver_port}")

        # Creating a socket and connecting
        self.sock1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock1.connect((receiver_ip, receiver_port))

        self.get_logger().info("Receiver01 connection OK")

        self.edge_index += 1

        receiver_ip2, receiver_port2 = self.wait_for_edge(PUB_TOPICS[0], self.edge_index)
        self.get_logger().info(f"{receiver_ip2} : {receiver_port2}")



        self.sock2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock2.connect((receiver_ip2, receiver_port2))

        self.get_logger().info("Receiver02 connection OK")
        self.edge_index += 1

        receiver_ip3, receiver_port3 = self.wait_for_edge(PUB_TOPICS[0], self.edge_index)
        self.get_logger().info(f"{receiver_ip3} : {receiver_port3}")


        self.sock3 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock3.connect((receiver_ip3, receiver_port3))

        self.get_logger().info("Receiver03 connection OK")
        self.edge_index += 1

        self.bridge = CvBridge()

        # Periodical message
        self.timer = self.create_timer(1.0 / FRAME_RATE, self.send_img)
    
    def send_img(self):
        frame_id = self.frame_id
        self.frame_id += 1

        ret, frame = cap.read()
        if not ret:
            print("Ret error")
        
        img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.pub_raw.publish(img_msg)
        
        ok, buf = cv2.imencode('.jpg', frame)
        if not ok:
            print("Ok error")
        
        jpeg_bytes = buf.tobytes()
        self.frame_cache.put(frame_id, jpeg_bytes)

        packet = struct.pack("!II", frame_id, len(jpeg_bytes)) + jpeg_bytes
        self.sock1.sendall(packet)
        self.sock2.sendall(packet)
        self.sock3.sendall(packet)
        self.fps_counter.step()
    
    def wait_for_edge(self, target_topic, index):
        while True:
            try:
                result = requests.get(
                    f"{SIGNAL_SERVER}/topic",
                    params={"name": target_topic}
                ).json()

                self.get_logger().info(str(result))

                pub = result.get("subscribers", [])
                edge = pub[index]
                return edge["ip"], edge["port"]

            except Exception as e:
                self.get_logger().warn(str(e))

            self.get_logger().info(f"Waiting for edge node {index + 1} to publish {target_topic}")
            time.sleep(1)

    def wait_for_node(self, target_topic):
        while True:
            try:
                result = requests.get(f"{SIGNAL_SERVER}/topic", params={"name" : target_topic}).json()
                ip = result["publishers"][0]["ip"]
                port = result["publishers"][0]["port"]
                return ip, port
                
            except:
                pass
            self.get_logger().info(f"Publisher for {target_topic} waiting...")
            time.sleep(1)

    def wait_for_node2(self, target_name):
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

    def recv_exact(self, sock, n):
        data = b""
        while len(data) < n:
            part = sock.recv(n - len(data))
            if not part:
                return None
            data += part
        return data

    def recv_loop(self):
        self.sock_detected = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_detected.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_detected.bind((self.my_ip, DETECTION_RECEIVE_PORT))
        self.sock_detected.listen(1)
        self.conn_detected, self.addr_detected = self.sock_detected.accept()

        self.get_logger().info(f"Recv loop start")

        while True:
            hdr = self.recv_exact(self.conn_detected, 8)
            if not hdr: 
                continue

            frame_id, length = struct.unpack("!II", hdr)
            json_bytes = self.recv_exact(self.conn_detected, length)
            self.t_recv_ns1 = time.monotonic_ns()
            json_bytes_decoded = json_bytes.decode("utf-8")

            json_detections = json.loads(json_bytes_decoded)
            
            jpeg_bytes = self.frame_cache.get(frame_id)
            #print(f"Object Det. E2E Latency= [FRAME {frame_id}]  {(self.t_recv_ns1 - self.frame_cache.get_ts(frame_id)) / 1_000_000}ms")

            if jpeg_bytes is None:
                self.get_logger().warn(f"Frame {frame_id} cache miss")
                continue
            
            
            self.handle_detection(jpeg_bytes, json_detections)

            msg = String()
            msg.data = json_bytes_decoded
            self.pub_det.publish(msg)
            #Getting frame from cache and drawing boxes
            #self.fps_counter2.step()

    def recv_loop2(self):
        self.sock_mapping = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_mapping.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_mapping.bind((self.my_ip, POSE_RECEIVE_PORT))
        self.sock_mapping.listen(1)
        self.conn_detected2, self.addr_detected2 = self.sock_mapping.accept()

        self.get_logger().info(f"Recv loop2 start")

        while True:
            hdr = self.recv_exact(self.conn_detected2, 8)
            if not hdr: 
                continue
            #self.get_logger().info(f"Header received")
            frame_id, length = struct.unpack("!II", hdr)
            json_bytes = self.recv_exact(self.conn_detected2, length)
            #self.fps_counter3.step()
            self.t_recv_ns2 = time.monotonic_ns()
            #print(f"ORB-SLAM3 E2E Latency= [FRAME {frame_id}]  {(self.t_recv_ns2 - self.frame_cache.get_ts(frame_id)) / 1_000_000}ms")
            json_bytes_decoded = json_bytes.decode("utf-8")
            #self.get_logger().info(f"Head unpacked and decoded")
            msg = String()
            msg.data = json_bytes_decoded
            self.pub_pose.publish(msg)

            data = json.loads(json_bytes_decoded)
            self.add_pose_to_path2(data)
            
            #self.update_odometry(dx=float(data["dx"]), dy=float(data["dy"]), dyaw = float(data["dyaw"]))
            #self.publish_odometry(self.x, self.y, self.yaw, 0, 0, 0)


    def recv_loop3(self):
        self.sock_barcode = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_barcode.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_barcode.bind((self.my_ip, BARCODE_RECEIVE_PORT))
        self.sock_barcode.listen(1)
        self.conn_detected3, self.addr_detected3 = self.sock_barcode.accept()

        self.get_logger().info(f"Recv loop3 start")

        while True:
            hdr = self.recv_exact(self.conn_detected3, 8)
            if not hdr: 
                continue
            #self.get_logger().info(f"Header received")
            frame_id, length = struct.unpack("!II", hdr)
            json_bytes = self.recv_exact(self.conn_detected3, length)
            #self.fps_counter4.step()

            self.t_recv_ns3 = time.monotonic_ns()
            #print(f"Barcode Detect. E2E Latency= [FRAME {frame_id}]  {(self.t_recv_ns3 - self.frame_cache.get_ts(frame_id)) / 1_000_000}ms")
            json_bytes_decoded = json_bytes.decode("utf-8")
            #self.get_logger().info(f"Head unpacked and decoded")
            

            msg = String()
            msg.data = json_bytes_decoded
            self.pub_barcode.publish(msg)

            self.draw_barcode_on_frame(frame_id, json_bytes_decoded)


    def draw_barcode_on_frame(self, frame_id, result_json):
        parsed_json = json.loads(result_json)

        text = f"{parsed_json["type"]} : {parsed_json["barcode"]}"

        jpeg_bytes = self.frame_cache.get(frame_id)

        if jpeg_bytes is None:
            self.get_logger().warn(f"Frame {frame_id} cache miss")
            return -1
        
        img = cv2.imdecode(np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_COLOR)
        cv2.putText(img, text, (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)

        msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "barcode"

        self.pub_barcode_img.publish(msg)
        
    
    def handle_detection(self, jpeg_bytes, detections):
        img = cv2.imdecode(
            np.frombuffer(jpeg_bytes, np.uint8),
            cv2.IMREAD_COLOR
        )

        for d in detections["detections"]:
            x, y, w, h = map(int, [d["x"], d["y"], d["w"], d["h"]])
            cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

        msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_det_img.publish(msg)


    def update_odometry(self, dx, dy, dyaw):
        self.yaw += dyaw
        self.x += math.cos(self.yaw) * dx - math.sin(self.yaw) * dy
        self.y += math.sin(self.yaw) * dx + math.cos(self.yaw) * dy
    
    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2)
        q.w = math.cos(yaw / 2)

        return q
        

    def publish_odometry(self, x, y, yaw, vx, vy, vyaw):
        msg = Odometry()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        q = self.yaw_to_quaternion(yaw)
        msg.pose.pose.orientation = q

        self.pub_odom.publish(msg)

        self.add_pose_to_path(x, y, q)

        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_path.publish(self.path_msg)


    def add_pose_to_path(self, x, y, quaternion):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = quaternion.z
        pose.pose.orientation.w = quaternion.w

        self.path_msg.poses.append(pose)

    def add_pose_to_path2(self, data):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"

        pose.pose.position.x = data["position"]["x"]
        pose.pose.position.y = data["position"]["y"]
        pose.pose.position.z = data["position"]["z"]

        pose.pose.orientation.x = data["orientation"]["qx"]
        pose.pose.orientation.y = data["orientation"]["qy"]
        pose.pose.orientation.y = data["orientation"]["qz"]
        pose.pose.orientation.y = data["orientation"]["qw"]

        self.path_msg.poses.append(pose)
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_path.publish(self.path_msg)

        if len(self.path_msg.poses) > MAX_POSES:
            self.path_msg.poses.pop(0)





def main():
    rclpy.init()
    node = SenderNode()
    rclpy.spin(node)
