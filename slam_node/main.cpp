#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>
#include <chrono>
#include <cstring>

#include <ifaddrs.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sys/socket.h>

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include <opencv2/opencv.hpp>

// ORB-SLAM3
#include "System.h"

// -------------------- CONFIG --------------------
static const std::string SIGNAL_SERVER = "http://13.62.99.235:80";
static const std::string MY_NAME       = "receiver02";
static const int         MY_PORT       = 6002;

// Drone result socket
//static const std::string DRONE_IP      = "172.27.122.10";
//static const int         DRONE_SEND_PORT = 7002;

// Queue
static const size_t MAX_QUEUE = 5;


static inline uint64_t now_ns() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec) * 1'000'000'000ULL + ts.tv_nsec;
}

// -------------------- Getting ZeroTier IP --------------------
std::string get_zerotier_ip()
{
    struct ifaddrs *ifaddr = nullptr, *ifa = nullptr;
    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        return "";
    }

    std::string zt_ip;

    for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) continue;

        if (ifa->ifa_addr->sa_family == AF_INET) {
            if (strncmp(ifa->ifa_name, "zt", 2) == 0) {
                char ip[INET_ADDRSTRLEN];
                auto *sa = (struct sockaddr_in*)ifa->ifa_addr;
                inet_ntop(AF_INET, &(sa->sin_addr), ip, INET_ADDRSTRLEN);
                zt_ip = ip;
                break;
            }
        }
    }

    freeifaddrs(ifaddr);
    return zt_ip;
}

// -------------------- HTTP POST JSON --------------------
double now_sec()
{
    using clock = std::chrono::steady_clock;
    static auto t0 = clock::now();
    auto now = clock::now();
    return std::chrono::duration<double>(now - t0).count();
}

bool http_post_json(const std::string& url, const std::string& json_body)
{
    CURL* curl = curl_easy_init();
    if (!curl) return false;

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_body.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)json_body.size());
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);

    CURLcode res = curl_easy_perform(curl);

    long code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    if (res != CURLE_OK) {
        std::cerr << "HTTP POST failed: " << curl_easy_strerror(res) << "\n";
        return false;
    }
    if (code < 200 || code >= 300) {
        std::cerr << "HTTP POST non-2xx: " << code << "\n";
        return false;
    }
    return true;
}

// -------------------- HTTP GET ----------------------------
static size_t write_cb(void* contents, size_t size, size_t nmemb, void* userp)
{
    size_t total = size * nmemb;
    std::string* s = static_cast<std::string*>(userp);
    s->append((char*)contents, total);
    return total;
}

bool http_get(const std::string& url, std::string& response)
{
    CURL* curl = curl_easy_init();
    if (!curl) return false;

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);

    CURLcode res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);

    return (res == CURLE_OK);
}

// -------------------- socket recv exact --------------------
bool recv_exact(int sock, void* buf, size_t n)
{
    uint8_t* p = reinterpret_cast<uint8_t*>(buf);
    size_t got = 0;
    while (got < n) {
        ssize_t r = ::recv(sock, p + got, n - got, 0);
        if (r <= 0) return false;
        got += (size_t)r;
    }
    return true;
}

// -------------------- socket send all --------------------
bool send_all(int sock, const void* buf, size_t n)
{
    const uint8_t* p = reinterpret_cast<const uint8_t*>(buf);
    size_t sent = 0;
    while (sent < n) {
        ssize_t s = ::send(sock, p + sent, n - sent, 0);
        if (s <= 0) return false;
        sent += (size_t)s;
    }
    return true;
}

// -------------------- Frame struct --------------------
struct FramePacket {
    uint32_t frame_id;
    uint64_t t_rx_done_ns;
    std::vector<uint8_t> jpeg;
};

// Thread-safe small queue (drop-oldest)
class FrameQueue {
public:
    void push_drop_old(FramePacket&& pkt) {
        std::unique_lock<std::mutex> lk(m_);
        if (q_.size() >= MAX_QUEUE) {
            q_.pop_front();
        }
        q_.push_back(std::move(pkt));
        cv_.notify_one();
    }

    bool pop(FramePacket& out) {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [&]{ return !q_.empty() || stop_; });
        if (stop_) return false;
        out = std::move(q_.front());
        q_.pop_front();
        return true;
    }

    void stop() {
        std::unique_lock<std::mutex> lk(m_);
        stop_ = true;
        cv_.notify_all();
    }

private:
    std::mutex m_;
    std::condition_variable cv_;
    std::deque<FramePacket> q_;
    bool stop_ = false;
};

// -------------------- Pose -> JSON --------------------
static inline std::string pose_to_json(
    uint32_t frame_id,
    double x, double y, double z,
    double qx, double qy, double qz, double qw,
    const char* frame = "Twc")
{
    char buf[1024];
    std::snprintf(
        buf, sizeof(buf),
        "{"
          "\"frame_id\":%u,"
          "\"position\":{\"x\":%.9f,\"y\":%.9f,\"z\":%.9f},"
          "\"orientation\":{\"qx\":%.9f,\"qy\":%.9f,\"qz\":%.9f,\"qw\":%.9f},"
          "\"pose_frame\":\"%s\""
        "}",
        frame_id, x, y, z, qx, qy, qz, qw, frame
    );
    return std::string(buf);
}

// -------------------- MAIN --------------------
int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage:\n"
                  << "  " << argv[0] << " <ORBvoc.txt> <settings.yaml>\n\n"
                  << "Example:\n"
                  << "  " << argv[0] << " ~/ORB_SLAM3/Vocabulary/ORBvoc.txt ~/ORB_SLAM3/Examples/Monocular/TUM1.yaml 30\n";
        return -1;
    }

    const std::string vocab_path = argv[1];
    const std::string settings_path = argv[2];
    double t = now_sec();
    using json = nlohmann::json;


    // Finding ZeroTier IP
    std::string my_ip;
    std::cout << "[DEBUG] Searching ZeroTier interfaces...\n" << std::flush;
    for (int i = 0; i < 10; i++) {
        my_ip = get_zerotier_ip();
        if (!my_ip.empty()) break;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (my_ip.empty()) {
        std::cerr << "ZeroTier IP not found. ZeroTier join/up kontrol et.\n";
        return 1;
    }
    std::cout << "ZeroTier IP: " << my_ip << "\n";
    
    	std::string resp;
	std::string url = SIGNAL_SERVER + "/topic?name=/slam/pose";

	if (!http_get(url, resp)) {
	    std::cerr << "Failed to query topic info\n";
	    return -1;
	}

	json j = json::parse(resp);

	if (!j.contains("subscribers") || j["subscribers"].empty()) {
	    std::cerr << "[ERROR] No subscribers found for /slam/pose\n";
	    std::cerr << j.dump(2) << std::endl;
	    return -1;
	}

	auto drone = j["subscribers"][0];

	if (drone["ip"].is_null() || drone["port"].is_null()) {
	    std::cerr << "[ERROR] Drone ip or port is null\n";
	    std::cerr << drone.dump(2) << std::endl;
	    return -1;
	}

	std::string drone_ip = drone["ip"].get<std::string>();
	int drone_pose_port  = drone["port"].get<int>();

	std::cout << "[DISCOVERY] Drone IP=" << drone_ip
		  << " pose_port=" << drone_pose_port << std::endl;

    // Signaling server register
    curl_global_init(CURL_GLOBAL_ALL);
    {
        std::string url = SIGNAL_SERVER + "/register";
        std::string body =
	    std::string("{") +
	    "\"name\":\"" + MY_NAME + "\"," +
	    "\"ip\":\"" + my_ip + "\"," +
	    "\"node_type\":\"edge\"," +
	    "\"topics\":{" +
		"\"subscribe\":{" +
		    "\"/img/raw\":{" +
		        "\"port\":" + std::to_string(MY_PORT) + "," +
		        "\"protocol\":\"tcp\"," +
		        "\"datatype\":\"jpeg\"" +
		    "}" +
		"}," +
		"\"publish\":{" +
		    "\"/slam/pose\":{" +
		        "\"port\":" + std::to_string(drone_pose_port) + "," +
		        "\"protocol\":\"tcp\"," +
		        "\"datatype\":\"json\"" +
		    "}" +
		"}" +
	    "}" +
	"}";

        std::cout << "Registering to signaling server: " << url << "\n";
        if (!http_post_json(url, body)) {
            std::cerr << "Signaling server register failed.\n";
            return -1;
        }
        std::cout << "Register OK\n";
    }

    // Drone pose socket connect (results)
    int map_sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (map_sock < 0) {
        perror("socket(map_sock)");
        return 1;
    }
    {
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(drone_pose_port);
        if (inet_pton(AF_INET, drone_ip.c_str(), &addr.sin_addr) != 1) {
            std::cerr << "Bad DRONE_IP: " << drone_ip << "\n";
            return 1;
        }
        if (::connect(map_sock, (sockaddr*)&addr, sizeof(addr)) != 0) {
            perror("connect(map_sock)");
            return 1;
        }
        std::cout << "Connected to drone result socket: " << drone_ip << ":" << drone_pose_port << "\n";
    }

    // Server socket
    int server_sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0) {
        perror("socket(server_sock)");
        return 1;
    }
    int opt = 1;
    setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(MY_PORT);
    if (inet_pton(AF_INET, my_ip.c_str(), &bind_addr.sin_addr) != 1) {
        std::cerr << "Bad local ip: " << my_ip << "\n";
        return 1;
    }

    if (::bind(server_sock, (sockaddr*)&bind_addr, sizeof(bind_addr)) != 0) {
        perror("bind(server_sock)");
        return 1;
    }
    if (::listen(server_sock, 1) != 0) {
        perror("listen(server_sock)");
        return 1;
    }
    std::cout << "Waiting drone connection on " << my_ip << ":" << MY_PORT << " ...\n";

    sockaddr_in peer{};
    socklen_t peerlen = sizeof(peer);
    int conn_sock = ::accept(server_sock, (sockaddr*)&peer, &peerlen);
    if (conn_sock < 0) {
        perror("accept");
        return 1;
    }
    std::cout << "Drone connected.\n";

    // ORB-SLAM3 init
    // eSensor: MONOCULAR
    ORB_SLAM3::System SLAM(vocab_path, settings_path, ORB_SLAM3::System::MONOCULAR, true);

    FrameQueue fq;
    std::atomic<bool> running{true};

    // Receive thread
    std::thread t_recv([&]{
        while (running.load()) {
            uint32_t hdr[2]; // frame_id, length (network order)
            if (!recv_exact(conn_sock, hdr, 8)) {
                std::cerr << "Drone disconnected (header).\n";
                break;
            }

            uint32_t frame_id = ntohl(hdr[0]);
            uint32_t len = ntohl(hdr[1]);

            uint64_t timestamp_ns = now_ns();


            if (len == 0 || len > 50 * 1024 * 1024) { // 50MB safety
                std::cerr << "Bad jpeg length: " << len << "\n";
                break;
            }

            FramePacket pkt;
            pkt.frame_id = frame_id;
            pkt.jpeg.resize(len);

            if (!recv_exact(conn_sock, pkt.jpeg.data(), len)) {
                std::cerr << "Drone disconnected (payload).\n";
                break;
            }
            pkt.t_rx_done_ns = now_ns();
            fq.push_drop_old(std::move(pkt));
        }

        running.store(false);
        fq.stop();
    });

    // Process thread
    std::thread t_proc([&]{
        while (running.load()) {
            FramePacket pkt;
            if (!fq.pop(pkt)) break;

            // JPEG -> cv::Mat
            cv::Mat buf(1, (int)pkt.jpeg.size(), CV_8UC1, pkt.jpeg.data());
            cv::Mat img = cv::imdecode(buf, cv::IMREAD_COLOR);
            if (img.empty()) {
                std::cerr << "imdecode failed for frame_id=" << pkt.frame_id << "\n";
                continue;
            }

            
            t = now_sec();
            // ORB-SLAM3 tracking
            auto Tcw = SLAM.TrackMonocular(img, t);

            if (SLAM.GetTrackingState() != ORB_SLAM3::Tracking::OK){
                continue;
            }
            

            auto Twc = Tcw.inverse(); // Getting inverse of Tcw

            
            // Sophus::SE3f -> matrix
            Eigen::Matrix4f m = Twc.matrix();

            // Translation
            double tx = m(0,3);
            double ty = m(1,3);
            double tz = m(2,3);

            // Rotation -> quaternion
            Eigen::Matrix3f R = m.block<3,3>(0,0);
            Eigen::Quaternionf q(R);
            q.normalize();

            double qx = q.x();
            double qy = q.y();
            double qz = q.z();
            double qw = q.w();

            std::string json = pose_to_json(pkt.frame_id, tx, ty, tz, qx, qy, qz, qw);

            // packet: [frame_id(uint32)][len(uint32)] + json
            uint32_t fid_n = htonl(pkt.frame_id);
            uint32_t len_n = htonl((uint32_t)json.size());

            std::vector<uint8_t> out;
            out.resize(8 + json.size());
            std::memcpy(out.data() + 0, &fid_n, 4);
            std::memcpy(out.data() + 4, &len_n, 4);
            std::memcpy(out.data() + 8, json.data(), json.size());

            if (!send_all(map_sock, out.data(), out.size())) {
                std::cerr << "Failed to send pose to drone.\n";
                running.store(false);
                fq.stop();
                break;
            }
            uint64_t t_send_done_ns = now_ns();
            uint64_t lat_ns = t_send_done_ns - pkt.t_rx_done_ns;
            double lat_ms = lat_ns / 1e6;


            std::cout << "Pose sent frame=" << pkt.frame_id << " json=" << json << "process time=" << lat_ms << "ms\n";
        }
    });

    // Wait
    t_recv.join();
    t_proc.join();

    // Shutdown
    std::cout << "Shutting down SLAM...\n";
    SLAM.Shutdown();

    ::close(conn_sock);
    ::close(server_sock);
    ::close(map_sock);

    curl_global_cleanup();

    std::cout << "Done.\n";
    return 0;
}
