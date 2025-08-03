#include<iostream>
#include <queue>
#include <chrono>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <thread>
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<cstring>
#include<unistd.h>
#include<cstdlib>
#include<nlohmann/json.hpp>
#include<cstdint>
#include"httplib.h"
#include<unordered_map>
#include"data_type.h"
#include<nlohmann/adl_serializer.hpp>
#include <filesystem>
#include <system_error>
#include <sys/mman.h>
#include"base64_code.h"
#include"data_handle.h"
#include <opencv2/opencv.hpp>
#include<future>
#define UDP_PORT 8111
#define HTTP_PORT 8080
#define destination_ip "127.0.0.1"
#define INF_PORT 8870
#define ROBOT_PORT 8880
#define MAP_PATH "./map/scans2_ps.pgm"


using  namespace httplib;
using json=nlohmann::json;
std::atomic<bool> stopReceiving(false);
std::vector<struct pathpoint> globalPathPoints;
std::vector<struct pathpoint> global_revPathPoints;
//data
struct ProcessDataParams processdata={
    {0.0f,0.0f, 0.0f, 0.0f},
    // 初始化仪表数据
    {0.0f, 0.0f, 0.0f},
    // 初始化温度信息
    {0.0f, 0.0f},
    // 初始化视觉数据
    {0, 0, 0, 0, 0}
};

struct pathpoint revPothPoints;
std::atomic<bool> dataProcessed(false); // 原子变量确保线程安全
// 定义全局变量存储数据
std::atomic<bool> pgmdataprocess(false);


// 映射命令到相应的请求帧
std::unordered_map<Command, std::string> commandFrameMap = {
    {Command::ResumeInspection, "ResumeInspection_Frame"},
    {Command::StopInspection, "StopInspection_Frame"},
    {Command::MoveForward, "MoveForward_Frame"},
    {Command::MoveBackward, "MoveBackward_Frame"},
    {Command::TurnLeft, "TurnLeft_Frame"},
    {Command::TurnRight, "TurnRight_Frame"},
    {Command::StandUp, "StandUp_Frame"},
    {Command::BowDown, "BowDown_Frame"},
    {Command::MoveLeft, "MoveLeft_Frame"},
    {Command::MoveRight, "MoveRight_Frame"}
};



commonFrame request_yanwu1Frame = {
    {0, 0, 4, 0},       // 初始化 frameHead，分别对应 frameType, source, dest 和 subObj
//    {0, 0, 0, 0.0f, 0.0f} // 初始化 framedata，分别对应 hasData, data1, data2, data3 和 data4
};

commonFrame request_yanwu2Frame = {
    {0, 0, 4, 1},       // 初始化 frameHead，分别对应 frameType, source, dest 和 subObj
//    {0, 0, 0, 0.0f, 0.0f} // 初始化 framedata，分别对应 hasData, data1, data2, data3 和 data4
};

commonFrame request_FACEFrame = {
    {0, 0, 1, 0},       // 初始化 frameHead，分别对应 frameType, source, dest 和 subObj
    {0, 0, 0, 0.0f, 0.0f} // 初始化 framedata，分别对应 hasData, data1, data2, data3 和 data4
};

commonFrame request_SMOKE_HATFrame = {
    {0, 0, 1, 2},       // 初始化 frameHead，分别对应 frameType, source, dest 和 subObj
    {0, 0, 0, 0.0f, 0.0f} // 初始化 framedata，分别对应 hasData, data1, data2, data3 和 data4
};

commonFrame request_METERFrame = {
    {0, 0, 3, 1},       // 初始化 frameHead，分别对应 frameType, source, dest 和 subObj
    {0, 0, 0, 0.0f, 0.0f} // 初始化 framedata，分别对应 hasData, data1, data2, data3 和 data4
};

commonFrame request_INFFrame = {
    {0, 0, 1, 3},       // 初始化 frameHead，分别对应 frameType, source, dest 和 subObj
    {0, 0, 0, 0.0f, 0.0f} // 初始化 framedata，分别对应 hasData, data1, data2, data3 和 data4
};

commonFrame request_TEMFrame = {
    {0, 0, 5, 0},       // 初始化 frameHead，分别对应 frameType, source, dest 和 subObj
    {0, 0, 0, 0.0f, 0.0f} // 初始化 framedata，分别对应 hasData, data1, data2, data3 和 data4
};

commonFrame request_FlameFrame = {
    {0, 0, 1, 4},       // 初始化 frameHead，分别对应 frameType, source, dest 和 subObj
    {0, 0, 0, 0.0f, 0.0f} // 初始化 framedata，分别对应 hasData, data1, data2, data3 和 data4
};
commonFrame stop_data_send = {
    {0, 0, 1, 5}, // 初始化 frameHead，frameType, source, dest 和 subObj
    {0, 0, 0, 0.0f, 0.0f} // 初始化 framedata
};
//----------
commonFrame close_infframe = {
        {2, 0, 1, 3}, // 初始化 frameHead，frameType, source, dest 和 subObj
        {0, 2, 0, 0.0f, 0.0f} // 初始化 framedata
};

commonFrame close_TEMframe = {
        {2, 0, 5, 0}, // 初始化 frameHead，frameType, source, dest 和 subObj
        {1, 2, 0, 0.0f, 0.0f} // 初始化 framedata
    };
commonFrame time_frame = {
   
        {2, 0, 2, 0}, // 初始化 frameHead，frameType, source, dest 和 subObj
        {1, 0, 0, 0.0f, 0.0f} // 初始化 framedata
    };

commonFrame request_HARDHATFrame = {
    {2, 0, 1, 2}, // subObj=2
    {0, 0, 0, 0.0f, 0.0f}
};
commonFrame request_SMOKEFrame = {
    {2, 0, 1, 5}, // subObj=5
    {0, 0, 0, 0.0f, 0.0f}
};
// 添加停止数据发送的帧
//commonFrame stop_data_send = {
 //   {0, 0, 1, 5}, // subObj=5
  //  {0, 0, 0, 0.0f, 0.0f}
//};


struct Robot_Status robot_status={};

struct MMapGuard {
    void* addr = nullptr;
    size_t length = 0;
    
    MMapGuard(void* ptr, size_t len) : addr(ptr), length(len) {}
    ~MMapGuard() { 
        if(addr) munmap(addr, length); 
    }
    
    // 禁用拷贝构造和赋值
    MMapGuard(const MMapGuard&) = delete;
    MMapGuard& operator=(const MMapGuard&) = delete;
};

int setup_udp_socket(int port) {
    // 创建 UDP 套接字
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Failed to create UDP socket: " << strerror(errno) << std::endl;
        return -1;
    }

    // 允许在同一端口上多次绑定套接字，即使之前的绑定还未完全释放
    int optval = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) < 0) {
        std::cerr << "Failed to set SO_REUSEADDR: " << strerror(errno) << std::endl;
        close(sockfd);
        return -1;
    }

    //  设置接收缓冲区大小
    int buf_size = 1024 * 1024;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &buf_size, sizeof(buf_size)) < 0) {
        std::cerr << "Failed to set SO_RCVBUF: " << strerror(errno) << std::endl;
    }

    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    //绑定套接字到指定端口
    if (bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Failed to bind UDP socket: " << strerror(errno) << std::endl;
        close(sockfd);
        return -1;
    }

    return sockfd;
}

void printCommonFrame(const commonFrame& frame) {
    std::cout << "=== Received Frame ===" << std::endl;
    std::cout << "Frame Head:" << std::endl;
    std::cout << "  frameType: " << frame.frameHead.frameType << std::endl;
    std::cout << "  source: " << frame.frameHead.source << std::endl;
    std::cout << "  dest: " << frame.frameHead.dest << std::endl;
    std::cout << "  subObj: " << frame.frameHead.subObj << std::endl;
    
    std::cout << "Frame Data:" << std::endl;
    std::cout << "  hasData: " << frame.framedata.hasData << std::endl;
    std::cout << "  data1: " << frame.framedata.data1 << std::endl;
    std::cout << "  data2: " << frame.framedata.data2 << std::endl;
    std::cout << "  data3: " << frame.framedata.data3 << std::endl;
    std::cout << "  data4: " << frame.framedata.data4 << std::endl;
    std::cout << "======================" << std::endl;
}

// 打印全局变量值的函数
void printGlobalVariables() {
    std::cout << "Global Variables:" << std::endl;
    std::cout << "O2 Concentration: " << processdata.gas.globalO2 << std::endl;
    std::cout << "CO Concentration: " << processdata.gas.globalCO << std::endl;
    std::cout << "H2S Concentration: " << processdata.gas.globalH2S << std::endl;
    std::cout << "EX Concentration: " <<processdata.gas. globalEX << std::endl;
    std::cout << "XIAOFANG Reading: " <<processdata.meter.XIAOFANG << std::endl;
    std::cout << "AIR Reading: " << processdata.meter.AIR << std::endl;
    std::cout << "JIXIE Reading: " << processdata.meter.JIXIE << std::endl;
    std::cout << "Face Info: " << processdata.vision.face << std::endl;
    std::cout << "Smoke Info: " << processdata.vision.smoke << std::endl;
    std::cout << "Hat Info: " << processdata.vision.hat << std::endl;
    std::cout << "Average Temperature: " << processdata.tem.average_tem << std::endl; 
    std::cout << "Medium_tem Temperature: " << processdata.tem.medium_tem << std::endl; 
    std::cout << "Intrusion Info: " << processdata.vision.inf << std::endl;
}

// 向机器人发送指令
void sendCommand(int sockfd, const commonFrame& commandFrame, const char* ip, uint16_t port)
{
    // 设置服务器地址
    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);  // 与服务器端口对应
    server_addr.sin_addr.s_addr = inet_addr(ip);  // 指定服务器IP地址

    // 发送控制指令
    int send_len = sendto(sockfd, &commandFrame, sizeof(commandFrame), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));
    if (send_len < 0) {
        std::cerr << "Failed to send control command!" << std::endl;
        // 不关闭套接字，这样可以重用它
    } else {
        std::cout << "Control command sent successfully!" << std::endl;
    }
}

string getCurrentTime() {
    time_t now = time(nullptr);
    tm localTime;
    localtime_r(&now, &localTime);
    ostringstream oss;
    oss << put_time(&localTime, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

// 客户端信息结构
struct ClientInfo {
    std::shared_ptr<httplib::DataSink> sink;
    std::unordered_set<std::string> topics; // 客户端订阅的所有主题
    std::atomic<bool> alive{true};
};

// 主题信息结构
struct TopicInfo {
    mutable std::shared_mutex rw_mutex;
    std::unordered_map<size_t, std::weak_ptr<ClientInfo>> clients; // 主题下的所有客户端
    std::atomic<size_t> active_count{0}; // 活跃客户端数量
};

std::unordered_map<std::string, TopicInfo> sse_topics; // 主题注册表
std::shared_mutex sse_topics_mutex;

std::unordered_map<size_t, std::shared_ptr<ClientInfo>> active_clients; // 活跃客户端注册表
std::shared_mutex active_clients_mutex;
std::atomic<size_t> client_id_counter{0}; // 全局客户端ID生成器

// 辅助函数：获取当前时间字符串

// 构建SSE消息负载
std::string buildSSEPayload(const std::string& event, const nlohmann::json& data) {
    std::ostringstream payload;
    payload << "event: " << event << "\n"
            << "data: " << data.dump() << "\n"
            << "\n";
    return payload.str();
}

void setupSSEConnection(const std::string& topics_str, httplib::Response& res, int udp_socket) {
    // 解析主题列表
    std::vector<std::string> topics;
    std::istringstream iss(topics_str);
    std::string topic;
    while (std::getline(iss, topic, ',')) {
        // 去除前后空格
        size_t start = topic.find_first_not_of(" \t");
        size_t end = topic.find_last_not_of(" \t");
        if (start != std::string::npos && end != std::string::npos) {
            topics.push_back(topic.substr(start, end - start + 1));
        }
    }
    
    if (topics.empty()) {
        res.status = 400;
        res.set_content("至少需要订阅一个主题", "text/plain");
        return;
    }

    // 设置SSE响应头
    res.set_header("Content-Type", "text/event-stream");
    res.set_header("Cache-Control", "no-cache");
    res.set_header("Connection", "keep-alive");
    res.set_header("Access-Control-Allow-Origin", "*");
    // 生成唯一客户端ID
    size_t client_id = client_id_counter.fetch_add(1);
    
    // 创建客户端信息对象
    auto client_info = std::make_shared<ClientInfo>();
    client_info->alive = true;
    
    // 为每个连接创建独立的初始化标志
    auto initialized = std::make_shared<std::atomic<bool>>(false);

    // 初始化回调函数
    auto init_callback = [client_id, topics, &res, client_info, initialized]() {
        // 使用原子标志确保只初始化一次
        if (initialized->exchange(true)) {
            return;
        }
        
        // 1. 将客户端添加到活跃客户端列表
        {
            std::unique_lock<std::shared_mutex> lock(active_clients_mutex);
            // 确保不会重复添加相同ID的客户端
            active_clients.erase(client_id);
            active_clients[client_id] = client_info;
            std::cout << "[" << getCurrentTime() << "] 客户端 " << client_id 
                      << " 已连接 (活跃客户端总数: " << active_clients.size() << ")\n";
        }

        // 2. 将客户端添加到各个订阅主题
        std::unique_lock<std::shared_mutex> global_lock(sse_topics_mutex);
        for (const auto& topic : topics) {
            // 获取或创建主题
            auto& topic_info = sse_topics[topic];
            std::unique_lock<std::shared_mutex> topic_lock(topic_info.rw_mutex);
            
            // 移除可能存在的旧客户端引用
            topic_info.clients.erase(client_id);
            
            // 添加客户端到主题
            topic_info.clients[client_id] = client_info;
            topic_info.active_count.fetch_add(1);
            
            // 记录客户端订阅的主题
            client_info->topics.insert(topic);
            
            std::cout << "[" << getCurrentTime() << "] 客户端 " << client_id 
                      << " 订阅了主题 '" << topic 
                      << "' (当前主题活跃客户端: " << topic_info.active_count.load() << ")\n";
        }
        global_lock.unlock();

        // 3. 发送连接成功消息
        nlohmann::json init_data = {
            {"status", "connected"},
            {"client_id", client_id},
            {"subscribed_topics", topics}
        };
        const std::string init_msg = buildSSEPayload("connected", init_data);
        if (client_info->sink && client_info->sink->is_writable()) {
            client_info->sink->write(init_msg.data(), init_msg.size());
        }
    };

    // 连接关闭时的回调函数
    auto close_callback = [client_id, udp_socket](bool success) {
        // 1. 从活跃客户端列表中移除
        std::shared_ptr<ClientInfo> client_info;
        {
            std::unique_lock<std::shared_mutex> lock(active_clients_mutex);
            auto it = active_clients.find(client_id);
            if (it != active_clients.end()) {
                client_info = it->second;
                active_clients.erase(it);
                std::cout << "[" << getCurrentTime() << "] 客户端 " << client_id 
                          << " 已断开连接 (活跃客户端总数: " << active_clients.size() << ")\n";
            }
        }

        // 2. 从各个订阅主题中移除
        if (client_info) {
            client_info->alive = false;
            
            std::unique_lock<std::shared_mutex> global_lock(sse_topics_mutex);
            for (const auto& topic : client_info->topics) {
                auto topic_it = sse_topics.find(topic);
                if (topic_it != sse_topics.end()) {
                    auto& topic_info = topic_it->second;
                    std::unique_lock<std::shared_mutex> topic_lock(topic_info.rw_mutex);
                    
                    auto client_it = topic_info.clients.find(client_id);
                    if (client_it != topic_info.clients.end()) {
                        topic_info.clients.erase(client_it);
                        topic_info.active_count.fetch_sub(1);
                                                
                        // 如果vision主题没有客户端订阅了，发送停止命令
                        if (topic == "vision" && topic_info.active_count.load() == 0) {
                            sendCommand(udp_socket, stop_data_send, "127.0.0.1", 8870);
                            std::cout << "[" << getCurrentTime() << "] 发送停止检测命令到视觉模块\n";
                        }
                        
                        std::cout << "[" << getCurrentTime() << "] 客户端 " << client_id 
                                  << " 已从主题 '" << topic 
                                  << "' 取消订阅 (当前主题活跃客户端: " << topic_info.active_count.load() << ")\n";
                    }
                }
            }
            global_lock.unlock();
        }
    };

    // 设置响应的分块内容提供器
    res.set_chunked_content_provider(
        "text/event-stream",
        [client_id, init_callback, close_callback, client_info, initialized](size_t offset, httplib::DataSink& sink) {
            // 执行初始化（原子操作确保只执行一次）
            init_callback();
            
            // 保存DataSink指针到客户端信息
            client_info->sink = std::shared_ptr<httplib::DataSink>(&sink, [](auto*){});

            // 15秒心跳机制
            static time_t last_heartbeat = time(nullptr);
            time_t now = time(nullptr);
            if (now - last_heartbeat >= 15 && client_info->alive.load()) {
                const std::string heartbeat = buildSSEPayload("heartbeat", nlohmann::json{});
                if (client_info->sink && client_info->sink->is_writable()) {
                    client_info->sink->write(heartbeat.data(), heartbeat.size());
                }
                last_heartbeat = now;
            }

            return client_info->alive.load();
        },
        [close_callback](bool success) {
            close_callback(success);
        }
    );
}

// 向特定主题推送数据
void pushSSEData(const std::string& topic, const nlohmann::json& data) {
    // 获取主题的客户端快照
    std::vector<std::pair<size_t, std::shared_ptr<ClientInfo>>> clients_snapshot;
    {
        std::shared_lock<std::shared_mutex> global_lock(sse_topics_mutex);
        auto topic_it = sse_topics.find(topic);
        if (topic_it == sse_topics.end()) {
            std::cout << "[" << getCurrentTime() << "] 主题 '" << topic << "' 不存在，没有推送任何数据\n";
            return;
        }

        auto& topic_info = topic_it->second;
        std::shared_lock<std::shared_mutex> topic_lock(topic_info.rw_mutex);
        
        // 从主题的客户端列表获取活跃客户端
        std::shared_lock<std::shared_mutex> clients_lock(active_clients_mutex);
        for (const auto& [client_id, weak_client] : topic_info.clients) {
            if (auto client = weak_client.lock()) {
                clients_snapshot.emplace_back(client_id, client);
            }
        }
        clients_lock.unlock();
        
        std::cout << "[" << getCurrentTime() << "] 主题 '" << topic 
                  << "' 当前客户端: " << topic_info.clients.size() 
                  << ", 活跃客户端: " << clients_snapshot.size() << "\n";
    }

    // 构建消息负载
    const std::string payload = buildSSEPayload("update", data);
    
    // 向客户端发送数据
    std::vector<size_t> failed_clients;
    for (const auto& [client_id, client] : clients_snapshot) {
        if (!client->alive) {
            failed_clients.push_back(client_id);
            continue;
        }
        
        try {
            if (!client->sink || !client->sink->is_writable() || 
                client->sink->write(payload.data(), payload.size()) <= 0) {
                failed_clients.push_back(client_id);
            }
        } catch (...) {
            failed_clients.push_back(client_id);
        }
    }

    // 清理失败的客户端连接
    if (!failed_clients.empty()) {
        std::unique_lock<std::shared_mutex> global_lock(sse_topics_mutex);
        auto topic_it = sse_topics.find(topic);
        if (topic_it != sse_topics.end()) {
            auto& topic_info = topic_it->second;
            std::unique_lock<std::shared_mutex> topic_lock(topic_info.rw_mutex);
            
            for (size_t client_id : failed_clients) {
                auto client_it = topic_info.clients.find(client_id);
                if (client_it != topic_info.clients.end()) {
                    if (client_it->second.expired()) {
                        topic_info.clients.erase(client_it);
                        topic_info.active_count.fetch_sub(1);
                        std::cout << "[" << getCurrentTime() << "] 已从主题 '" << topic 
                                  << "' 移除失败的客户端 " << client_id 
                                  << " (当前主题活跃客户端: " << topic_info.active_count.load() << ")\n";
                    }
                }
            }
        }
        global_lock.unlock();
    }

    std::cout << "[" << getCurrentTime() << "] 已向主题 '" << topic 
              << "' 的 " << (clients_snapshot.size() - failed_clients.size()) 
              << "/" << clients_snapshot.size() << " 个客户端推送数据\n";
}

// 推送烟雾数据到smoke主题
void pushSmokeData() {
    try {
        nlohmann::json data = {
            {"O2_concentration", processdata.gas.globalO2},
            {"CO_concentration", processdata.gas.globalCO},
            {"H2S_concentration", processdata.gas.globalH2S},
            {"ex_concentration", processdata.gas.globalEX}
        };
        
        // 打印当前订阅情况（可选）
        {
            std::shared_lock<std::shared_mutex> read_lock(sse_topics_mutex);
            if (auto it = sse_topics.find("robot_status"); it != sse_topics.end()) {
                std::shared_lock<std::shared_mutex> client_lock(it->second.rw_mutex);
                std::cout << "当前订阅 'robot_status' 主题的客户端数量: " << it->second.clients.size() << std::endl;
            } else {
                std::cout << "没有客户端订阅 'robot_status' 主题" << std::endl;
            }
        }
        
        pushSSEData("smoke", data);
    } catch (const std::exception& e) {
        std::cerr << "pushSmokeData 异常: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "pushSmokeData 未知异常" << std::endl;
    }
}
//
void pushHardhatData() {
    std::cout << "push hardhat data" << std::endl;
    try { 
        std::string hat_status;
        if (processdata.vision.hat == 1) {
            hat_status = "未戴安全帽";  // 1 = 未戴
        } else if (processdata.vision.hat == 0) {
            hat_status = "有安全帽";    // 0 = 戴
        } else {
            hat_status = "检测异常";
        }
        json response_data = {
            //{"[告警-安全帽]"},
           
            {"安全帽检测", hat_status},
            { getCurrentTime()}
        };
        std::cout << "JSON Data: " << response_data.dump(2) << std::endl;
        pushSSEData("vision", response_data);
    } catch (const std::exception& e) {
        std::cerr << "Exception in pushHardhatData: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown exception in pushHardhatData" << std::endl;
    }
}

void pushSmokingData() {
    std::cout << "push smoking data" << std::endl;
    try { 
        json response_data = {
            //{"[告警-吸烟]"},
            {"吸烟检测", (processdata.vision.smoke == 1) ? "有吸烟" : "没有吸烟"},
            { getCurrentTime()}
        };
        std::cout << "JSON Data: " << response_data.dump(2) << std::endl;  // 使用response_data
      
        pushSSEData("vision", response_data); // 传入正确的变量名
    } catch (const std::exception& e) {
        std::cerr << "Exception in pushSmokingData: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown exception in pushSmokingData" << std::endl;
    }
}

void pushRobotStatus() {    
    try {
        std::string Tasktype = robot_status.task_type == 1 ? "定时任务" : "非定时任务";
        std::string Workstatus = robot_status.work_status == 1 ? "正常" : "异常";
        
        json data = {
            {"[运行信息更新]"},
            {"工作状态", Workstatus},
            {"当前巡检位置", {
                {"巡检路径编号", robot_status.now_point.number},
                {"巡检点编号", robot_status.now_point.seq},
                {"坐标x", robot_status.now_point.x},
                {"坐标y", robot_status.now_point.y},
                {"巡检点类型", robot_status.now_point.keyPointType}
            }},
            {"任务类型", Tasktype},
            {"当前时间", robot_status.task_begin_time}
          //  {"任务结束时间", robot_status.task_end_time}
        };
        
        // 打印完整的JSON数据，便于调试
        std::cout << "JSON Data: " << data.dump(2) << std::endl;
        
      
        pushSSEData("robot_status", data);
        
    } catch (const std::exception& e) {
        std::cerr << "Exception in pushRobotStatus: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown exception in pushRobotStatus" << std::endl;
    }
}

void pushMeterData() {
    std::cout<<"push meterdata"<<std::endl;
    try {json data = {
        {"【仪表数据-更新】",getCurrentTime()},
        {"消防仪表", processdata.meter.XIAOFANG},
        {"空气仪表", processdata.meter.AIR},
        {"机械仪表", processdata.meter.JIXIE}
    };
    std::cout << "JSON Data: " << data.dump(2) << std::endl;
   
    pushSSEData("meter", data); // 替换原有实现
        }catch (const std::exception& e) {
        std::cerr << "Exception in pushRobotStatus: " << e.what() << std::endl;
    }    catch (...) {
        std::cerr << "Unknown exception in pushRobotStatus" << std::endl;
    }
}
void pushInfData() {
    std::cout << "push infdata" << std::endl;
    try { 
        // 注意：这里变量名应为response_data（原代码误写为data）
        json response_data = {
            {"[告警-人员入侵]"},
            {"检测时间", getCurrentTime()},
            {"人员入侵状态", (processdata.vision.inf == 0) ? "无人入侵" : "有人入侵"}
        };
        std::cout << "JSON Data: " << response_data.dump(2) << std::endl;  // 使用response_data
       
        pushSSEData("vision", response_data); // 传入正确的变量名
    } catch (const std::exception& e) {
        std::cerr << "Exception in pushInfdata: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown exception in pushInfData" << std::endl;
    }
}

void pushFlameData() {
    std::cout << "push flamedata" << std::endl;
    try { 
        json response_data = {
            //{"[告警-火焰]"},
            {"火焰检测", (processdata.vision.flame == 1) ? "有火焰" : "无火焰"},
            { getCurrentTime()}
        };
        std::cout << "JSON Data: " << response_data.dump(2) << std::endl;  // 使用response_data
      
        pushSSEData("vision", response_data); // 传入正确的变量名
    } catch (const std::exception& e) {
        std::cerr << "Exception in pushFlamedata: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown exception in pushFlameData" << std::endl;
    }
}

void pushHatData() {
    std::cout << "push hatdata" << std::endl;
    try { 
        json response_data = {
            {"[告警-安全帽/吸烟]"},
            {"检测时间", getCurrentTime()},
            {"吸烟检测", (processdata.vision.smoke == 1) ? "有吸烟" : "没有吸烟"},
            {"安全帽检测", (processdata.vision.hat == 1) ? "有安全帽" : "没有安全帽"}
        };
        std::cout << "JSON Data: " << response_data.dump(2) << std::endl;  // 使用response_data
      
        pushSSEData("vision", response_data); // 传入正确的变量名
    } catch (const std::exception& e) {
        std::cerr << "Exception in pushHatdata: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown exception in pushHatData" << std::endl;
    }
}

// 添加互斥锁保护共享资源
std::mutex robotStatusMutex;
int robotflag = 0;
int meterflag=0;
int smokeflag=0;
void trigger(const commonFrame& frame) {
    if (frame.frameHead.frameType == 1) {
        switch (frame.frameHead.source) {
            case 1:
                switch (frame.frameHead.subObj) {
                    case 2: pushHardhatData(); break; // 只推送安全帽
                    case 5: pushSmokingData(); break;   // 只推送吸烟
                    case 3: pushInfData(); break;
                    case 4: pushFlameData(); break;
                    default: break;
                }
                break;
            case 3: //handleSource3(frame,processdata.meter ); 
                    meterflag+=1;
                    std::cout<<meterflag<<std::endl;
                    if(meterflag==3){
                       pushMeterData();
                       meterflag=0;
                    }
                    break;
            case 4: //handleSource4(frame,processdata.gas); 
                    smokeflag+=1;
                    if(smokeflag==2){
                    pushSmokeData();
                    smokeflag=0;
                    }
                    break;
            case 5: //handleSource5(frame,processdata.tem); break; 
            default: break;
        }
    }
    else if (frame.frameHead.frameType == 3) {
        std::lock_guard<std::mutex> lock(robotStatusMutex);
        robotflag += 1;
        if (robotflag == 2) {
            pushRobotStatus();
            robotflag = 0;
        }
    }
}
void receiveData(int sockfd) {
    std::cout << "Listening for incoming data..." << std::endl;

    while (!stopReceiving) {
        commonFrame responseFrame;
        sockaddr_in remote_addr;
        socklen_t remote_addr_len = sizeof(remote_addr);

        int recv_len = recvfrom(sockfd, &responseFrame, sizeof(responseFrame), 0, (struct sockaddr*)&remote_addr, &remote_addr_len);
        if (recv_len < 0) {
            std::cerr << "Failed to receive data: " << strerror(errno) << std::endl;
            // 可以添加重试机制
            continue;
        }

        
        if(responseFrame.frameHead.frameType != 2) {
           // printCommonFrame(responseFrame);
            processData(responseFrame,processdata,revPothPoints,global_revPathPoints,robot_status);     
            //printGlobalVariables();
            dataProcessed = true;
            trigger(responseFrame);
            //return;
        }
       // printCommonFrame(responseFrame);
        processData(responseFrame,processdata,revPothPoints,global_revPathPoints,robot_status);
        const bool isComplete = (global_revPathPoints.size() == responseFrame.frameHead.subObj);
        if(responseFrame.frameHead.subObj==1&&responseFrame.frameHead.dest==0){
            robot_status.now_point=revPothPoints;
        }
        //std::cout<<"1"<<global_revPathPoints.size()<<std::endl;
      //  std::cout<<responseFrame.frameHead.subObj<<std::endl;
        //trigger(responseFrame);
        dataProcessed = isComplete;
    }
        close(sockfd);
}

struct PGMServerState {
    std::mutex data_mutex;
    std::shared_ptr<PGMData> current_data;
    std::atomic<bool> data_ready{false};
    std::condition_variable data_cv;
};

PGMServerState pgm_state;

void pgm_receiver_thread() {
    PGMReceiver receiver;
    if (!receiver.start(1234)) return;

    while (true) {
        PGMDataPtr raw_data;
        if (receiver.get_data(raw_data)) {
            std::lock_guard<std::mutex> lock(pgm_state.data_mutex);
            
            // 直接转移所有权到shared_ptr
            pgm_state.current_data = std::shared_ptr<PGMData>(
                raw_data.release(),  // 释放unique_ptr所有权
                PGMDataDeleter{}     // 保留删除器
            );
            
            pgm_state.data_ready.store(true);
            pgm_state.data_cv.notify_all();
        }
      
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

struct ScheduledTask {
    std::chrono::system_clock::time_point exec_time;
};

struct CompareScheduledTask {
    bool operator()(const ScheduledTask& a, const ScheduledTask& b) {
        return a.exec_time > b.exec_time;
    }
};

std::priority_queue<ScheduledTask, std::vector<ScheduledTask>, CompareScheduledTask> taskQueue;
std::mutex queueMutex;
bool scheduler_running = true;
std::mutex path_mutex;

void scheduler_loop(int socket) {
    while (scheduler_running) {
        auto now = std::chrono::system_clock::now();
        
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            
            while (!taskQueue.empty() && taskQueue.top().exec_time <= now) {
                auto task = taskQueue.top();
                taskQueue.pop();

                // 获取触发时的精确时间
                auto trigger_time = std::chrono::system_clock::now();
                std::time_t trigger_t = std::chrono::system_clock::to_time_t(trigger_time);
                std::tm local_tm = *std::localtime(&trigger_t);
                sendCommand(socket, time_frame, "127.0.0.1", 8111);
                // 格式化输出当前时间
                char time_str[32];
                strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &local_tm);
                std::cout << "[Scheduled Task] Triggered at: " << time_str << std::endl;
               // scheduler_running = false;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


using namespace cv;

// 全局变量
//VideoCapture camera;
std::mutex camera_mutex;
const std::string video_file = "../video/test.mp4";

bool fileExists(const string& path) {
    struct stat buffer;
    return (stat(path.c_str(), &buffer) == 0);
}
std::mutex frame_mutex;
cv::Mat current_frame;
std::shared_ptr<cv::VideoCapture> global_camera;

void start_http_server(int udp_socket){
    Server svr;
    //当客户端访问/smoke_concentration路径时，触发该处理函数
   svr.Get("/sse", [&](const Request& req, Response& res) {
    std::string topics_str;
    
    // 从查询参数中获取topics
    if (req.has_param("topics")) {
        topics_str = req.get_param_value("topics");
    }
    
    // 如果没有指定主题，使用默认主题或返回错误
    if (topics_str.empty()) {
        res.status = 400;
        res.set_content("No topics specified", "text/plain");
        return;
    }
    
    // 设置SSE连接，支持多主题
    setupSSEConnection(topics_str, res, udp_socket);
});
     svr.Get("/trigger", [](const auto& req, auto& res) {
        // 示例数据推送
        pushSSEData("robot_status", json{{"status","working"},{"temp",42.5}});
        res.set_content("Triggered push", "text/plain");
    });   
    //客户端发送指令数据
    svr.Post("/vision", [&](const httplib::Request& req, httplib::Response& res) {
        // 你的处理逻辑
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Access-Control-Allow-Methods", "POST, OPTIONS");
        res.set_header("Access-Control-Allow-Headers", "Content-Type");
        std::cout << "收到POST/vision" << std::endl;
        auto jsonData =json::parse(req.body);
        std::string action = jsonData["action"];
        std::string object = jsonData["object"];
        commonFrame frame={
            {2,0,1,0},
            {0,0,0,0.0f,0.0f}
        };
        //指令映射
        if (object == "face") frame.frameHead.subObj = 0; // FACE
         //else if (object == "smoke_hat") frame.frameHead.subObj = 2; // SMOKE_HAT
         else if (object == "hardhat") frame.frameHead.subObj = 2; // 安全帽检测
         else if (object == "smoke") frame.frameHead.subObj = 5; // 吸烟检测
         else if (object == "inf") frame.frameHead.subObj = 3; // INF
         else if (object == "flame") frame.frameHead.subObj = 4; // INF    
         else {
        res.set_content("Invalid object", "text/plain");
        return;
    }

    // 根据操作填充 data1
        if (action == "open") {
    	if(frame.frameHead.subObj == 3) 
    	{
    		sendCommand(udp_socket, close_TEMframe, "127.0.0.1", 9010);
    		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }
        frame.framedata.data1 = static_cast<uint32_t>(1); // OPEN
        } 
        else if (action == "close") 
        {
        frame.framedata.data1 = static_cast<uint32_t>(2); // CLOSE
        } 
        else {
        res.set_content("Invalid action", "text/plain");
        return;
        }
        std::cout<<"frame.framedata.data1 :"<<frame.framedata.data1<<std::endl;
        std::cout<<"frame.frameHead.subObj :"<<frame.frameHead.subObj<<std::endl;
        sendCommand(udp_socket, frame, "127.0.0.1", 8870);
        res.set_content("Command executed: " + action + " " , "text/plain");
        }
    );
    svr.Options("/vision", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Access-Control-Allow-Methods", "POST, OPTIONS");
        res.set_header("Access-Control-Allow-Headers", "Content-Type");
        res.status = 204;
    });
    svr.Get("/vision/inf", [&](const Request& req, Response& res) {
        sendCommand(udp_socket,request_INFFrame,"127.0.0.1", INF_PORT);
        
        while (!dataProcessed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        printf("iray = %d\n", processdata.vision.inf);
        json response_data = {
            {"人员入侵状态", (processdata.vision.inf == 0) ? "无人入侵" : "有人入侵"}
        };

        // 返回JSON格式的响应
        res.set_content(response_data.dump(), "application/json");
        dataProcessed = false;
    }); 
    svr.Get("/robot/temp", [&](const Request& req, Response& res) {
        sendCommand(udp_socket,request_TEMFrame,"127.0.0.1", 9010);
        
        while (!dataProcessed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
    // 创建 JSON 响应数据
    json response_data = {
        {"average_temperature", processdata.tem.average_tem},
        {"medium_temperature", processdata.tem.medium_tem}
    };

        // 返回JSON格式的响应
        res.set_content(response_data.dump(), "application/json");
        dataProcessed = false;
    });

    svr.Get("/vision/flame", [&](const Request& req, Response& res) {
        sendCommand(udp_socket,request_FlameFrame,"127.0.0.1", 8870);
        
        while (!dataProcessed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
    // 创建 JSON 响应数据
    json response_data = {
        {"flame_status", (processdata.vision.flame == 1) ? "有火焰" : "无火焰"}  // 根据 flame 的值返回状态
    };

        // 返回JSON格式的响应
        res.set_content(response_data.dump(), "application/json");
        dataProcessed = false;
    });

    svr.Get("/vision/face", [&](const Request& req, Response& res) {
        sendCommand(udp_socket, request_FACEFrame, "127.0.0.1", 8870);
        
        while (!dataProcessed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        json response_data = {
            {"FACE_concentration", (processdata.vision.face == 9999) ? "unknown" : std::to_string(processdata.vision.face)}
        };
        
        res.set_content(response_data.dump(), "application/json");
        dataProcessed = false;
    });

    svr.Get("/vision/smoke_hat", [&](const Request& req, Response& res) {
        sendCommand(udp_socket, request_SMOKE_HATFrame, "127.0.0.1", 8870);
        
        while (!dataProcessed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        json response_data = {
            {"smoke_status", (processdata.vision.smoke == 1) ? "有吸烟" : "没有吸烟"},
            {"hat_status", (processdata.vision.hat == 1) ? "有安全帽" : "没有安全帽"}
        };
        
        res.set_content(response_data.dump(), "application/json");
        dataProcessed = false;
    });

    svr.Get("/vision/meter", [&](const Request& req, Response& res) {
        sendCommand(udp_socket, request_METERFrame, "127.0.0.1", 8889);
        
        while (!dataProcessed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
	
        json response_data = {
            {"消防仪表读数", processdata.meter.XIAOFANG},
            {"空气仪表读数", processdata.meter.AIR},
            {"机械仪表读数", processdata.meter.JIXIE}
        };
        
        res.set_content(response_data.dump(), "application/json");
        dataProcessed = false;
    });

    svr.Get("/vision/hardhat", [&](const Request& req, Response& res) {
        sendCommand(udp_socket, request_HARDHATFrame, "127.0.0.1", 8870);
        
        while (!dataProcessed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        json response_data = {
            {"hat_status", (processdata.vision.hat == 1) ? "有安全帽" : "没有安全帽"}
        };
        
        res.set_content(response_data.dump(), "application/json");
        dataProcessed = false;
    });

    svr.Get("/vision/smoke", [&](const Request& req, Response& res) {
        sendCommand(udp_socket, request_SMOKEFrame, "127.0.0.1", 8870);
        
        while (!dataProcessed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        json response_data = {
            {"smoke_status", (processdata.vision.smoke == 1) ? "有吸烟" : "没有吸烟"}
        };
        
        res.set_content(response_data.dump(), "application/json");
        dataProcessed = false;
    });
    
    svr.Get("/get_all_pathpoint", [&](const Request& req, Response& res) {
       while (!dataProcessed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        try {
            // 线程安全访问全局路径点
            std::vector<pathpoint> current_path;
            {
                std::lock_guard<std::mutex> lock(path_mutex);
                current_path = global_revPathPoints; // 复制数据避免长期锁定
            }
    
            // 序列化为JSON数组
            nlohmann::json j_array = nlohmann::json::array();
            for (const auto& point : current_path) {
                j_array.push_back({
                    {"path", point.number},
                    {"point_seq",point.seq},
                    {"x", point.x},
                    {"y", point.y},
                    {"keyPointType", point.keyPointType}
                });
            }
    
            // 设置分块传输编码
            res.set_chunked_content_provider("application/json", 
                [j_array = std::move(j_array)](size_t offset, DataSink& sink) {
                    const std::string json_str = j_array.dump();
                    
                    if(offset >= json_str.size()) {
                        sink.done();
                        return false;
                    }
    
                    size_t chunk_size = std::min(4096UL, json_str.size() - offset);
                    sink.write(json_str.data() + offset, chunk_size);
                    return true;
                },
                [](bool success) { std::cout<<"done"<<std::endl;}
            );
    
        } catch (const std::exception& e) {
            res.status = 500;
            res.set_content(json{{"error", e.what()}}.dump(), "application/json");
        }
    });
    svr.Get("/map", [](const Request& req, Response& res) {
        // 带超时的等待
        std::shared_ptr<PGMData> active_pgm;
        {
            std::unique_lock<std::mutex> lock(pgm_state.data_mutex);
            if (!pgm_state.data_cv.wait_for(lock, std::chrono::seconds(1),
                []{ return pgm_state.data_ready.load(); })) 
            {
                res.status = 504;
                res.set_content("Data Not Ready", "text/plain");
                return;
            }
            active_pgm = pgm_state.current_data;
        }
        
        // 数据有效性检查!cv::imencode(".jpg", frame, buffer)
        if (!active_pgm || !active_pgm->pixel_data) {
            res.status = 500;
            res.set_content("Invalid Data", "text/plain");
            return;
        }
    
        std::ostringstream header_stream;

	// 标准头
	header_stream << "P5\n"                    // 强制换行符统一为LF
             << active_pgm->header.width << " " 
             << active_pgm->header.height << "\n"
             << active_pgm->header.max_gray << "\n"; 

	// 附加元数据注释（确保在标准头之后）
	header_stream << "# MAP_NUM:" << active_pgm->header.map_number << "\n"
             << "# RESOLUTION:" << active_pgm->header.resolution << "\n";

	std::string header_str = header_stream.str();
        // 构造分块传输上下文
        struct ChunkContext {
            std::string header;
            std::shared_ptr<PGMData> pgm_data;
            size_t total_size;
            size_t header_size;
        };
        
        auto ctx = std::make_shared<ChunkContext>(ChunkContext{
            header_str,
            active_pgm,
            header_str.size() + active_pgm->header.data_size,
            header_str.size()
        });
    
        // 正确设置分块传输
        res.set_chunked_content_provider(
            "image/x-portable-graymap",  // 内容类型
            [ctx](size_t offset, DataSink& sink) -> bool {  // 内容提供器
                try {
                    constexpr size_t CHUNK_SIZE = 131072;
    
                    if (offset >= ctx->total_size) {
                        sink.done();
                        return false;
                    }
    
                    // 判断当前数据位置
                    if (offset < ctx->header_size) {
                        // 发送头部数据
                        size_t remain_header = ctx->header_size - offset;
                        size_t send_size = std::min(CHUNK_SIZE, remain_header);
                        sink.write(ctx->header.data() + offset, send_size);
                    } else {
                        // 发送像素数据
                        size_t pixel_offset = offset - ctx->header_size;
                        size_t remain_pixels = ctx->pgm_data->header.data_size - pixel_offset;
                        size_t send_size = std::min(CHUNK_SIZE, remain_pixels);
                        
                        sink.write(
                            reinterpret_cast<const char*>(ctx->pgm_data->pixel_data + pixel_offset),
                            send_size
                        );
                    }
                    return true;
                } catch (...) {
                    sink.done();
                    return false;
                }
            },
            [](bool success) {}// 必需的资源释放器
        );
    });
   
    svr.Post("/set_pathpoint", [&](const Request& req, Response& res) {
        try {
            auto jsonData = json::parse(req.body);
            
            // 参数校验
            if (!jsonData.contains("points") || !jsonData["points"].is_array()) {
                throw std::runtime_error("Invalid request format: 'points' array required");
            }
    
            std::vector<pathpoint> receivedPoints;
            for (const auto& pointData : jsonData["points"]) {
                pathpoint point;
                
                // 提取必需字段
                point.number = pointData.value("number", 0);
                point.seq=pointData.value("point_seq",0);
                point.x = pointData.value("x", -1.0);
                point.y = pointData.value("y", -1.0);
                point.keyPointType = pointData.value("type", 0);
    
                // 坐标有效性验证
               /* if (point.x < 0 || point.x >= PGM_WIDTH || 
                    point.y < 0 || point.y >= PGM_HEIGHT) {
                    throw std::runtime_error("Coordinate out of bounds: (" + 
                        std::to_string(point.x) + "," + std::to_string(point.y) + ")");
                }
                */
                // 类型校验
                if (point.keyPointType > 2) { // 假设允许0-2类型
                    throw std::runtime_error("Invalid keyPointType: " + 
                        std::to_string(point.keyPointType));
                }
                
                std::cout<<"number:"<<point.number<<std::endl;
                std::cout<<"x:"<<point.x<<std::endl;
                std::cout<<"y:"<<point.y<<std::endl;
                std::cout<<"type:"<<point.keyPointType<<std::endl;
                receivedPoints.push_back(point);
            }
    
            // 原子化更新全局路径点
            auto& points = globalPathPoints;
            points.clear();
            points.insert(points.end(), receivedPoints.begin(), receivedPoints.end());
            //std::cout<<"pathpoint sent"<<std::endl;
            commonFrame frame={
                {2,0,1,0},
                {0,0,0,0.0f,0.0f}
            };
            frame.frameHead.subObj=points.size();  
            for (auto it=points.begin();it!=points.end();++it){
              
                frame.frameHead.source=it->number;
                frame.framedata.data1=it->seq;
                frame.framedata.data2=it->keyPointType;
                frame.framedata.data3=it->x;
                frame.framedata.data4=it->y;
                sendCommand(udp_socket, frame, "127.0.0.1", 8870);
            }
            //std::cout<<points<<std::endl;
            res.set_content(json{{"status", "success"}, {"received_points", points.size()}}.dump(), 
                           "application/json");
    
        } catch (const std::exception& e) {
            res.status = 400;
            res.set_content(json{{"error", e.what()}}.dump(), "application/json");
        }
    });
    svr.Post("/robot", [&](const Request& req, Response& res) {
        auto jsonData = json::parse(req.body);
        std::string commandStr = jsonData["robot"];
    
        commonFrame frame = {
            {2, 0, 2, 0}, // 初始化 frameHead，frameType, source, dest 和 subObj
            {1, 0, 0, 0.0f, 0.0f} // 初始化 framedata
        };
    
        // 根据不同的指令填充 data1
        if (commandStr == "ResumeInspection") frame.framedata.data1 = static_cast<uint32_t>(Command::ResumeInspection);
        else if (commandStr == "StopInspection") frame.framedata.data1 = static_cast<uint32_t>(Command::StopInspection);
        else if (commandStr == "MoveForward") frame.framedata.data1 = static_cast<uint32_t>(Command::MoveForward);
        else if (commandStr == "MoveBackward") frame.framedata.data1 = static_cast<uint32_t>(Command::MoveBackward);
        else if (commandStr == "TurnLeft") frame.framedata.data1 = static_cast<uint32_t>(Command::TurnLeft);
        else if (commandStr == "TurnRight") frame.framedata.data1 = static_cast<uint32_t>(Command::TurnRight);
        else if (commandStr == "StandUp") frame.framedata.data1 = static_cast<uint32_t>(Command::StandUp);
        else if (commandStr == "BowDown") frame.framedata.data1 = static_cast<uint32_t>(Command::BowDown);
        else if (commandStr == "MoveLeft") frame.framedata.data1 = static_cast<uint32_t>(Command::MoveLeft);
        else if (commandStr == "MoveRight") frame.framedata.data1 = static_cast<uint32_t>(Command::MoveRight);
        else {
            res.set_content("Invalid command", "text/plain");
            return;
        }
    
        // 发送控制指令
        sendCommand(udp_socket, frame, "127.0.0.1", 8081);
        res.set_content("Command executed: " + commandStr, "text/plain");
    });
    
    svr.Post("/scheduled_task", [&](const Request& req, Response& res) {
        try {
            auto jsonData = nlohmann::json::parse(req.body);
            std::string commandStr = jsonData["scheduled_task"];
            int path =jsonData["path"]; 
            std::string datetimeStr = jsonData["task_begin_time"];
            std::string datatimeStr1 =jsonData["task_end_time"];
            
            // 解析时间
            struct tm tm = {};
            std::istringstream ss(datetimeStr);
            ss >> std::get_time(&tm, "%Y-%m-%d %H:%M");
            tm.tm_isdst = -1;
            
            time_t tt = std::mktime(&tm);
            if (tt == -1) throw std::runtime_error("Invalid datetime");
            auto execTime = std::chrono::system_clock::from_time_t(tt);
            //
            if (commandStr == "ResumeInspection") {
                time_frame.framedata.data1 = static_cast<uint32_t>(Command::ResumeInspection);
            } else if (commandStr == "StopInspection") {
                time_frame.framedata.data1 = static_cast<uint32_t>(Command::StopInspection);
            } else {
                res.set_content("Invalid command", "text/plain");
                return;
            }
            {
                std::lock_guard<std::mutex> lock(queueMutex);
                taskQueue.push({execTime});
            }
            
             
            res.set_content("Scheduled successfully", "text/plain");
        } catch (const std::exception& e) {
            res.status = 400;
            res.set_content(e.what(), "text/plain");
        }
    });
   
        svr.Get("/video_store", [&](const httplib::Request &, httplib::Response &res) {
            auto video_reader = std::make_shared<cv::VideoCapture>(video_file, cv::CAP_FFMPEG);
    
            // 检查视频文件是否成功打开
            if (!video_reader->isOpened()) {
                res.status = 500;
                res.set_content("Failed to open video file", "text/plain");
                return;
            }
    
            video_reader->set(cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY);
            video_reader->set(cv::CAP_PROP_BUFFERSIZE, 3);
    
            const double fps = video_reader->get(cv::CAP_PROP_FPS);
            const cv::Size target_size(640, 480);
            int frame_counter = 0;
            int total_frames = static_cast<int>(video_reader->get(cv::CAP_PROP_FRAME_COUNT));
    
            res.set_chunked_content_provider(
                "multipart/x-mixed-replace; boundary=frame",
                [=](size_t offset, httplib::DataSink &sink) mutable {
                    cv::Mat frame;
                    if (!video_reader->read(frame)) {
                        video_reader->set(cv::CAP_PROP_POS_FRAMES, 0);
                        if (!video_reader->read(frame)) {
                            // 释放资源
                            video_reader->release();
                            return false;
                        }
                    }
    
                    cv::resize(frame, frame, target_size);
    
                    std::vector<uchar> buffer;
                    if (!cv::imencode(".jpg", frame, buffer, {cv::IMWRITE_JPEG_QUALITY, 80})) {
                        // 释放资源
                        video_reader->release();
                        return false;
                    }
    
                    // 构建HTTP分块响应
                    const std::string chunk_header = 
                        "--frame\r\n"
                        "Content-Type: image/jpeg\r\n"
                        "Content-Length: " + std::to_string(buffer.size()) + 
                        "\r\nX-Frame-Info: " + std::to_string(++frame_counter) +
                        "/" + std::to_string(total_frames) + 
                        "\r\n\r\n";
    
                    sink.write(chunk_header.data(), chunk_header.size());
                    sink.write(reinterpret_cast<char*>(buffer.data()), buffer.size());
                    sink.write("\r\n", 2);
                    const int delay_ms = static_cast<int>(1000/(fps ));

// 添加时间戳补偿机制
                  static auto last_frame_time = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::steady_clock::now() - last_frame_time;
                int compensation = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
                std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms - compensation));
                last_frame_time = std::chrono::steady_clock::now();
                    return true;
                }
            );
        });
    
    
    svr.Get("/video_feed", [&](const httplib::Request &, httplib::Response &res) {
        // 初始化相机（复用全局相机对象）
        if (!global_camera) {
            global_camera = std::make_shared<cv::VideoCapture>(0);
            if (!global_camera->isOpened()) {
                res.status = 500;
                res.set_content("Error opening camera", "text/plain");
                return;
            }
            global_camera->set(cv::CAP_PROP_FRAME_WIDTH, 480);
            global_camera->set(cv::CAP_PROP_FRAME_HEIGHT, 320);
        }
    
        res.set_chunked_content_provider(
            "multipart/x-mixed-replace; boundary=frame",
            [](size_t /*offset*/, httplib::DataSink &sink) {
                cv::Mat frame;
                *global_camera >> frame;
    
                // 更新共享帧
                {
                    std::lock_guard<std::mutex> lock(frame_mutex);
                    current_frame = frame.clone(); // 深拷贝保证线程安全
                }
                // 视频结束处理（循环播放）
                if (frame.empty()) {
                    // 方式1：重置播放位置
                    global_camera->set(cv::CAP_PROP_POS_FRAMES, 0);
                    *global_camera >> frame;  // 重新读取第一帧
                    
                    // 方式2：重新打开文件（更可靠）
                    // camera->release();
                    // if (!camera->open(video_file)) {
                    //     sink.done();
                    //     return false;
                    // }
                    // *camera >> frame;
                    
                    if (frame.empty()) {
                        sink.done();
                        return false;
                    }
                }
    
                // 编码为 JPEG
                std::vector<uchar> buffer;
                std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70}; // 质量值 0-100

                if (cv::imencode(".jpg", frame, buffer, params)) {
                    sink.done();
                    return false;
                }
    
                // 构建响应块
                const std::string part = 
                    "--frame\r\n"
                    "Content-Type: image/jpeg\r\n"
                    "Content-Length: " + std::to_string(buffer.size()) + "\r\n\r\n";
    
                // 发送数据块
                sink.write(part.data(), part.size());
                sink.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
                sink.write("\r\n", 2);
    
                // 控制帧率（按视频实际帧率）
                const double fps = global_camera->get(cv::CAP_PROP_FPS);
                if (fps > 0) {
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(static_cast<int>(1000 / fps))
                    );
                }
    
              
    
                return true;
            }
        );
    });
    svr.Get("/capture", [](const httplib::Request &, httplib::Response &res) {
        cv::Mat snapshot;
        if (!global_camera) {
            global_camera = std::make_shared<cv::VideoCapture>(0);
            if (!global_camera->isOpened()) {
                res.status = 500;
                res.set_content("Error opening camera", "text/plain");
                return;
            }
            global_camera->set(cv::CAP_PROP_FRAME_WIDTH, 640);
            global_camera->set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        }
        // 加锁获取当前帧
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (current_frame.empty()) {
                res.status = 503;
                res.set_content("No frame available", "text/plain");
                return;
            }
            snapshot = current_frame.clone();
        }
    
        // 编码为JPEG
        std::vector<uchar> buffer;
        if (!cv::imencode(".jpg", snapshot, buffer)) {
            res.status = 500;
            res.set_content("Encode failed", "text/plain");
            return;
        }
        res.set_header("X-Capture-Timestamp", 
            std::to_string(std::chrono::system_clock::now().time_since_epoch().count()));
        // 返回单张图片
        res.set_content(
            reinterpret_cast<const char*>(buffer.data()),
            buffer.size(),
            "image/jpeg"
        );
    });
svr.Get("/video_test", [&](const httplib::Request &, httplib::Response &res) {
    // 打开摄像头（明确指定 V4L2 后端）
    auto camera = make_shared<VideoCapture>(0);
    if (!camera->isOpened()) {
        res.status = 500;
        res.set_content("无法打开摄像头", "text/plain");
        return;
    }
    int width = camera->get(CAP_PROP_FRAME_WIDTH);
    int height = camera->get(CAP_PROP_FRAME_HEIGHT);
    //double actualFps = camera->get(CAP_PROP_FPS);

    // 设置分辨率和帧率
    /*camera->set(CAP_PROP_FRAME_WIDTH, 640);
    camera->set(CAP_PROP_FRAME_HEIGHT, 480);
    camera->set(CAP_PROP_FPS, 30); // 明确设置帧率
    */
    // 确认实际使用的格式和帧率
    //int fourcc = static_cast<int>(camera->get(CAP_PROP_FOURCC));
    int fourcc=30;
    double actualFps = camera->get(CAP_PROP_FPS);
    cout << "实际视频格式: " << char(fourcc & 0xFF) 
         << char((fourcc >> 8) & 0xFF)
         << char((fourcc >> 16) & 0xFF)
         << char((fourcc >> 24) & 0xFF) << endl;
    cout << "实际帧率: " << actualFps << " FPS" << endl;

    // 创建视频写入器（使用 MP4 容器和 H.264 编码）
    VideoWriter writer;
    writer.open("./output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), actualFps, Size(640, 480));
    if (!writer.isOpened()) {
        // 尝试使用其他编码器
        writer.open("./output.avi", VideoWriter::fourcc('H', '2', '6', '4'), actualFps, Size(640, 480));
        if (!writer.isOpened()) {
            res.status = 500;
            res.set_content("无法创建视频文件", "text/plain");
            return; 
        }
    }

    // 录制时长（秒）
    const int recordSeconds = 10;
    const int frameCount = static_cast<int>(recordSeconds * actualFps);

    // 摄像头预热
    Mat frame, converted;
    for (int i = 0; i < 5; i++) { // 读取几帧让摄像头稳定
        if (!camera->read(frame)) {
            cerr << "摄像头预热失败" << endl;
        }
    }

    // 录制视频
    int framesWritten = 0;
    for (int i = 0; i < frameCount; i++) {
        if (!camera->read(frame)) {
            cerr << "读取帧失败，已写入 " << framesWritten << " 帧" << endl;
            if (framesWritten == 0) { // 如果一帧都没写入
                res.status = 500;
                res.set_content("无法读取摄像头帧", "text/plain");
                writer.release();
                return;
            }
            break; // 继续处理已捕获的帧
        }

        // 确保每次都有有效的 converted 帧
        if (fourcc == VideoWriter::fourcc('Y', 'U', 'Y', 'V')) {
            cvtColor(frame, converted, COLOR_YUV2BGR_YUYV);
        } else {
            converted = frame.clone(); // 确保 converted 始终有效
        }

        writer.write(converted);
        framesWritten++;
    }

    // 释放资源
    writer.release();
    camera->release();

    // 返回成功响应
    res.status = 200;
    res.set_content("视频已保存为: output.mp4", "text/plain");
});
    
   
        svr.listen("0.0.0.0", HTTP_PORT);
    }
    
   

    
int  main(){

    int sockfd=setup_udp_socket(UDP_PORT);//绑定UDP端口
    if (sockfd < 0) {
        std::cerr << "Failed to set up UDP socket!" << std::endl;
        return -1;
    }
    // 启动数据接收线程
    std::thread receiverThread(receiveData, sockfd);
    std::thread httpThread([&](){
        start_http_server(sockfd);
    });
 
    std::thread receivepgmThread(pgm_receiver_thread);
    std::thread scheduler(scheduler_loop,sockfd);

    receiverThread.join();
    httpThread.join();
    receivepgmThread.join();
    scheduler.join();
   // heartbeatThread.join();
   return 0;
   
}  
        
    
    
    
    
    


