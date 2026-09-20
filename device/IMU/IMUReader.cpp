#include "IMUReader.h"
#include <cstdio>
#include <unistd.h> // for usleep
#include <iostream>
#include <thread>
#include <iostream>
#include <chrono>
#include <iomanip>  // for std::put_time
#include <ctime>    // for std::tm
#include "yaml-cpp/yaml.h"


#define CMD_REPLAY_TIMEOUT_MS (100)
#define DISPLAY_UPDATE_INTERVAL 0.05
#define MAX_ATTEMPTS 2

bool IMUReader::start() {

    std::string file_path = "../device/config/device.yaml";
    YAML::Node config = YAML::LoadFile(file_path);//这里调用了yaml-cpp库的LoadFile函数来加载指定路径的YAML文件，并将其解析为一个YAML::Node对象。这个对象可以用来访问YAML文件中的数据结构。
    std::string port_str = config["port_name"].as<std::string>();
    port_name = port_str.c_str();
    baud_rate = config["baud_rate"].as<int>();

    if ((fd_ = serial_port_open(port_name)) < 0 || serial_port_configure(fd_, baud_rate) < 0) {
        log_error("Failed to open or configure port %s with %d",port_name, baud_rate);
        return false;
    }

    log_info("Reading from port %s at %d baud. Press CTRL+C to exit.", port_name, baud_rate);

    bool ACK_status = sendConfigCommand("$DSIMC,CALIB,FRAME,RFU");//发送配置指令
    // if (!sendConfigCommand("$DSIMC,CALIB,FRAME,RFU")) { // IM1R默认FRD 前右下(FRD) 前左上(FLU) 右前上(RFU)
    //     log_error("Failed to configure IMU to frame. Continuing anyway...");
        
    //     return false;
    // }//ghd+切换坐标系


    clock_gettime(CLOCK_MONOTONIC, &last_time_);
    last_display_time_ = last_time_;

    std::thread main_loop_thread(&IMUReader::mainLoop, this);
    main_loop_thread.detach();  // 让线程在后台运行，不阻塞主线程

    return true;
}

void IMUReader::mainLoop() {
    std::vector<unsigned char> binary_buffer;
    while (true) {
        bool new_data = false;//数据更新标志位

        // struct timespec start, end;
        // double elapsed_ms;        
        // clock_gettime(CLOCK_MONOTONIC, &start);//获取开始时间

        int len = serial_port_read(fd_, (char*)recv_buf, sizeof(recv_buf));//读取串口数据

        // clock_gettime(CLOCK_MONOTONIC, &end);   // 获取结束时间
        // elapsed_ms = (end.tv_sec - start.tv_sec) * 1000.0 + (end.tv_nsec - start.tv_nsec) / 1e6;// 计算读取串口数据经过的时间（毫秒）
        // printf("Elapsed time: %.3f ms\n", elapsed_ms);
        // printf("Elapsed time: %.3d ms\n", len);


        if (len > 0) {//解析数据
            for (int i = 0; i < len; i++) {
                unsigned char byte = recv_buf[i];
                // --- 二进制协议解析逻辑开始 ---
                if (binary_buffer.size() == 0 && byte == 0xA5) {
                    // 1. 找到帧头第一个字节 0xA5
                    binary_buffer.push_back(byte);
                } else if (!binary_buffer.empty()) {
                    // 2. 已经找到帧头，继续接收数据
                    binary_buffer.push_back(byte);                
                }
                // 检查是否收满一帧 (总长度72字节)
                    if (binary_buffer.size() == 72) {
                        // 3. 检查帧尾是否正确 (0x0D, 0x0A)
                        if (binary_buffer[70] == 0x0D && binary_buffer[71] == 0x0A) {
                            // 4. 解析这一帧
                            if (parseIMUBinaryPacket(binary_buffer)) {
                                new_data = true;
                                frame_count_++;
                            }
                            binary_buffer.clear();
                        }else{
                            binary_buffer.clear();
                        }                        
                    }

        }

        // struct timespec current_time;//需要观测帧率时释放
        // clock_gettime(CLOCK_MONOTONIC, &current_time);
        // elapsed_time_ = (current_time.tv_sec - last_time_.tv_sec) +
        //                 (current_time.tv_nsec - last_time_.tv_nsec) / 1e9;
        // double display_elapsed_time = (current_time.tv_sec - last_display_time_.tv_sec) +
        //                               (current_time.tv_nsec - last_display_time_.tv_nsec) / 1e9;



            if (new_data) {
                acc(0) =  imu_raw_.IM1R.acc[1];// 单位已是 m/s²
                acc(1) = -imu_raw_.IM1R.acc[0];
                acc(2) =  imu_raw_.IM1R.acc[2];

                ang_vel(0) =  imu_raw_.IM1R.gyr[1];// 单位已是 °/s
                ang_vel(1) = -imu_raw_.IM1R.gyr[0];
                ang_vel(2) =  imu_raw_.IM1R.gyr[2];

                rpy(0) = -imu_raw_.IM1R.roll;// 横滚角x
                rpy(1) =  imu_raw_.IM1R.pitch;// 俯仰角y                
                rpy(2) =  imu_raw_.IM1R.yaw;// 偏航角z

                quat(0) = imu_raw_.IM1R.quat[0];
                quat(1) = imu_raw_.IM1R.quat[1];
                quat(2) = imu_raw_.IM1R.quat[2];
                quat(3) = imu_raw_.IM1R.quat[3];


                rpyToQuaternion(-imu_raw_.IM1R.roll, imu_raw_.IM1R.pitch, 0, quat_no_yaw(0), quat_no_yaw(1), quat_no_yaw(2), quat_no_yaw(3));
                rotateQuatAroundY180(quat_no_yaw(0), quat_no_yaw(1), quat_no_yaw(2), quat_no_yaw(3));
                quatToZyx(quat_no_yaw(0), quat_no_yaw(1), quat_no_yaw(2), quat_no_yaw(3), zyx(2), zyx(1), zyx(0));

                zyx(1) = -zyx(1);

                gravity_vec = computeGravityVec(-imu_raw_.IM1R.roll, imu_raw_.IM1R.pitch);

                acc_new(0) =  imu_raw_.IM1R.acc[0];//ghd 
                acc_new(1) = -imu_raw_.IM1R.acc[1];
                acc_new(2) = -imu_raw_.IM1R.acc[2];

                ang_vel_new(0) =  imu_raw_.IM1R.gyr[0];
                ang_vel_new(1) = -imu_raw_.IM1R.gyr[1];
                ang_vel_new(2) = -imu_raw_.IM1R.gyr[2];

                // last_display_time_ = current_time;//需要观测帧率时释放
            }
            

        // if (elapsed_time_ >= 1.0) {//需要观测帧率时释放
        //     frame_rate_ = (int)(frame_count_ / elapsed_time_);
        //     frame_count_ = 0;
        //     last_time_ = current_time;
        // }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));//500hz，1/500s = 2ms发一帧，1ms满足需求。

        }
    }
}

int IMUReader::safe_sleep(unsigned long usec) {
    struct timespec ts;
    ts.tv_sec = usec / 1000000;
    ts.tv_nsec = (usec % 1000000) * 1000;
    
    while (nanosleep(&ts, &ts) == -1) {
        if (errno != EINTR) {
            return -1;
        }
    }
    return 0;
}


bool IMUReader::parseIMUBinaryPacket(const std::vector<unsigned char>& packet) {//ghd+ 将读取到的二进制数据进行解析，提取出IMU的加速度、角速度、姿态角等信息，并存储到imu_raw_结构体中。

    if (packet.size() != 72 || packet[0] != 0xA5 || packet[1] != 0x5A) {
        return false;
    }

    // 辅助函数：将4个字节转换为 float (小端序)
    auto bytesToFloat = [](const unsigned char* p) -> float {
        float f;
        memcpy(&f, p, 4);
        return f;
    };
    
    // 辅助函数：将2个字节转换为 short (小端序)
    auto bytesToShort = [](const unsigned char* p) -> short {
        short s;
        memcpy(&s, p, 2);
        return s;
    };

    try {
        // 数据段从 packet 的第 5 个字节 (索引为 4) 开始
        const unsigned char* data = packet.data() + 5;

        // D0: 帧计数 (1字节)
        // D1-D8: 时间戳 (8字节) - 暂时忽略
        
        // D9-D12: 加速度 X轴 (4字节 Float)
        imu_raw_.IM1R.acc[0] = bytesToFloat(data + 9);
        // D13-D16: 加速度 Y轴 (4字节 Float)
        imu_raw_.IM1R.acc[1] = bytesToFloat(data + 13);
        // D17-D20: 加速度 Z轴 (4字节 Float)
        imu_raw_.IM1R.acc[2] = bytesToFloat(data + 17);

        // D21-D24: 角速度 X轴 (4字节 Float)
        imu_raw_.IM1R.gyr[0] = bytesToFloat(data + 21);
        // D25-D28: 角速度 Y轴 (4字节 Float)
        imu_raw_.IM1R.gyr[1] = bytesToFloat(data + 25);
        // D29-D32: 角速度 Z轴 (4字节 Float)
        imu_raw_.IM1R.gyr[2] = bytesToFloat(data + 29);

        // D33-D36: 俯仰角 Pitch (4字节 Float)
        imu_raw_.IM1R.pitch = bytesToFloat(data + 33);
        // D37-D40: 横滚角 Roll (4字节 Float)
        imu_raw_.IM1R.roll = bytesToFloat(data + 37);
        // D41-D44: 偏航角 Yaw (4字节 Float)
        imu_raw_.IM1R.yaw = bytesToFloat(data + 41);

        // D45-D48: q0
        imu_raw_.IM1R.quat[0] = bytesToFloat(data + 45 );
        // D49-D52: q1
        imu_raw_.IM1R.quat[1] = bytesToFloat(data + 49 );
        // D53-D56: q2
        imu_raw_.IM1R.quat[2] = bytesToFloat(data + 53 );
        // D57-D60: q3
        imu_raw_.IM1R.quat[3] = bytesToFloat(data + 57 );

        // D61-D62: 温度 (2字节 Int16, 系数 0.1)
        short temp_raw = bytesToShort(data + 61);
        imu_raw_.IM1R.temp = temp_raw * 0.1f;

        return true;
    } catch (...) {
        return false;
    }
}//ghd+

// 将滚转角、俯仰角、偏航角转换为四元数
void IMUReader::rpyToQuaternion(double roll, double pitch, double yaw, double& qw, double& qx, double& qy, double& qz) 
{
    // 将角度转换为弧度
    roll = roll * M_PI / 180.0;
    pitch = pitch * M_PI / 180.0;
    yaw = yaw * M_PI / 180.0;

    // 计算四元数
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;
}



void IMUReader::quatToZyx(double qw, double qx, double qy, double qz, double& x, double& y, double& z)
{
    // 保证 sin(pitch) 的值不会超过 [-1, 1] 范围，避免 NaN
    double v = -2.0 * (qx * qz - qw * qy);
    v = std::max(-0.99999999, std::min(0.9999999999, v));
    

    // 计算欧拉角
    z = std::atan2(2 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz);
    y = std::asin(v);
    x = std::atan2(2 * (qy * qz + qw * qx), qw * qw - qx * qx - qy * qy + qz * qz);

    // // 将弧度转换为度（可选）
    // z = z * 180.0 / M_PI;
    // y = y * 180.0 / M_PI;
    // x = x * 180.0 / M_PI;
}

void IMUReader::rotateQuatAroundY180(double& qw, double& qx, double& qy, double& qz)
{
    // 定义绕y轴旋转180度的四元数
    double qw_rot = 0.0;
    double qx_rot = 0.0;
    double qy_rot = 1.0;
    double qz_rot = 0.0;

    // 保存原四元数
    double ow = qw, ox = qx, oy = qy, oz = qz;

    // 四元数乘法：new_q = original_q * rotation_q
    qw = ow * qw_rot - ox * qx_rot - oy * qy_rot - oz * qz_rot;
    qx = ow * qx_rot + ox * qw_rot + oy * qz_rot - oz * qy_rot;
    qy = ow * qy_rot - ox * qz_rot + oy * qw_rot + oz * qx_rot;
    qz = ow * qz_rot + ox * qy_rot - oy * qx_rot + oz * qw_rot;
}


Eigen::Vector3d IMUReader::computeGravityVec(double roll, double pitch)
{
    // 1. RPY → quaternion
    Eigen::AngleAxisd rollAngle(roll * M_PI/180.0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch * M_PI/180.0, Eigen::Vector3d::UnitY());
    Eigen::Quaterniond q = pitchAngle * rollAngle;

    // 2. IMU倒装修正（Y轴180°）
    Eigen::Quaterniond q_flip(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));

    Eigen::Quaterniond q_corrected = q_flip * q;

    // 3. 重力向量
    Eigen::Vector3d g_world(0,0,-1);

    // 4. inverse rotate
    Eigen::Vector3d g_body = q_corrected.conjugate() * g_world;

    // 对齐旧代码
    g_body.x() = -g_body.x();
    g_body.y() = -g_body.y();


    return g_body;
}

// uint8_t IMUReade::CRC8_Calculate(const uint8_t *data, uint8_t len) {
//     uint8_t crc = 0x00; // 初始值
//     const uint8_t polynomial = 0x07; // 多项式

//     for (uint8_t i = 0; i < len; i++) {
//         crc ^= data[i];
//         for (uint8_t bit = 0; bit < 8; bit++) {
//             if (crc & 0x80) {
//                 crc = (crc << 1) ^ polynomial;
//             } else {
//                 crc <<= 1;
//             }
//         }
//     }
//     return crc;
// }

//ghd++ 计算 NMEA 0183 格式的校验和 
std::string IMUReader::calculateChecksum(const std::string& command) {
    uint8_t checksum = 0;
    // 校验和计算从 '$' 之后开始，到 '*' 之前结束
    for (size_t i = 1; i < command.length(); ++i) {
        checksum ^= command[i];
    }
    
    char checksum_str[3];
    sprintf(checksum_str, "%02X", checksum);
    return std::string(checksum_str);
}//ghd++

// 发送IM1R配置指令并等待 ACK 响应
bool IMUReader::sendConfigCommand(const std::string& command) {
    if (fd_ < 0) {
        log_error("Serial port is not open.");
        return false;
    }

    // 1. 拼接完整指令并计算校验和
    std::string full_command = command + "*" + calculateChecksum(command) + "\r\n";

    // 2. 清空串口缓冲区，防止旧数据干扰
    tcflush(fd_, TCIFLUSH);

    // 3. 发送指令
    int bytes_written = serial_port_write(fd_, full_command.c_str(), full_command.length());
    if (bytes_written < 0) {
        log_error("Failed to write command to serial port.");
        return false;
    }
    log_info("Config command sent: %s", full_command.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    // 4. 循环读取，从数据流中“捞”出 ACK 响应
    char read_buf[256];
    std::string response_buffer;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(4);

    while (true) {

        auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {//超时判断
            log_warn("Timeout waiting for ACK response for command: %s", command.c_str());
            return false;
        }

        // 尝试读取数据
        int bytes_read = serial_port_read(fd_, read_buf, sizeof(read_buf) - 1);
        
        if (bytes_read > 0) {
            // 直接处理读取到的字节，避免不必要的字符串拼接和查找
            response_buffer.append(read_buf, bytes_read);
            
            // 在累积的响应中查找 ACK
            size_t ack_pos = response_buffer.find("$DSIMC,ACK");
            if (ack_pos != std::string::npos) {
                //提取并打印完整的ACK响应
                size_t end_pos = response_buffer.find("\r\n", ack_pos);
                if (end_pos != std::string::npos) {
                    std::string ack_line = response_buffer.substr(ack_pos, end_pos - ack_pos);
                    log_info("Successfully received ACK: %s", ack_line.c_str());
                    return true;
                }
            }
            
            if (response_buffer.length() > 1024) {
                response_buffer = response_buffer.substr(response_buffer.length() - 512);
            }
        }

    }
    return true;
}

