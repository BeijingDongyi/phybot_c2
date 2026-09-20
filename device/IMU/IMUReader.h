// HipnucReader.h
#pragma once

#include <cstdint>
#include <ctime>
#include <cstring>
#include "serial_port.h"
#include "log.h"
#include <Eigen/Dense>
#include <thread>

class IMUReader {
public:

    bool start();
    Eigen::Vector4d quat = Eigen::Vector4d::Zero();
    Eigen::Vector4d quat_no_yaw = Eigen::Vector4d::Zero();
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity_vec = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_new = Eigen::Vector3d::Zero();
    Eigen::Vector3d ang_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d ang_vel_new = Eigen::Vector3d::Zero();


    Eigen::Vector3d zyx = Eigen::Vector3d::Zero();

    void rpyToQuaternion(double roll, double pitch, double yaw, double& qw, double& qx, double& qy, double& qz) ;
    void quaternionToRPY(double qw, double qx, double qy, double qz, double& roll, double& pitch, double& yaw);
    void quatToZyx(double qw, double qx, double qy, double qz, double& x, double& y, double& z);
    Eigen::Vector3d computeGravityVec(double roll, double pitch);
    void rotateQuatAroundX180(double& qw, double& qx, double& qy, double& qz);
    void rotateQuatAroundY180(double& qw, double& qx, double& qy, double& qz);
    void rotateQuatAroundZ90(double& qw, double& qx, double& qy, double& qz);
    bool parseIMUBinaryPacket(const std::vector<unsigned char>& packet);

private:

    typedef struct __attribute__((__packed__))
    {
        uint8_t         tag;            /* Data packet tag, if tag = 0x00, means that this packet is null */
        uint16_t        sttaus;         /* reserved */
        int8_t          temp;           /* Temperature */
        float           air_pressure;   /* Pressure */
        uint32_t        system_time;    /* Timestamp */
        float           acc[3];         /* Accelerometer data (x, y, z) */
        float           gyr[3];         /* Gyroscope data (x, y, z) */
        float           mag[3];         /* Magnetometer data (x, y, z) */
        float           roll;           /* Roll angle */
        float           pitch;          /* Pitch angle */
        float           yaw;            /* Yaw angle */
        float           quat[4];        /* Quaternion (w, x, y, z) */

    } IM1R_t;

    typedef struct
    {
        int nbyte;                          /* Number of bytes in message buffer */ 
        int len;                            /* Message length (bytes) */
        uint8_t buf[256];   /* Message raw buffer */
        IM1R_t IM1R;                        /* Decoded IM1R packet data */

    } IMU_raw_t;

    int fd_;
    char recv_buf[100];
    char log_buf_[512];
    IMU_raw_t imu_raw_ = {0};
    //nmea_raw_t nmea_raw_= {0};
    struct timespec last_time_;
    struct timespec last_display_time_;
    long long frame_count_;
    int frame_rate_ = 0;
    double elapsed_time_;
    uint8_t CRC8_Calculate(const uint8_t *data, uint8_t len);
    const char* port_name = "/dev/ttyUSB0";
    int baud_rate = 921600;
    int safe_sleep(unsigned long usec);

    void mainLoop();
    std::string calculateChecksum(const std::string& command);
    bool sendConfigCommand(const std::string& command);
    bool changeBaudrate(int fd, speed_t baudrate);
};
