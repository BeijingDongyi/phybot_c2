#include "HipnucReader.h"
#include "commands.h"
#include "global_options.h"
#include "log.h"
#include <cstdint>
#include <ctime>
#include <cstring>
#include "serial_port.h"    // 假设你有串口操作的函数
#include "hipnuc_dec.h"
#include "nmea_dec.h"
#include "hex2bin.h"
#include "kboot.h"
#include "log.h"
#include <iostream>
#include <thread>
#include <chrono>
int main() {


    HipnucReader reader;


    reader.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    while(1)
    {
        std::cout<<"x: "<<reader.rpy(0)<<std::endl;
        std::cout<<"y: "<<reader.rpy(1)<<std::endl;
        std::cout<<"z: "<<reader.rpy(2)<<std::endl;
        
        std::cout<<"z: "<<reader.zyx(0)<<std::endl;
        std::cout<<"y: "<<reader.zyx(1)<<std::endl;
        std::cout<<"x: "<<reader.zyx(2)<<std::endl;
        std::cout<<"g: "<<reader.gravity_vec<<std::endl;
        std::cout<<"xacc: "<<reader.acc(0)<<std::endl;
        std::cout<<"yacc: "<<reader.acc(1)<<std::endl;
        std::cout<<"zacc: "<<reader.acc(2)<<std::endl;
        std::cout<<"xacc_new: "<<reader.acc_new(0)<<std::endl;
        std::cout<<"yacc_new: "<<reader.acc_new(1)<<std::endl;
        std::cout<<"zacc_new: "<<reader.acc_new(2)<<std::endl;
        std::cout<<"xvel: "<<reader.ang_vel(0)<<std::endl;
        std::cout<<"yvel: "<<reader.ang_vel(1)<<std::endl;
        std::cout<<"zvel: "<<reader.ang_vel(2)<<std::endl;
        std::cout<<"xvel_new: "<<reader.ang_vel_new(0)<<std::endl;
        std::cout<<"yvel_new: "<<reader.ang_vel_new(1)<<std::endl;
        std::cout<<"zvel_new: "<<reader.ang_vel_new(2)<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::cout << "\033[2J\033[H";
        // // reader.rpyToQuaternion();
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }


    // if (!reader.start()) {
    //     return -1;
    // }
    return 0;
}