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
        std::cout<<reader.acc_new<<std::endl;
        // // reader.rpyToQuaternion();
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }


    // if (!reader.start()) {
    //     return -1;
    // }
    return 0;
}