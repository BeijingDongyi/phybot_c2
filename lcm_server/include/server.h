#ifndef SERVER_H_
#define SERVER_H_

/*--------------------------------------------------------------------------*/
/* include */
#include "DataPackage/include/DataPackage.h"
#include "lcm_server/src/lcmcommunicator.h"
#include <iostream>
#include <chrono>
#include <csignal>
#include <random>
#include <thread>
#include <mutex>
#include <atomic>
#include "lcm_server/custom_lcm/system2Algorithm_packet.hpp"
#include "MotorList/include/Common.h"

/* 定义帧类型常量 */ 
#define FRAME_TYPE_APP_STARTUP          0xD001  /* 应用启动消息 */ 
#define FRAME_TYPE_APP_STARTUP_REP      0xE001  /* 应用启动回报 */
#define FRAME_TYPE_ROBOT_CONTROL        0x0001  /* 机器人控制指令 */
#define FRAME_TYPE_ROBOT_CONTROL_REP    0x0002  /* 机器人控制回报 */
#define FRAME_TYPE_BMS_BASIC_DATA       0xD051  /* BMS基础信息，上报topic: BMS_BasicData */
#define FRAME_TYPE_BMS_BASIC_DATA_REP   0xE051  /* BMS基础信息接收回报topic: BMS_BasicData_rep */

/*--------------------------------------------------------------------------*/
struct Base_DriveCmd {
    // 3. 模式（1字节）：
    //              0 ： 初始状态
    //              1 ： 零位
    //              2 ： 走路
    //              3 ： 挥手
    //              4 ： 出拳舞
    //              5 ： 出拳
    //              6 ： 踢腿
    //              7 ： 爬起
    //              8 ： 趴下
    //              9 ： 太极
    //              10： 功夫
    //              11： 左介绍
    //              12： 双手介绍2
    //              13： (预留)
    //              14： 右介绍
    //              15： 挠头
    //              16： 双手介绍1
    //              17： 双手挥手
    //              18： 握手
    //              19： 自我介绍
    //              20： 舞蹈
    int8_t model;    

    // 4. x速度（4字节）：（-0.9～0.9）
    float x_velocity; 

    // 5. y速度（4字节）：（-1.0～1.0）
    float y_velocity;  

    // 6. 角速度（4字节）：（-0.6～0.6）
    float angular_velocity;  
};

struct ApplicationStartupMessage
{
    //! \brief      当前机型支持的动作列表最大支持32个动作     bit占位
    //!             D0: 归零      1-支持该动作      0-不支持该动作
    //!             D1: 走路      1-支持该动作      0-不支持该动作
    //!             D2: 挥手      1-支持该动作      0-不支持该动作
    //!             D3: 出拳舞     1-支持该动作      0-不支持该动作
    //!             D4: 出拳      1-支持该动作      0-不支持该动作
    //!             D5: 踢腿      1-支持该动作      0-不支持该动作
    //!             D6: 爬起      1-支持该动作      0-不支持该动作
    //!             D7: 趴下      1-支持该动作      0-不支持该动作
    //!             D8: 太极      1-支持该动作      0-不支持该动作
    //!             D9: 功夫      1-支持该动作      0-不支持该动作
    //!             D10: 左介绍     1-支持该动作      0-不支持该动作
    //!             D11: 双手介绍2  1-支持该动作      0-不支持该动作
    //!             D12: (预留)     1-支持该动作      0-不支持该动作
    //!             D13: 右介绍     1-支持该动作      0-不支持该动作
    //!             D14: 挠头      1-支持该动作      0-不支持该动作
    //!             D15: 双手介绍1  1-支持该动作      0-不支持该动作
    //!             D16: 双手挥手   1-支持该动作      0-不支持该动作
    //!             D17: 握手      1-支持该动作      0-不支持该动作
    //!             D18: 自我介绍   1-支持该动作      0-不支持该动作
    //!             D19: 舞蹈      1-支持该动作      0-不支持该动作
    //!             D20~D31 预留
    int32_t action_list ;

    //! \brief      线速度上限
    float vx_max ;

    //! \brief      线速度下限
    float vx_min;

    //! \brief      线速度上限
    float vy_max;

    //! \brief      线速度下限
    float vy_min ;

    //! \brief      角速度上限
    float omegaz_max ;

    //! \brief      角速度下限
    float omegaz_min ;
};

struct ApplicationStartupMessage_rep
{
    //! \brief      监听算法程序成功        0x01-成功，其他则失败
    int8_t listen_successful;
};

//! \brief      控制消息接收回报        周期(500Hz)
//!         算法->后端
struct Base_DriveCmd_rep
{
    //! \brief  当前执行动作
    uint8_t action_cmd ;
    
    //! \brief  0x00-无效 0x01-执行成功 0x02-正在执行 0x03-执行失败
    int8_t action_Result;
};

//! \brief 低电量控制数据获取 topic:BMS_BasicData frame_type=0xD051 周期1Hz
//!        算法->后端
using BMS_BasicData = BMSBasicData_t;

//! \brief 低电量控制数据回报 topic:BMS_BasicData_rep frame_type=0xE051
//!        后端->算法
struct BMS_BasicData_rep
{
    //! \brief 电池状态：0x00-未知，0x01-正常，0x02-低电量，0x03-严重低电量
    int8_t battery_status = 0x00;

    //! \brief 充电字段：0x00-无效，0x01-充电指令
    uint8_t charge = 0x00;
};

/*--------------------------------------------------------------------------*/
/* 模式→状态映射配置表 */
struct ModelStateMapping {
    int model;   /* 模式编号 */
    State state; /* 对应状态 */
};

inline constexpr ModelStateMapping robot_model_mappings[] = {
    {static_cast<uint8_t>(State::ZERO),             State::ZERO},            /* 零位 */
    {static_cast<uint8_t>(State::RL_walk),          State::RL_walk},         /* 走路 */
    {static_cast<uint8_t>(State::RL_mimic),         State::RL_mimic},        /* 招手 */
    {static_cast<uint8_t>(State::RL_long_motion),   State::RL_long_motion},  /* 出拳舞 */
    {static_cast<uint8_t>(State::RL_forward_punch), State::RL_forward_punch},/* 打拳 */
    {static_cast<uint8_t>(State::RL_forward_kick),  State::RL_forward_kick}, /* 踢腿 */
    {static_cast<uint8_t>(State::RL_climb_up),      State::RL_climb_up},     /* 爬起 */
    {static_cast<uint8_t>(State::RL_crane_down),    State::RL_crane_down},   /* 趴下 */
    {static_cast<uint8_t>(State::RL_taichi),        State::RL_taichi},       /* 太极 */
    {static_cast<uint8_t>(State::RL_kongfu),        State::RL_kongfu},       /* 功夫 */
    /*------------------------- 导览(LowerCPG) -------------------------*/
    {static_cast<uint8_t>(State::RL_introduct_left),     State::RL_introduct_left},
    {static_cast<uint8_t>(State::RL_introduct_double_2), State::RL_introduct_double_2},
    {static_cast<uint8_t>(State::RL_introduct_right),    State::RL_introduct_right},
    {static_cast<uint8_t>(State::RL_scratch_head),       State::RL_scratch_head},
    {static_cast<uint8_t>(State::RL_introduct_double_1), State::RL_introduct_double_1},
    {static_cast<uint8_t>(State::RL_wave_double),        State::RL_wave_double},
    {static_cast<uint8_t>(State::RL_handshake),          State::RL_handshake},
    /*------------------------------------------------------------------*/
    {static_cast<uint8_t>(State::RL_introduct),          State::RL_introduct},
    {static_cast<uint8_t>(State::RL_dance),              State::RL_dance},
    /*------------------------- ASE -------------------- ---------------*/
    {static_cast<uint8_t>(State::RL_ASE_combinations),   State::RL_ASE_combinations},
    {static_cast<uint8_t>(State::RL_ASE_sidekick),       State::RL_ASE_sidekick},
    {static_cast<uint8_t>(State::RL_ASE_uppercut),       State::RL_ASE_uppercut},
    {static_cast<uint8_t>(State::RL_ASE_stand),          State::RL_ASE_stand},
    /*------------------------------------------------------------------*/
};

/*----------------------------------------------------------------------*/

class Server {
    
  public:
    Server();
    ~Server();
    void init();
    void SetDataToPackage(DataPackage &DataPackage);
    void run();
    void GetDataFromPackage(DataPackage &DataPackage);
    void Program_begin();
    
  private:
    void pack_ApplicationStartupMessage(const ApplicationStartupMessage &app_msg, 
                                       custom_lcm::system2Algorithm_packet &packet);
    void unpack_ApplicationStartupMessage_rep(const custom_lcm::system2Algorithm_packet &packet, 
                                             ApplicationStartupMessage_rep &app_rep_msg);
    void unpack_Base_DriveCmd(const custom_lcm::system2Algorithm_packet &packet, 
                             Base_DriveCmd &drive_cmd);
    void pack_Base_DriveCmd_rep(const Base_DriveCmd_rep &drive_rep_cmd, 
                               custom_lcm::system2Algorithm_packet &packet);


    std::atomic<bool> g_running{true};
    int8_t calculate_checksum(const custom_lcm::system2Algorithm_packet &packet);
    void get_cmd_vel(const std::string& channel, const unsigned char* data, long unsigned int size);
    void unpack_BMS_BasicData_rep(const unsigned char* data, long unsigned int size);
    void Pub_ApplicationStartupMessage();
    void cmd_vel_run();
    void Pub_BMS_BasicData();
    void Pub_Base_DriveCmd_rep();
    void Heartbeat();
    std::thread base_thread;
    std::thread heartbeat;
    std::mutex data_mutex;
    PHYBOT_TOOL::LcmCommunicator lcm;
    State NextState{State::ZERO};
    State last_NextState{State::ZERO};

    double js_vx_desire{0};
    double js_vy_desire{0};
    double js_OmegaZ_desire{0};

    double neck_yaw_xx{0};
    double neck_pitch_yy{0};
    int control_mode{0};
    int mimic_mode{1};
    int model{0};
    int ase_skill_id{-1};      // ASE skill ID (-1=非ASE, 0~19=supernice, 0~18=fast_kick)
    int ase_profile_id{0};     // ASE profile: 0=supernice(F↑), 1=fast_kick(F↓)
    int ase_skill_loop{0};     // ASE 循环标志
    int ase_skill_trigger_count{0};  // ASE 触发计数
    void update_ase_skill_id(); // 根据 model/NextState 选择 ASE skill
    void KeyProtect();

    ZeroPoseType CurrentZeroState{ZeroPoseType::Standing};  

    int32_t action_list{2097151};
    float vx_max{1.5};
    float vx_min{-1.4};
    float vy_max{0.4};
    float vy_min{-0.4};
    float omegaz_max{0.9};
    float omegaz_min{-0.9};
    bool ApplicationStartupMessage_rep_flag{false};

    /* 控制消息回报 */
    Base_DriveCmd_rep drive_rep_cmd;
    BMS_BasicData bms_basic_data{};
    int8_t bms_battery_status{0};

};
/*--------------------------------------------------------------------------*/
#endif
