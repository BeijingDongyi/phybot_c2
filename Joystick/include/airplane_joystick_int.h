#ifndef __AIRPLANE_JOYSTICK_INT_H__
#define __AIRPLANE_JOYSTICK_INT_H__
/*--------------------------------------------------------------------------*/
/* include */
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>
#include <mutex>
#include <cmath>
#include <iostream>
#include <vector>
#include <cstdint>
#include <sys/ioctl.h>
#include <chrono>
#include <linux/input.h>
#include <linux/joystick.h>
#include <termios.h>
#include <cstring>  
#include "DataPackage/include/DataPackage.h"

/*--------------------------------------------------------------------------*/
#ifndef BOTHER
#define BOTHER 0010000
#endif
#ifndef TCGETS2
#define TCGETS2 0x802C542A
#endif
#ifndef TCSETS2
#define TCSETS2 0x402C542B
#endif

/*--------------------------------------------------------------------------*/

#define XBOX_TYPE_BUTTON        0x01
#define XBOX_TYPE_AXIS          0x02


#define XBOX_BUTTON_E           5
#define XBOX_BUTTON_G           6
#define XBOX_BUTTON_H           7
#define XBOX_BUTTON_F           8
#define XBOX_BUTTON_A           9
#define XBOX_BUTTON_B           10
#define XBOX_BUTTON_X           11      /* 映射为C键 */
#define XBOX_BUTTON_Y           12      /* 映射为D键 */
#define XBOX_BUTTON_LB          0x04
#define XBOX_BUTTON_RB          0x05
#define XBOX_BUTTON_START       0x06
#define XBOX_BUTTON_BACK        0x07
#define XBOX_BUTTON_HOME        0x08
#define XBOX_BUTTON_LO          0x09    /* 左摇杆按键 */
#define XBOX_BUTTON_RO          0x0a    /* 右摇杆按键 */

#define XBOX_AXIS_LX            4       /* 左摇杆X轴 */
#define XBOX_AXIS_LY            3       /* 左摇杆Y轴 */
#define XBOX_AXIS_RX            1       /* 右摇杆X轴 */
#define XBOX_AXIS_RY            2       /* 右摇杆Y轴 */
#define XBOX_AXIS_LT            0x02
#define XBOX_AXIS_RT            0x05
#define XBOX_AXIS_XX            0x06    /* 方向键X轴 */
#define XBOX_AXIS_YY            0x07    /* 方向键Y轴 */

#define XBOX_AXIS_VAL_UP        -32767
#define XBOX_AXIS_VAL_DOWN      32767
#define XBOX_AXIS_VAL_LEFT      -32767
#define XBOX_AXIS_VAL_RIGHT     32767

#define XBOX_AXIS_VAL_MIN       -32767
#define XBOX_AXIS_VAL_MAX       32767
#define XBOX_AXIS_VAL_MID       0x00

#define XBOX_BUTTON_ON          0x01
#define XBOX_BUTTON_OFF         0x00

/*--------------------------------------------------------------------------*/
/* SBUS 帧参数 */
const uint8_t SBUS_FRAME_HEADER = 0x0F;     /* 帧头 */
const uint8_t SBUS_FRAME_TAIL_1 = 0x00;     /* 帧尾1 */
const uint8_t SBUS_FRAME_TAIL_2 = 0x0D;     /* 帧尾2（兼容部分设备） */
const size_t  SBUS_FRAME_LENGTH = 25;       /* 帧长度 */
const size_t  SBUS_CHANNEL_COUNT = 16;      /* 通道数 */
/*--------------------------------------------------------------------------*/
/**
 * @brief joystick:
 * author : rxy
 * email : rxy19940622@126.com
 */
typedef struct xbox_map {
    /* 时间戳 */
    int time;
    /* 按钮与拨杆 */
    int a;
    int b;
    int x;
    int y;
    int f;
    int h;
    int g;
    int e;

    /* 摇杆 */
    int lx;
    int ly;
    int rx;
    int ry;
} xbox_map_t;


struct termios2 {
    tcflag_t c_iflag;    /* 输入模式标志 */
    tcflag_t c_oflag;    /* 输出模式标志 */
    tcflag_t c_cflag;    /* 控制模式标志 */
    tcflag_t c_lflag;    /* 本地模式标志 */
    cc_t c_line;         /* 线路规程 */
    cc_t c_cc[19];       /* 控制字符 */
    speed_t c_ispeed;    /* 输入波特率 */
    speed_t c_ospeed;    /* 输出波特率 */
};

/*--------------------------------------------------------------------------*/
/* 键位→状态映射 */
struct KeyStateMapping {
    int xbox_map_t::*button;
    int match_value;
    State state;
};

inline constexpr KeyStateMapping joystick_mappings[] = {
    {&xbox_map_t::x, 1, State::ZERO},           
    {&xbox_map_t::b, 1, State::RL_walk},       
    {&xbox_map_t::a, 1, State::RL_climb_up},    
    {&xbox_map_t::y, 1, State::RL_crane_down},  
};

inline constexpr KeyStateMapping joystick_mappings_G_right[] = {
    {&xbox_map_t::x,  1, State::RL_taichi},  
    {&xbox_map_t::b,  1, State::RL_long_motion},   
    {&xbox_map_t::a,  1, State::RL_mimic},          
    {&xbox_map_t::y,  1, State::RL_kongfu},  
};

inline constexpr KeyStateMapping joystick_mappings_H_left[] = {
    {&xbox_map_t::b,  1, State::RL_dance},   
    {&xbox_map_t::a,  1, State::RL_introduct},          
};

inline constexpr KeyStateMapping joystick_mappings_H_right[] = {
    {&xbox_map_t::x,  1, State::RL_ASE_stand},  
    {&xbox_map_t::b,  1, State::RL_ASE_combinations},   
    {&xbox_map_t::a,  1, State::RL_ASE_sidekick},          
    {&xbox_map_t::y,  1, State::RL_ASE_uppercut},           
};
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
class AirPlaneJoystick {
  public:
    AirPlaneJoystick();
    ~AirPlaneJoystick();
    void init();
    void xbox_run();
    void SetDataToPackage(DataPackage &DataPackage);
    void GetDataFromPackage(DataPackage &DataPackage);


    //飞机手柄操作杆
    int ly_deadarea = 50;
    int ly_center = 1002;
    int ly_maxvalue = 1722;
    int ly_mixvalue = 282;

    int lx_deadarea = 50;
    int lx_center = 501;
    int lx_maxvalue = 861;
    int lx_minvalue = 141;

    int ry_deadarea = 50;
    int ry_center = 1002;
    int ry_maxvalue = 282;
    int ry_mixvalue = 1722;

    int rx_deadarea = 50;
    int rx_center = 1002;
    int rx_maxvalue = 1722;
    int rx_mixvalue = 282;




    int maxvalue = 32767;
    int deadarea = 5000;
    double lx_dir = 1.0;
    double ly_dir = 1.0;

    double rx_dir = -1.0;
    double ry_dir = -1.0;

    double xx_dir = -1.0;
    double yy_dir = -1.0;
    // get command  velocity for walk
    double get_walk_x_direction_speed();
    // x + direction
    double maxspeed_x = 1.5;
    // x - direction
    double minspeed_x = -0.5;

    double get_walk_yaw_direction_speed();
    double maxspeed_yaw = 1.3;
    double minspeed_yaw = -1.3;

    double get_stand_up_pos();
    double max_stand_up_pos = 0.03;
    double min_stand_up_pos = -0.05;

    double get_stand_forward_pos();
    double max_stand_forward_pos = 0.05;
    double min_stand_forward_pos = -0.03;

    double get_stand_left_pos();
    double max_stand_left_pos = 0.05;
    double min_stand_left_pos = -0.05;

    double get_stand_yaw_pos();
    double max_stand_yaw_pos = 0.3;
    double min_stand_yaw_pos = -0.3;

    double get_stand_up_vel();
    double max_stand_up_vel = 0.08;
    double min_stand_up_vel = -0.08;

    double get_stand_forward_vel();
    double max_stand_forward_vel = 0.02;
    double min_stand_forward_vel = -0.01;

    double get_stand_left_vel();
    double max_stand_left_vel = 0.02;
    double min_stand_left_vel = -0.02;

    double get_stand_yaw_vel();
    double max_stand_yaw_vel = 0.1;
    double min_stand_yaw_vel = -0.1;

    double get_walk_y_direction_speed();
    double maxspeed_y = 0.1;
    double minspeed_y = -0.1;

    void get_mimic_mode();
    int mimic_mode{1};

    //打开串口设备
    int open_serial(const char* port_name);
    //解析串口数据
    std::vector<uint16_t> parse_sbus_frame(const std::vector<uint8_t>& frame);
    //映射按钮
    int Serial_map_read(std::vector<uint16_t> channels) ;

    //临时计算
    double left_y(double ly,double MinCoefficient,double MaxCoefficient);
    double right_x(double rx,double MinCoefficient,double MaxCoefficient);
    double right_y(double ry,double MinCoefficient,double MaxCoefficient); 
    double left_x(double ry,double MinCoefficient,double MaxCoefficient); 

    void KeyProtect();

    void get_speed();
    void get_state_change();     // 按键→NextState (F拨杆选ASE组, ABXY选具体动作)
    void get_control_mode();
    // void change_state_by_speed();
    void run();
    double js_vx_desire{0};
    double js_vx_offset{0};
    double js_vx_control{0};
    double js_vx_feedback{0};
    double neck_yaw_xx{0};
    double neck_pitch_yy{0};
    double kp_js_vx;
    double control_period=0.002;
    double speed_forward_avg{0};
    int control_mode{0};

    double js_OmegaZ_desire{0};
    double js_OmegaZ_control{0};

    double vx_Final{0};

    double js_vy_desire{0};
    double js_vy_offset{0};

    double stand_up_pos{0};
    double stand_forward_pos{0};
    double stand_left_pos{0};
    double stand_yaw_pos{0};

    // void get_stand_pos();

    double stand_up_vel{0};


    int ase_skill_id{-1};      // ASE skill ID (-1=非ASE, 0~19=supernice, 0~18=fast_kick)
    int ase_profile_id{0};     // ASE profile: 0=supernice(F↑), 1=fast_kick(F↓)
    int ase_skill_loop{0};     // ASE 循环标志
    int ase_skill_trigger_count{0};  // ASE 触发计数

  
    // fsmstate
    std::string current_fsmstate_command;
    std::string current_motion_command;
    //
    xbox_map_t xbox_m;
    // pthread_t xbox_thread;
    std::thread xbox_thread;
    std::mutex data_mutex;
    // xbox
    int xbox_fd;
    // int xbox_open(const char *file_name);
    // int xbox_map_read(xbox_map_t *map);
    void xbox_close(void);
    int xbox_init(void);
private:
    double acc_max;
    double angacc_max;
    State NextState{State::ZERO};
    State last_NextState{State::ZERO};
    ZeroPoseType CurrentZeroState{ZeroPoseType::Standing}; 
    std::string m_dev_path;
};
/*--------------------------------------------------------------------------*/
#endif
