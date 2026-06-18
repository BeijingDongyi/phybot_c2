#ifndef TYPES_H
#define TYPES_H

#include <string>
#include <atomic>
#include <vector>
#include <list>

//添加电机参数
#include "UserParametersList.h"


using namespace std;
#define SDOREQUEST 0x600
#define SDORECEIVE 0x580
#define HEARTBEAT  0x700
#define PDORECEIVE 0x180

#define MOTOROTA 0x81


typedef enum : uint8_t{

    ReadData = 0x03,  //读数据
    WriteData = 0x06  //写数据

}Red_Wrt;

typedef enum {

    CTRL_NONE = 0,

    CTRL_SERVO_OFF = 1,    // 失能
    CTRL_SERVO_ON = 2,     // 使能
    CTRL_CLEAR_FAULT = 3,  // 清除错误

    RESET_DEFAULT = 4,  // 恢复默认参数

    // 传感器相关
    CTRL_CALIBRATE_ENCODER = 5,  // 编码器校准
    CTRL_RETURN_ZERO = 6 ,      // 执行器自动回零
    CTRL_POSITION_SET_ZERO = 7, // 执行器位置置零

    // 控制相关
    CTRL_FRICTION_IDENTIFY = 8,  // 摩擦校准
    CTRL_COGGING_IDENTIFY = 9,   // 齿槽转矩校准

    // 功能开关 0x3xxxF
    CTRL_FRICTION_COMP_OFF = 10,       // 失能摩擦补偿
    CTRL_FRICTION_COMP_ON = 11,        // 使能摩擦补偿
    CTRL_COGGING_COMP_OFF = 12,        // 失能齿槽转矩补偿
    CTRL_COGGING_COMP_ON = 13,         // 使能齿槽转矩补偿
    CTRL_TLOAD_COMP_OFF = 14,          // 失能负载补偿
    CTRL_TLOAD_COMP_ON = 15,           // 使能负载补偿
    CTRL_SOFT_POS_LIMIT_OFF = 16,      // 失能软限位
    CTRL_SOFT_POS_LIMIT_ON = 17,       // 使能软限位
    CTRL_OVER_TEMP_PROTECT_OFF = 18,   // 失能过温保护
    CTRL_OVER_TEMP_PROTECT_ON = 19,    // 使能过温保护
    CTRL_HEART_BEAT_PROTECT_OFF = 20,  // 失能心跳保护
    CTRL_HEART_BEAT_PROTECT_ON = 21,   // 使能心跳保护

} ControlWord_e;

// 电机控制模式
typedef enum {

    MOTOR_CTRL_MODE_NONE = 0,         // 无控制模式
    MOTOR_CTRL_MODE_CURRENT = 0x0A,      // 电流控制模式
    MOTOR_CTRL_MODE_VELOCITY = 0x09,     // 速度控制模式
    MOTOR_CTRL_MODE_POSITION = 0x08,     // 位置控制模式
    MOTOR_CTRL_MODE_PD = 0x0B,        // 转矩位置控制模式
    MOTOR_CTRL_MODE_BRAKE = 5,        // 制动模式
    MOTOR_CTRL_MODE_OPENLOOP = 0x0C,  // 开环控制模式

} MotorCtrlMode_e;

// 电机状态码
typedef enum {

    IDLE_WM = 0,                  // 空闲状态
    Init_WM = 1,                  // 初始化状态
    Normal_WM = 2,                // 正常控制状态
    Fault_WM = 3,                 // 异常状态
    ENCODER_CAIL_WM = 4,          // 特殊状态 电机编码器校准
    LINER_HALL_CAIL_WM = 5,       // 特殊状态 电机编码器校准
    OUTPUT_ENCODER_CAIL_WM = 6,   // 特殊状态 出轴编码器校准
    ACTUATOR_RETURN_ZERO_WM = 7,  // 特殊状态 执行器回零模式
    FRICTION_IDENTIFY_WM = 8,     // 特殊状态 摩擦辨识模式

} UserWorkMode_e;

//通信控制指令
typedef enum : uint16_t{
  
    ConfigNetWork = 0x600,        //网络通信参数设置
    SetId = 0x2001,               //设置电机ID
    ControlMode = 0x6060,         //控制模式
    ControlWord = 0x6040,         //控制字
    WorkMode,                     //工作模式
    FastMode = 0x1800,            //快速上报模式
    Reboot,                       //软件重启                     
    ControlSetPos = 0x607A,       //指令位置
    ControlSetVel = 0x60FF,       //指令速度
    ControlSetTor =0x6071,        //指令力矩
    ControlSetCur,                //指令电流
    ControlSetPosPID = 0x2020,    //指令位置环PID
    ControlSetVelPID = 0x2021,    //指令速度环PID
    ControlSetPID,
    ControlSetPD = 0x2022,        //指令PD
    GetMotorPVCT,                 //反馈PVCT
    ControlSetLimmit = 0x2011,     //设置电机限制
    NMT = 0x0100,                 //NMT状态机开启
    SaveParamToFlash = 0x200A,     //保存参数到flash
    Inverter,                     //反馈逆变器状态
    Heartbeat = 0x1017,           //设置电机心跳
    SetDirction,                  //指令方向
    SetOutputShaftRatio,          //指令减速比
    SetPosThreshold,              //指令位置限制
    SetVCTMax,                    //指令最大VCT
    SetOutputOffset,              //指令出轴传感器位置偏置
    SetOutEnable,                 //指令输出轴编码器使能
    SetTorToCur,                  //指令转矩到电流多项式系数

    AcBaseParams,                 //执行器基础参数集合

    SendCalibrationData,	    // 发送校准数据到SDK
    SetFocThetaSorce,			// 设置角度源
    SetM1Voltage,				// 设置强拖电压

    OTA_CMD_HandShanking = 1563u,      // OTA 握手
    OTA_CMD_UpDateStart,       // OTA 开始下载
    OTA_CMD_FileTransmission,  // OTA 文件传输
    OTA_CMD_UpDateEnd,         // OTA 下载结束
    OTA_CMD_SendVersion,       // OTA 发送版本号
    Get_MotorID = 0x620,        // 获取电机
    DrvInfo = 0X2002,          //驱动器信息结构
    SetMotor_Mode,			   // 设置电机类型  
}Command_Types;




//网络参数
typedef struct{

     uint32_t LocalIP;               //本地IP
    uint16_t LocalPort;              //本地端口
    uint32_t TargetIP;               //目标IP

} NetWorkParams;

typedef struct{

    float PVCT_InterfaceLpfWc_V1;           //位置反馈滤波器截止频率
    float PVCT_InterfaceLpfWc_V2;           //速度反馈滤波器截止频率
    float PVCT_InterfaceLpfWc_V3;           //电流/力矩反馈滤波器截止频率

} PVCT_InterfaceLpfWc;



//FastMode参数
typedef struct{

    std::atomic<int> FastPos_;     //位置
    std::atomic<short> FastVel_;   //速度
    std::atomic<short> FastTor_e_;  //力矩
    std::atomic<uint8_t> FastStateMechine;  //stateMechine
    std::atomic<uint8_t> FastMosTemperature; //mos 温度
    std::atomic<uint8_t> FastWindingTemperature; //绕组温度
    std::atomic<uint8_t> FastBusVoltage;   //母线电压
    std::atomic<uint32_t> FastErrorCode;   //错误码

}FastParams;

typedef struct
{
    float MAX_CURRENT;    //最大电流R/W
    float MAX_SPEED;      //最高转速R/W
    float MAX_ACC;        //最大加速度R/W
    float MIN_POSITION;   //最小位置限制R/W
    float MAX_POSITION;   //最大位置限制R/W

}MotorLimit;

typedef struct
{
    float PID_Speed_P;  //速度环P
    float PID_Speed_I;  //速度环I
    float PID_Speed_D;  // 速度环D
    float PID_Speed_OUTMAX; //速度环最大输出

}MotorVelPID;

typedef struct
{
    float PID_Pos_P;  //位置环P
    float PID_Pos_I;  //位置环I
    float PID_Pos_D;  // 位置环D
    float PID_Pos_OUTMAX; //位置环最大输出

}MotorPosPID;

typedef struct
{
    float PD_P;  //pd环P
    float PD_D;  //pd环D
    float PD_OUTMAX; //pd环最大输出

}MotorPD;
//创建电机对象参数
typedef struct{
    uint16_t canId;     //can  id
    uint16_t canLind;   //can线 id
}MotorCan;










typedef struct
{
   uint32_t sysModelType; //驱动器型号
   uint32_t polePairs; //极对数
   uint32_t reduRatio; //模块减速比
   uint32_t hardWareVersion; //硬件版本
   uint32_t softWareVersion; //软件版本
   uint32_t encoderRawValue; //编码器值
   uint32_t openloopVoltage; //开环强脱电压 
} DevInfo;


//电机所有参数
typedef struct{

    uint32_t ControlMode_;
    uint32_t WorkMode_;

    uint32_t Pos_;
    uint32_t Vel_;
    uint32_t Cur_;
    uint32_t Tor_l_;
    uint32_t Tor_e_;
    uint32_t Fault_;
    uint32_t FaultExt2_;
    uint32_t FaultExt3_;
    uint32_t FaultExt4_;
    uint32_t MosTemp_;
    uint32_t CoilTemp_;
    uint32_t VBusTemp_;
    uint32_t State; //状态版本号
    uint32_t Major; //主版本号
    uint32_t Minor; //次版本号
    uint32_t Patch; //修订版本号
    int32_t EncoderValue; //编码器值
    std::atomic<uint16_t> MotorId;   //电机id 
    std::atomic<uint16_t> canLineId; //can线id
    std::atomic<int> ACK_;
    std::atomic<int> CurrCmd;
    std::atomic<int> Serial_Number; //传输文件序号
    char model[20];  //电机型号
    MotorLimit motorLimit_;
    MotorVelPID motorVelPID_;
    MotorPosPID motorPosPID_;
    MotorPD motorPD_;
    std::list<MotorCan> MotorCanList;
    ParamList_t paramList;
    DevInfo devInfo;
} TotalParams;


//存储升级文件组
using ByteGroup = std::vector<std::uint8_t>;



#define Profile_Position_mode 0x01//轮毂位置模式（预留）
#define Profile_Velocity_mode 0x03//轮毂速度模式（预留）
#define Profile_Torque_mode 0x04//轮毂扭矩模式（预留）
#define Homing_mode 0x06//回零模式
#define Interpolated_Position_mode 0x07//位置插补模式（预留）
#define Cyclic_Synchronous_Position_mode 0x08//周期同步位置模式
#define Cyclic_Synchronous_Velocity_mode 0x09//周期同步速度模式
#define Cyclic_Synchronous_Torque_mode 0x0A//周期同步转矩模式
#define Cyclic_Synchronous_MIT_mode 0x0B//MIT


// 设备信息结构体
typedef struct{

    char MainEditionIp[16]; //主机Ip
    uint16_t MainEditionPort; //主机端口
    uint32_t MainID; //主机ID
} DeviceInfo;



//版本号
//主版本.次版本.修订版本
//│      │      │        
//│      │      │        
//│      │      │        
//│      │     向下兼容的 Bug 修复
//│      └ 向下兼容的功能新增
//└不兼容的 API 变更
#define VERSION "2.2.0"

#endif // TYPES_H