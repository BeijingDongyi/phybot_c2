#ifndef DATAPACKAGE_H
#define DATAPACKAGE_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include "yaml-cpp/yaml.h"



#include "StateMachine/include/fsmlist.h"

#include "device/ImuReaderSelector.h"
// #include "/home/phybot/Documents/PhybotSofware/StateMachine/include/fsmlist.h"
/* 控制消息回报枚举 */
enum class ControlMsg_Rep {
    No_Runing,
    Finish,
    Runing,
};
/* 零位状态 */
enum class ZeroPoseType {
    Standing,
    CraneDown
};

class DataPackage
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    DataPackage();
    // void UpdateAfterSetData();
    void init();
    void getIMUdata(ActiveImuReader reader);
    // data
    // dimension of the robot dim = motorNum + floatingNum


    int generalizedCoordinatesNum;
    int actuatedDofNum;
    // control cycle time
    double control_period;

    int IndexLegStart, IndexLegLength;
    int IndexArmStart, IndexArmLength;
    int IndexWaistStart, IndexWaistLength;
    /**
     * @brief q_a
     *     joint position sense
     * qa = [q_float, q_joint] for floating base robot and mobile robot
     * qa = [q_joint] for fixed robot
     */

    // only pinocchio use
    Eigen::Matrix<double, Eigen::Dynamic,1> generalized_q_actual;
    Eigen::Matrix<double, Eigen::Dynamic,1> generalized_q_desired;
 
    Eigen::Matrix<double, Eigen::Dynamic,1> generalized_q_dot_actual;
    Eigen::Matrix<double, Eigen::Dynamic,1> generalized_q_dot_desired;


    // from sensors 
    Eigen::Matrix<double, Eigen::Dynamic,1> motor_torque;
    Eigen::Matrix<double, Eigen::Dynamic,1> motor_vel;
    Eigen::Matrix<double, Eigen::Dynamic,1> motor_pos;
    Eigen::Matrix<double, Eigen::Dynamic,1> sim_P;
    Eigen::Matrix<double, Eigen::Dynamic,1> sim_D;
    Eigen::Matrix<double, 3,1> imu_lin_acc;     
    Eigen::Quaternion<double> imu_quat;      
    Eigen::Matrix<double, 4,1>  baseQuat;      
    Eigen::Matrix<double, 3,1> imu_rpy;  
    Eigen::Matrix<double, 3,1> imu_zyx;     
    Eigen::Matrix<double, 3,1> imu_angular_vel;      

    //from state estimator
    Eigen::Matrix<double, Eigen::Dynamic,1> contact_force;      
    Eigen::Matrix<double, 3,1> base_lin_vel;     
    Eigen::Matrix<double, 3,1> base_pos;     
    std::vector<Eigen::VectorXd> contact_force_history;
    std::vector<double> speed_forward_history;
    double speed_forward_avg{0};




    Eigen::Matrix<double, Eigen::Dynamic,1> q_desire;    // joint position command

    Eigen::Matrix<double, Eigen::Dynamic,1> q_dot_desire;    // joint velocity command

    Eigen::Matrix<double, Eigen::Dynamic,1> q_dot_dot_desire;      // joint acceleration command

    Eigen::Matrix<double, Eigen::Dynamic,1> torq_desire;    // joint torque command

    Eigen::Matrix<double, Eigen::Dynamic,1> motor_Pos_desire;  
    Eigen::Matrix<double, Eigen::Dynamic,1> motor_upper_body_buffer;  
    Eigen::Matrix<double, Eigen::Dynamic,1> motor_Vel_desire;    
    Eigen::Matrix<double, Eigen::Dynamic,1> motor_Torque_desire;  



    Eigen::Matrix<double, 3,1> desire_imu_zyx;     
    Eigen::Matrix<double, 3,1> desire_imu_angular_vel;  
    Eigen::Matrix<double, 3,1> desire_lin_vel;  
    Eigen::Matrix<double, Eigen::Dynamic,1> desire_motor_pos;
    Eigen::Matrix<double, Eigen::Dynamic,1> desire_motor_vel;


    // states and key variables for Joystick
    double js_vx_desire{0}, js_vy_desire{0};
    double js_OmegaZ_desire{0};
    double js_z_height_desire{0};
    bool js_play;
    int control_mode{0};
    int mimic_mode{0};
    double acc_max;

    // ASE 动作控制
    int ase_profile_id{0};          
    int ase_skill_id{-1};           
    int ase_skill_loop{0};          
    int ase_skill_trigger_count{0}; 

    // Command sent to the LED controller. The protocol uses the raw 16-bit value.
    std::uint16_t led_command{0};
    
    double neck_yaw_xx{0};
    double neck_pitch_yy{0};

    State CurrentState{State::ZERO};
    State NextState{State::ZERO};

    // model 编号: 用于 RL_deploy_lower_cpg 的动作库切换
    int model{0};

    /* 
     * 0:站立零位
     * 1:趴下零位
     */
    ZeroPoseType CurrentZeroState{ZeroPoseType::Standing}; 
    
    uint8_t action_cmd;
    int8_t action_Result;


    bool bms_valid{false};
    float bms_voltage{0.0f};
    float bms_current{0.0f};
    float bms_soc{0.0f};
    uint16_t bms_fault_code_0_1{0};
    uint16_t bms_fault_code_2_3{0};
    uint16_t bms_fault_code_4_5{0};
    uint16_t bms_fault_code_6_7{0};
    uint16_t bms_fault_code_8_9{0};
    uint16_t bms_fault_code_10_11{0};
    uint16_t bms_fault_code_12_13{0};
    int8_t bms_battery_status{0};


    bool If_first_Zero = true;
    
    Eigen::VectorXd error_code ;
    Eigen::VectorXd isOnline ;
        
    Eigen::VectorXd FastStateMechine ;
    Eigen::VectorXd FastMosTemperature ;
    Eigen::VectorXd FastWindingTemperature ;
    Eigen::VectorXd FastBusVoltage ;
    Eigen::VectorXd FastErrorCode ;

};

#endif // DATAPACKAGE_H
