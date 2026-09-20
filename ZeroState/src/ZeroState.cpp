#include "ZeroState/include/ZeroState.h"
#include "StateMachine/include/fsmlist.h"



ZeroState::ZeroState() : pose_type_(ZeroPoseType::Standing)
{
    std::string file_path = "../ZeroState/config/ZERO.yaml";
    YAML::Node task_config = YAML::LoadFile(file_path);
    
    std::cout<<"ZeroState path : "<<file_path<<std::endl;
    const std::string state_name = task_config["ZeroState"].as<std::string>();
    const char* pose_key = nullptr;

    /* 判断零位状态 */
    if (state_name == "Stand") {                /* 站立零位 */
        pose_type_ = ZeroPoseType::Standing;
        pose_key = "standing_joint_position";
    } else if (state_name == "Crane_down") {    /* 趴下零位 */
        pose_type_ = ZeroPoseType::CraneDown;
        pose_key = "crane_down_joint_position";
    } else {                                    /* 无效零位 */
        throw std::runtime_error(
            "Invalid ZeroState: " + state_name +
            ". Expected Stand or Crane_down");
    }
    
    std::cout <<"ZeroState pose_key : "<< pose_key <<std::endl;
    std::cout <<"ZeroState pose_type_ : "<< static_cast<int>(pose_type_) <<std::endl;

    auto data = task_config[pose_key];

    if (!data || !data.IsSequence()) {
        throw std::runtime_error(std::string("Missing or invalid pose: ") + pose_key);
    }


    int N = data.size();  // 期望向量长度
    zero_pose.resize(N);
    for (int i = 0; i < N; ++i) {
        zero_pose(i) = data[i].as<double>();
    }


    data = task_config["zero_p"];
    N = data.size();  // 期望向量长度
    zero_p.resize(N);

    for (int i = 0; i < N; ++i) {
        zero_p(i) = data[i].as<double>();
    }

    data = task_config["zero_d"];
    N = data.size();  // 期望向量长度
    zero_d.resize(N);

    for (int i = 0; i < N; ++i) {
        zero_d(i) = data[i].as<double>();
    }

    zero_totalTime = task_config["zero_totalTime"].as<double>();
}


void ZeroState::GetDataFromPackage(DataPackage &data)
{
    If_first_Zero = data.If_first_Zero;
    dt_Zero = data.control_period;
    motor_pos = data.motor_pos;
    actuatedDofNum =data.actuatedDofNum;

}


void ZeroState::SetDataToPackage(DataPackage &data)
{
    if(If_first_Zero){
        If_first_Zero = false;
        t_Zero = 0;
        init_pose = motor_pos;
    }

    t_Zero += dt_Zero;

    Eigen::VectorXd Zero_vector = Eigen::VectorXd::Zero(actuatedDofNum);
    Eigen::VectorXd JointPos_desired(actuatedDofNum);
    Eigen::VectorXd JointVel_desired(actuatedDofNum);

    ThirdpolyVector(init_pose, Zero_vector, zero_pose, Zero_vector, zero_totalTime, t_Zero, JointPos_desired, JointVel_desired);

    data.If_first_Zero = If_first_Zero;                        
    data.motor_Torque_desire.setZero();

    data.motor_Pos_desire = JointPos_desired;
    data.motor_Vel_desire = JointVel_desired;

    data.sim_P = zero_p;
    data.sim_D = zero_d;

    data.generalized_q_desired.tail(actuatedDofNum) = JointPos_desired;
    data.generalized_q_dot_desired.tail(actuatedDofNum) = JointVel_desired;

    /* 设置控制消息回报：当前正在归零 */
    data.action_cmd = static_cast<uint8_t>(State::ZERO);
    data.action_Result = static_cast<int8_t>(ControlMsg_Rep::Runing);

    data.CurrentZeroState = pose_type_;

}

void ZeroState::ThirdpolyVector(const Eigen::VectorXd& p0, const Eigen::VectorXd& p0_dot,
    const Eigen::VectorXd& p1, const Eigen::VectorXd& p1_dot,
    double totalTime, double currentTime,
    Eigen::VectorXd& pd, Eigen::VectorXd& pd_dot) {
    // 分量数量检查
    assert(p0.size() == p0_dot.size());
    assert(p1.size() == p1_dot.size());
    assert(p0.size() == p1.size());

    int dim = p0.size();
    pd.resize(dim);
    pd_dot.resize(dim);

    if (currentTime < 0.0) {
        pd = p0;
        pd_dot.setZero();
    } else if (currentTime <= totalTime) {
        for (int i = 0; i < dim; ++i) {
            double a0 = p0[i];
            double a1 = p0_dot[i];
            double m = p1[i] - p0[i] - p0_dot[i] * totalTime;
            double n = p1_dot[i] - p0_dot[i];
            double a2 = 3.0 * m / (totalTime * totalTime) - n / totalTime;
            double a3 = -2.0 * m / (totalTime * totalTime * totalTime) + n / (totalTime * totalTime);
            pd[i] = a3 * currentTime * currentTime * currentTime +
            a2 * currentTime * currentTime +
            a1 * currentTime + a0;
            pd_dot[i] = 3.0 * a3 * currentTime * currentTime +
                2.0 * a2 * currentTime + a1;
    }
    } else {
        pd = p1;
        pd_dot.setZero();
    }
}


