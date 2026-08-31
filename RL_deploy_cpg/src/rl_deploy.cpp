#include "RL_deploy_cpg/include/rl_deploy.h"
// #include "RL_deploy_cpg/include/CPGControl.h"
#include <cmath>  // 包含 sin、cos 等数学函数

//构造函数：初始化RL策略和CPG参数
rl_deploy_cpg::rl_deploy_cpg()
{
    torch::autograd::GradMode::set_enabled(false);
    std::string file_path = "../RL_deploy_cpg/config/rl_params.yaml";
    YAML::Node config = YAML::LoadFile(file_path);
    // load the policy
    policy_path = config["policy_path"].as<std::string>();
    policy = torch::jit::load(policy_path, torch::kCPU);
    std::cout << "valinia policy init complete" << std::endl;


    // ---- 读取控制与观测参数 ----
    decimation = config["decimation"].as<int>();        // 策略推理降采样倍数（每N个控制周期推理一次）
    num_proprio = config["num_proprio"].as<int>();      // 本体感知观测维度数

    clip_obs = config["clip_obs"].as<double>();                    // 观测值裁剪上限
    clip_actions_lower = config["clip_actions_lower"].as<double>(); // 动作输出裁剪下限
    clip_actions_upper = config["clip_actions_upper"].as<double>(); // 动作输出裁剪上限

    num_of_dofs = config["num_of_dofs"].as<int>();      // 机器人自由度（关节）数量
    history_len = config["history_len"].as<int>();      // 历史缓冲区长度
    lin_vel_scale = config["lin_vel_scale"].as<double>();   // 线速度缩放因子
    ang_vel_scale = config["ang_vel_scale"].as<double>();   // 角速度缩放因子
    dof_pos_scale = config["dof_pos_scale"].as<double>();   // 关节位置缩放因子
    dof_vel_scale = config["dof_vel_scale"].as<double>();   // 关节速度缩放因子

    // ---- 初始化Eigen向量（全部置零） ----
    last_actions_eigen = Eigen::VectorXd::Zero(num_of_dofs);           // 上一时刻动作输出
    outputdata_eigen = Eigen::VectorXd::Zero(num_of_dofs);             // 当前策略网络输出
    pos_hist_buf_eigen = Eigen::VectorXd::Zero(history_len * num_of_dofs);   // 关节位置历史缓冲区
    vel_hist_buf_eigen = Eigen::VectorXd::Zero(history_len * num_of_dofs);   // 关节速度历史缓冲区
    action_hist_buf_eigen = Eigen::VectorXd::Zero(history_len * num_of_dofs); // 动作历史缓冲区
    rot_hist_buf_eigen = Eigen::VectorXd::Zero(15);                    // 旋转/重力向量历史（5帧×3维）
    ang_vel_hist_buf_eigen = Eigen::VectorXd::Zero(15);                 // 角速度历史（5帧×3维）

    // ---- 从YAML读取向量并转换为Eigen格式 ----
    // 默认关节位置（站立位姿）
    std::vector<double> default_dof_pos_vec = ReadVectorFromYaml<double>(config["default_dof_pos"]);
    default_dof_pos_eigen = Eigen::Map<Eigen::VectorXd>(default_dof_pos_vec.data(), default_dof_pos_vec.size());
    
    // PD控制的比例增益P
    std::vector<double> rl_p_vec = ReadVectorFromYaml<double>(config["rl_p"]);
    rl_p = Eigen::Map<Eigen::VectorXd>(rl_p_vec.data(), rl_p_vec.size());
    
    // PD控制的微分增益D
    std::vector<double> rl_d_vec = ReadVectorFromYaml<double>(config["rl_d"]);
    rl_d = Eigen::Map<Eigen::VectorXd>(rl_d_vec.data(), rl_d_vec.size());

    // 动作缩放系数（策略输出到关节目标位置的映射）
    std::vector<double> action_scale_vec = ReadVectorFromYaml<double>(config["action_scale"]);
    action_scale = Eigen::Map<Eigen::VectorXd>(action_scale_vec.data(), action_scale_vec.size());

    // 需要移除的无效关节索引列表
    indices_to_remove = ReadVectorFromYaml<int>(config["useless_joint_id"]);

    // ---- 初始化传感器与指令向量 ----
    lin_vel = Eigen::VectorXd::Zero(3);           // 机体线速度（当前未使用，预留）
    imu_angular_vel = Eigen::VectorXd::Zero(3);   // IMU角速度
    commands = Eigen::VectorXd::Zero(3);          // 速度指令 [vx, vy, omega_z]

    base_euler = Eigen::VectorXd::Zero(3);        // 机体欧拉角

    dof_pos = Eigen::VectorXd::Zero(num_of_dofs); // 当前关节位置
    dof_vel = Eigen::VectorXd::Zero(num_of_dofs); // 当前关节速度
    last_actions = Eigen::VectorXd::Zero(num_of_dofs); // 上一时刻动作

    // ---- 初始化Torch张量（用于后续可能的GPU/张量运算） ----
    torque_limits = torch::tensor(ReadVectorFromYaml<double>(config["torque_limits"])).view({1, -1}); // 力矩限制
    default_dof_pos = torch::tensor(ReadVectorFromYaml<double>(config["default_dof_pos"])).view({1, -1}); // 默认位置（Torch版）

    commands_scale = torch::tensor({lin_vel_scale, lin_vel_scale, ang_vel_scale}); // 指令缩放 [vx, vy, yaw]缩放
    output_dof_pos = torch::zeros({1, num_of_dofs}); // 输出关节位置（预留）

    // ---- 计算并初始化观测向量维度 ----
    // 观测维度 = 本体感知维度 + 关节位置历史 + 关节速度历史 + 其他历史...
    num_observations = num_proprio + num_of_dofs * 5 * 2;
    inputdata_eigen = Eigen::VectorXd::Zero(num_observations); // 策略网络输入向量

    // ---- 初始化CPG（中枢模式发生器） ----
    CPGInit(4, 0.02, float(1.0 / 0.8)); // 4肢，步长0.02s，基础频率1.25Hz
    CPGReset(true); // 重置CPG状态到初始相位

    // ---- 初始化仿真/调试用的力矩分解向量 ----
    sim_tor = Eigen::VectorXd::Zero(num_of_dofs);   // 总仿真力矩
    sim_tor_p = Eigen::VectorXd::Zero(num_of_dofs); // P项力矩
    sim_tor_d = Eigen::VectorXd::Zero(num_of_dofs); // D项力矩

    // ---- 打开数据日志文件 ----
    foutData.open("./c2_policy_sim_walk.txt", std::ios::out);
    dataL = Eigen::VectorXd::Zero(600); // 日志数据向量（足够存储所有调试信息）

}

//析构函数
rl_deploy_cpg::~rl_deploy_cpg() {
}

//获取传感器数据
void rl_deploy_cpg::GetDataFromPackage(DataPackage &DataPackage){

    imu_angular_vel = DataPackage.imu_angular_vel;
    gravity_vec_eigen = Euler_ZYXToGravityVec(DataPackage.imu_zyx);

    q_origin = DataPackage.motor_pos;




    dot_q_origin = DataPackage.motor_vel;

    tor_origin = DataPackage.motor_torque;

    js_vx_desire = DataPackage.js_vx_desire;
    js_vy_desire = DataPackage.js_vy_desire;
    js_OmegaZ_desire = DataPackage.js_OmegaZ_desire;

}

//计算结果写回到数据包中
void rl_deploy_cpg::SetDataToPackage(DataPackage &data)
{

    data.torq_desire.setZero() ;

    data.motor_Vel_desire.setZero();


    for(int i =0; i<num_of_dofs; i++)
    {
        double a = outputdata_eigen(i);
        data.motor_Pos_desire(i) = a * action_scale(i) + default_dof_pos_eigen[i];
        sim_tor[i] = rl_p[i] *(data.motor_Pos_desire[i] - q_origin[i]) + rl_d[i]* (0 - dot_q_origin[i]);
        sim_tor_p[i] = rl_p[i] *(data.motor_Pos_desire[i] - q_origin[i]);
        sim_tor_d[i] = rl_d[i]* (0 - dot_q_origin[i]);
    }



    data.motor_Torque_desire.setZero();

    data.sim_P = rl_p*1.0;
    data.sim_D = rl_d*1.0;

    dataL[0] = 0;
    // std::cout<<"action: "<<outputdata_eigen(15)<<std::endl;
    // std::cout<<"vel: "<<dot_q_origin(15)<<std::endl;
    dataL.block(1, 0, 21, 1) = q_origin;
    dataL.block(22, 0, 21, 1) = dot_q_origin;
    dataL.block(43, 0, 21, 1) = outputdata_eigen;
    dataL.block(64, 0, 21, 1) = tor_origin;
    dataL.block(85, 0, 21, 1) = sim_tor;
    dataL.block(106, 0, 21, 1) = sim_tor_p;
    dataL.block(127, 0, 21, 1) = sim_tor_d;

    dataLog(dataL, foutData);

}


//退出函数，清空历史数据
void rl_deploy_cpg::Exit()
{
    // clear the hist buf and the step counter
    pos_hist_buf_eigen.setZero();
    vel_hist_buf_eigen.setZero();

}

//主循环函数，执行一次RL策略计算
void rl_deploy_cpg::Step()
{
    if (step_num == 0){
        step_num = 1;
    }
    else{
        step_num += 1;
    }

    
    if(step_num % decimation == 0)
    {   

        commands(0) = js_vx_desire * lin_vel_scale;
        commands(1) = js_vy_desire * lin_vel_scale;
        commands(2) = js_OmegaZ_desire * ang_vel_scale;

        inputdata_eigen.segment(0, 3) =  imu_angular_vel * ang_vel_scale;     
        inputdata_eigen.segment(3, 3) =  gravity_vec_eigen ;
        inputdata_eigen.segment(6, 3) = commands;
        inputdata_eigen(9) = (abs(commands(0)) < 0.3) ? 0.0 : 1.0;

        inputdata_eigen.segment(10, num_of_dofs) = (q_origin - default_dof_pos_eigen) * dof_pos_scale;
        inputdata_eigen.segment(10 + num_of_dofs, num_of_dofs) = dot_q_origin * dof_vel_scale ;
        inputdata_eigen.segment(10 + 2*num_of_dofs, num_of_dofs) = last_actions_eigen;


        // std::cout << "q_origin: " << q_origin << std::endl;

        inputdata_eigen.segment(10 + 3 * num_of_dofs, 5 * num_of_dofs) = pos_hist_buf_eigen.tail(5 * num_of_dofs);
        inputdata_eigen.segment(10 + 3 * num_of_dofs +  5 * num_of_dofs, 5 * num_of_dofs) = vel_hist_buf_eigen.tail(5 * num_of_dofs);

        
        // std::vector<torch::jit::IValue> inputs;
        // torch::Tensor inputdata_tensor = torch::zeros({num_observations}).toType(torch::kFloat);

        // auto input_accessor = inputdata_tensor.accessor<float, 1>();
        // for (int i = 0; i < num_observations; i++) {
        //     input_accessor[i] = static_cast<float>(inputdata_eigen(i));
        // }
        // inputdata_tensor = inputdata_tensor.to(torch::kCPU);
        // inputdata_tensor = inputdata_tensor.unsqueeze(0);
        // inputs.push_back(inputdata_tensor);

        
        // torch::Tensor output_tensor;

        // output_tensor = policy.forward(inputs).toTensor();

        // if (output_tensor.dim() > 1 && output_tensor.size(0) == 1){
        //     output_tensor = output_tensor.squeeze(0);
        // }

        // auto accessor = output_tensor.accessor<float, 1>();
        // for (int i = 0; i < num_of_dofs; i++) {
        //     outputdata_eigen(i) = static_cast<double>(accessor[i]);
        // }
        


        // last_actions_eigen = outputdata_eigen;



        //==================================================
        // Clip Observation
        //==================================================

        inputdata_eigen =
            inputdata_eigen
                .cwiseMax(-clip_obs)
                .cwiseMin( clip_obs );

        //==================================================
        // Eigen -> Tensor
        //==================================================

        std::vector<torch::jit::IValue> inputs;

        torch::Tensor inputdata_tensor =
            torch::zeros({num_observations},
                        torch::TensorOptions().dtype(torch::kFloat));

        auto input_accessor =
            inputdata_tensor.accessor<float, 1>();

        for (int i = 0; i < num_observations; i++)
        {
            input_accessor[i] =
                static_cast<float>(inputdata_eigen(i));
        }

        inputdata_tensor = inputdata_tensor.unsqueeze(0);

        inputs.push_back(inputdata_tensor);

        //==================================================
        // Policy Forward
        //==================================================

        torch::Tensor output_tensor =
            policy.forward(inputs).toTensor();

        if (output_tensor.dim() > 1 &&
            output_tensor.size(0) == 1)
        {
            output_tensor = output_tensor.squeeze(0);
        }

        //==================================================
        // Tensor -> Eigen
        //==================================================

        auto output_accessor =
            output_tensor.accessor<float, 1>();

        for (int i = 0; i < num_of_dofs; i++)
        {
            outputdata_eigen(i) =
                static_cast<double>(output_accessor[i]);
        }

        //==================================================
        // Clip Action
        //==================================================

        outputdata_eigen =
            outputdata_eigen
                .cwiseMax(clip_actions_lower)
                .cwiseMin(clip_actions_upper);

        last_actions_eigen = outputdata_eigen;



        // pos_hist_buf_eigen.head(pos_hist_buf_eigen.size() - num_of_dofs) = pos_hist_buf_eigen.tail(pos_hist_buf_eigen.size() - num_of_dofs);
        // pos_hist_buf_eigen.tail(num_of_dofs) = (q_origin - default_dof_pos_eigen) * dof_pos_scale;
        
        // vel_hist_buf_eigen.head(vel_hist_buf_eigen.size() - num_of_dofs) = vel_hist_buf_eigen.tail(vel_hist_buf_eigen.size() - num_of_dofs);
        // vel_hist_buf_eigen.tail(num_of_dofs) = dot_q_origin * dof_vel_scale;


        pos_hist_buf_eigen.head(pos_hist_buf_eigen.size() - num_of_dofs) = pos_hist_buf_eigen.tail(pos_hist_buf_eigen.size() - num_of_dofs);
        pos_hist_buf_eigen.tail(num_of_dofs) = q_origin * dof_pos_scale;
        
        vel_hist_buf_eigen.head(vel_hist_buf_eigen.size() - num_of_dofs) = vel_hist_buf_eigen.tail(vel_hist_buf_eigen.size() - num_of_dofs);
        vel_hist_buf_eigen.tail(num_of_dofs) = dot_q_origin * dof_vel_scale;


        action_hist_buf_eigen.head(action_hist_buf_eigen.size() - num_of_dofs) = action_hist_buf_eigen.tail(action_hist_buf_eigen.size() - num_of_dofs);
        action_hist_buf_eigen.tail(num_of_dofs) = last_actions_eigen;



        rot_hist_buf_eigen.head(12) = rot_hist_buf_eigen.tail(12);
        rot_hist_buf_eigen.tail(3) = gravity_vec_eigen;

        ang_vel_hist_buf_eigen.head(12) = ang_vel_hist_buf_eigen.tail(12);
        ang_vel_hist_buf_eigen.tail(3) = imu_angular_vel * ang_vel_scale;

        episode_length ++;

    }
}

//CPG初始化函数，设置Hopf参数
void rl_deploy_cpg::CPGInit(int num_feet, float dt, float fre) {
    step_dt = dt;
    gait_cy = fre;

    cycle_r = Eigen::Vector4d::Ones();
    px = Eigen::Vector4d::Ones();
    py = Eigen::Vector4d::Zero();

    phase_offset = Eigen::VectorXd::Zero(num_feet);
    phase_offset_target = Eigen::VectorXd::Zero(num_feet);
    phase_offset_change_rate = 0.01 * (step_dt / 0.0005);

    CPGSetP(phase_offset);

    phase_offset_stand << 0.0, (0.0) * M_PI, 0.0, (0.0) * M_PI;
    phase_offset_walk << 0.0, (1.0) * M_PI, (-0.15) * M_PI, (-1.15) * M_PI;
    phase_offset_run << 0.0, (1.0) * M_PI, (-0.16) * M_PI, (-1.16) * M_PI;

    coupling_change_rate = Eigen::Vector4d::Ones(num_feet) * (2*2);
    coupling_change_rate_increase = 1.0 * (2*2);
    coupling_change_rate_decrease = 1.0 * (0.3+0.7);

    amplitude_change_rate = Eigen::Vector4d::Ones(num_feet);
    amplitude_change_rate_increase = std::pow(1.0 * (1+0.05), (step_dt / 0.0005));
    amplitude_change_rate_decrease = std::pow(1.0 * (1-0.005), (step_dt / 0.0005));

    CPGSetC(Eigen::VectorXd::Ones(4) * 0.5);

}
//计算Hopf震荡器的增益参数gamma（角速度调制项）
//gamma决定极限环上的角速度，与触地比例相关
Eigen::VectorXd rl_deploy_cpg::CPGGammaCal() {
    Eigen::VectorXd term1 = Eigen::VectorXd::Zero(num_feet);
    Eigen::VectorXd term2 = Eigen::VectorXd::Zero(num_feet);
    for (int i = 0; i < num_feet; i++) {
        term1(i) = M_PI / ((1 - contact_ratio(i)) * (std::exp((-1) * coff_b * py(i)) + 1));
        term2(i) = M_PI / ((0 + contact_ratio(i)) * (std::exp(coff_b * py(i)) + 1));

    }
    return term1 + term2;
}
// 计算CPG耦合项：使四肢的振荡器保持期望相位关系
// 基于Kuramoto-like耦合，使各振荡器趋向目标相位差
Eigen::MatrixXd rl_deploy_cpg::CPGCoupling() {
    Eigen::VectorXd coupling_x = Eigen::VectorXd::Zero(num_feet);
    Eigen::VectorXd coupling_y = Eigen::VectorXd::Zero(num_feet);
    double coupling_x_ij = 0.0;
    double coupling_y_ij = 0.0;
    double r_i = 0.0;
    double r_j = 0.0;
    
    for (int i = 0; i < num_feet; i++) {
        coupling_x(i) = 0.0;
        coupling_y(i) = 0.0;
        for (int j = 0; j < num_feet; j++) {
            if (i == j) {
                continue;
            }
            coupling_x_ij = std::cos(phase_offset(i) - phase_offset(j)) * px(j) 
                         - std::sin(phase_offset(i) - phase_offset(j)) * py(j);
            coupling_y_ij = std::sin(phase_offset(i) - phase_offset(j)) * px(j) 
                         + std::cos(phase_offset(i) - phase_offset(j)) * py(j);
            
            r_i = std::sqrt(std::pow(px(i), 2) + std::pow(py(i), 2));
            r_j = std::sqrt(std::pow(px(j), 2) + std::pow(py(j), 2));
            
            coupling_x_ij = coupling_x_ij / r_j * r_i;
            coupling_y_ij = coupling_y_ij / r_j * r_i;
            
            coupling_x(i) += coupling_x_ij;
            coupling_y(i) += coupling_y_ij;
        }
    }
    
    Eigen::MatrixXd result(num_feet, 2);
    result.col(0) = coupling_x;
    result.col(1) = coupling_y;
    return result;
}
//相位更新函数：根据目标相位差和当前相位差调整各振荡器的相位，向目标相位偏移
void rl_deploy_cpg::CPGPhaseChange() {
    phase_offset += phase_offset_change_rate * (phase_offset_target - phase_offset);
}
//Hopf震荡器更新函数：离散积分
void rl_deploy_cpg::CPGHopfOscillator() {
    Eigen::Vector4d r;

    for (int i = 0; i < num_feet; i++) {
        r(i) = std::sqrt(std::pow(px(i), 2) + std::pow(py(i), 2));
    }
    
    Eigen::Vector4d gamma = CPGGammaCal();
    double alpha = gait_cy * rate;
    Eigen::Vector4d omega = gait_cy * gamma;
    
    CPGPhaseChange();
    Eigen::MatrixXd coupling_xy = CPGCoupling();
    Eigen::Vector4d coupling_x = coupling_xy.col(0);
    Eigen::Vector4d coupling_y = coupling_xy.col(1);

    Eigen::Vector4d dx;
    Eigen::Vector4d dy;
    
    for (int i = 0; i < num_feet; i++) {
        dx(i) = (alpha * (std::pow(cycle_r(i), 2) - std::pow(r(i), 2)) * px(i) - omega(i) * py(i)) 
              + coupling_change_rate(i) * coupling_x(i);
        dy(i) = (alpha * (std::pow(cycle_r(i), 2) - std::pow(r(i), 2)) * py(i) + omega(i) * px(i)) 
              + coupling_change_rate(i) * coupling_y(i);

        px(i) = px(i) + dx(i) * step_dt;
        py(i) = py(i) + dy(i) * step_dt;

        if (r(i) > cycle_r_min) {
            px(i) = px(i) * amplitude_change_rate(i);
            py(i) = py(i) * amplitude_change_rate(i);
        }

        r(i) = std::sqrt(std::pow(px(i), 2) + std::pow(py(i), 2));
        if (r(i) > cycle_r_max) {
            px(i) = px(i) / r(i);
            py(i) = py(i) / r(i);
        }
    }
    
}

// CPG状态重置：将振荡器重置到初始相位
void rl_deploy_cpg::CPGReset(bool flag) {
    if(flag) {
        px << std::cos(0.5*M_PI), std::cos(0.5*M_PI), std::cos(0.5*M_PI), std::cos(0.5*M_PI);
        py << std::sin(0.5*M_PI), std::sin(0.5*M_PI), std::sin(0.5*M_PI), std::sin(0.5*M_PI);
    } else {
        px << std::cos(0.5*M_PI), std::cos(0.5*M_PI), std::cos(0.5*M_PI), std::cos(0.5*M_PI);
        py << std::sin(0.5*M_PI), std::sin(0.5*M_PI), std::sin(0.5*M_PI), std::sin(0.5*M_PI);          
    }
    CPGSetP(phase_offset_target);
}

void rl_deploy_cpg::CPGSetCy(Eigen::VectorXd Cycle_r) {
    cycle_r = Cycle_r;// 设置目标极限环半径
}

void rl_deploy_cpg::CPGSetCou(Eigen::VectorXd Coupling_change_rate) {
    coupling_change_rate = Coupling_change_rate;// 设置耦合变化率
}

void rl_deploy_cpg::CPGSetA(Eigen::VectorXd Amplitude_change_rate) {
    amplitude_change_rate = Amplitude_change_rate;// 设置振幅变化率
}

void rl_deploy_cpg::CPGSetP(Eigen::VectorXd Phase_offset) {
    phase_offset_target = Phase_offset;// 设置目标相位偏移
}

void rl_deploy_cpg::CPGSetC(Eigen::VectorXd Contact_ratio) {
    contact_ratio = Contact_ratio;// 设置触地比例
}

void rl_deploy_cpg::CPGSetncy(double fre) {
    gait_cy = fre;// 设置步态频率
}

//设置站立模式参数
void rl_deploy_cpg::CPGSetSn() {
    CPGSetncy((1));
    CPGSetCy(Eigen::Vector4d::Ones(4) * cycle_r_min);
    CPGSetCou(Eigen::Vector4d::Ones(4) * coupling_change_rate_decrease);
    CPGSetA(Eigen::Vector4d::Ones(4) * amplitude_change_rate_decrease);
    CPGSetP(phase_offset_stand);
    CPGSetC(Eigen::Vector4d::Ones(4) * 0.5);

}
// 设置行走（Walk）模式参数 WP_1 ~ WP_8
// 不同WP对应不同速度和触地比例，实现速度-步态映射
void rl_deploy_cpg::CPGSetWP_1() {
    CPGSetncy((1.4286));
    CPGSetCy(Eigen::VectorXd::Ones(4) * cycle_r_max);
    CPGSetCou(Eigen::VectorXd::Ones(4) * coupling_change_rate_increase);
    CPGSetA(Eigen::VectorXd::Ones(4) * amplitude_change_rate_increase);
    CPGSetP(phase_offset_walk);
    CPGSetC(Eigen::VectorXd::Ones(4) * 0.57);

    
}
void rl_deploy_cpg::CPGSetWP_2() {
    CPGSetncy((1.4286));
    CPGSetCy(Eigen::VectorXd::Ones(4) * cycle_r_max);
    CPGSetCou(Eigen::VectorXd::Ones(4) * coupling_change_rate_increase);
    CPGSetA(Eigen::VectorXd::Ones(4) * amplitude_change_rate_increase);
    CPGSetP(phase_offset_walk);
    CPGSetC(Eigen::VectorXd::Ones(4) * 0.54);

    
}
void rl_deploy_cpg::CPGSetWP_3() {
    CPGSetncy((1.4286));
    CPGSetCy(Eigen::VectorXd::Ones(4) * cycle_r_max);
    CPGSetCou(Eigen::VectorXd::Ones(4) * coupling_change_rate_increase);
    CPGSetA(Eigen::VectorXd::Ones(4) * amplitude_change_rate_increase);
    CPGSetP(phase_offset_walk);
    CPGSetC(Eigen::VectorXd::Ones(4) * 0.51);

    
}
void rl_deploy_cpg::CPGSetWP_4() {
    CPGSetncy((1.4286));
    CPGSetCy(Eigen::VectorXd::Ones(4) * cycle_r_max);
    CPGSetCou(Eigen::VectorXd::Ones(4) * coupling_change_rate_increase);
    CPGSetA(Eigen::VectorXd::Ones(4) * amplitude_change_rate_increase);
    CPGSetP(phase_offset_walk);
    CPGSetC(Eigen::VectorXd::Ones(4) * 0.50);

    
}
void rl_deploy_cpg::CPGSetWP_5() {
    CPGSetncy((1.667));
    CPGSetCy(Eigen::VectorXd::Ones(4) * cycle_r_max);
    CPGSetCou(Eigen::VectorXd::Ones(4) * coupling_change_rate_increase);
    CPGSetA(Eigen::VectorXd::Ones(4) * amplitude_change_rate_increase);
    CPGSetP(phase_offset_walk);
    CPGSetC(Eigen::VectorXd::Ones(4) * 0.35);

    
}
void rl_deploy_cpg::CPGSetWP_6() {
    CPGSetncy((1.667));
    CPGSetCy(Eigen::VectorXd::Ones(4) * cycle_r_max);
    CPGSetCou(Eigen::VectorXd::Ones(4) * coupling_change_rate_increase);
    CPGSetA(Eigen::VectorXd::Ones(4) * amplitude_change_rate_increase);
    CPGSetP(phase_offset_walk);
    CPGSetC(Eigen::VectorXd::Ones(4) * 0.3);

    
}
void rl_deploy_cpg::CPGSetWP_7() {
    CPGSetncy((1.8182));
    CPGSetCy(Eigen::VectorXd::Ones(4) * cycle_r_max);
    CPGSetCou(Eigen::VectorXd::Ones(4) * coupling_change_rate_increase);
    CPGSetA(Eigen::VectorXd::Ones(4) * amplitude_change_rate_increase);
    CPGSetP(phase_offset_walk);
    CPGSetC(Eigen::VectorXd::Ones(4) * 0.3);

    
}
void rl_deploy_cpg::CPGSetWP_8() {
    CPGSetncy((1.8182));
    CPGSetCy(Eigen::VectorXd::Ones(4) * cycle_r_max);
    CPGSetCou(Eigen::VectorXd::Ones(4) * coupling_change_rate_increase);
    CPGSetA(Eigen::VectorXd::Ones(4) * amplitude_change_rate_increase);
    CPGSetP(phase_offset_walk);
    CPGSetC(Eigen::VectorXd::Ones(4) * 0.27);

    
}

// 获取CPG归一化X输出
Eigen::VectorXd rl_deploy_cpg::CPGGetXNorm() {
    Eigen::Vector4d r(num_feet);
    Eigen::Vector4d x_norm(num_feet);
    for (int i = 0; i < num_feet; i++) {
        r(i) = std::sqrt(std::pow(px(i), 2) + std::pow(py(i), 2));
        x_norm(i) = (r(i) > cycle_r_min) ? px(i) : 0.0;
    }
    return x_norm;
}

// 获取CPG归一化Y输出
Eigen::VectorXd rl_deploy_cpg::CPGGetYNorm() {
    Eigen::Vector4d r(num_feet);
    Eigen::Vector4d y_norm(num_feet);
    for (int i = 0; i < num_feet; i++) {
        r(i) = std::sqrt(std::pow(px(i), 2) + std::pow(py(i), 2));
        y_norm(i) = (r(i) > cycle_r_min) ? py(i) : 0.0;
    }
    return y_norm;
}

// 根据速度指令范数切换步态模式
// 速度分区：flag_1 < flag_2 < ... < flag_9
void rl_deploy_cpg::UpdateGaitGeneratorPattern(Eigen::VectorXd Commands) {

    double vno = Commands.norm();
    if (vno < flag_1)
    {

        CPGSetSn();
    }

    if (vno >= flag_1 and vno < flag_2) 
    {

        CPGSetWP_1();
    }
    else if (vno >= flag_2 and vno < flag_3) 
    {

        CPGSetWP_2();
    }
    else if (vno >= flag_3 and vno < flag_4) 
    {

        CPGSetWP_3();
    }
    else if (vno >= flag_4 and vno < flag_5)
    {
        CPGSetWP_4();
    }
    else if (vno >= flag_5 and vno < flag_6)
    {
        CPGSetWP_5();
    }
    else if (vno >= flag_6 and vno < flag_7)
    {
        CPGSetWP_6();
    }
    else if (vno >= flag_7 and vno < flag_8)
    {
        CPGSetWP_7();
    }
    else if (vno >= flag_8 and vno < flag_9)
    {
        CPGSetWP_8();
    }
}

void rl_deploy_cpg::CommandRefinement() {

    Eigen::VectorXd Commands_buf = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd last_Commands_buf;

}

// 用于过滤无效关节数据
Eigen::VectorXd rl_deploy_cpg::remove_indices(const Eigen::VectorXd& input, const std::vector<int>& indices_to_remove) 
{
    int original_size = input.size();

    // 构建保留的索引列表
    std::vector<int> keep_indices;
    for (int i = 0; i < original_size; ++i) {
        if (std::find(indices_to_remove.begin(), indices_to_remove.end(), i) == indices_to_remove.end()) {
            keep_indices.push_back(i);
        }
    }
    // 构造新向量
    Eigen::VectorXd output(keep_indices.size());
    for (size_t i = 0; i < keep_indices.size(); ++i) {
        output[i] = input[keep_indices[i]];
    }

    return output;
}

// ZYX欧拉角 -> 重力向量（机体坐标系）
Eigen::Vector3d rl_deploy_cpg::Euler_ZYXToGravityVec(Eigen::Vector3d euler_a) {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R = Eigen::AngleAxisd(euler_a[0], Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        Eigen::AngleAxisd(euler_a[1], Eigen::Vector3d::UnitY()).toRotationMatrix() *
        Eigen::AngleAxisd(euler_a[2], Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Vector3d grav_vec = -R.transpose().col(2);
    return grav_vec;
}


void rl_deploy_cpg::assign_with_skipped_zero(
    Eigen::VectorXd& target,
    const Eigen::VectorXd& source,
    const std::vector<int>& indices_to_skip)
{
    int target_size = target.size();
    int source_size = source.size();

    std::unordered_set<int> skip_set(indices_to_skip.begin(), indices_to_skip.end());

    int source_idx = 0;

    for (int i = 0; i < target_size; ++i) {
        if (skip_set.count(i)) {
            target[i] = 0.0;  // 跳过位置赋0
        } else {
            if (source_idx < source_size) {
                target[i] = source[source_idx++];
            } else {
                target[i] = 0.0;  // 如果 source 不够长，剩余位置补0（可选）
            }
        }
    }
    // 可选：保证 source 完全用完
    assert(source_idx == source_size && "Source vector is longer than available non-skipped slots");
}


// 任务观测日志初始化：创建CSV文件并写入表头
void rl_deploy_cpg::InitTaskObsLogging(const std::string& log_file_path) {
    try {
        // Create logs directory if it doesn't exist
        std::string dir_path = "../RL_deploy/logs/";
        system(("mkdir -p " + dir_path).c_str());

        task_obs_log_file.open(log_file_path, std::ios::out | std::ios::trunc);
        if (!task_obs_log_file.is_open()) {
            std::cerr << "Failed to open task_obs log file: " << log_file_path << std::endl;
            enable_task_obs_logging = false;
            return;
        }

        // Write CSV header
        task_obs_log_file << "step_num,frame_count,control_mode";
        for (int i = 0; i < num_observations; ++i) {
            task_obs_log_file << ",proprio_obs_" << i;
        }
        task_obs_log_file << std::endl;

        std::cout << "Task observation logging initialized: " << log_file_path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error initializing task_obs logging: " << e.what() << std::endl;
        enable_task_obs_logging = false;
    }
}

// 记录任务观测到日志文件
void rl_deploy_cpg::LogTaskObs(const torch::Tensor& task_obs_tensor) {
    if (!enable_task_obs_logging || !task_obs_log_file.is_open()) {
        return;
    }

    try {
        // Convert tensor to CPU and double precision for logging
        torch::Tensor task_obs_cpu = task_obs_tensor.to(torch::kCPU).to(torch::kDouble);

        // Write step info
        // task_obs_log_file << step_num << "," << frame_count << "," << control_mode;

        // Write task_obs values
        auto accessor = task_obs_cpu.accessor<double, 2>();
        for (int i = 0; i < num_observations; ++i) {
            task_obs_log_file << " " << std::fixed << std::setprecision(6) << accessor[0][i];
        }
        task_obs_log_file << std::endl;

        // Flush every 100 steps to ensure data is written
        if (step_num % 100 == 0) {
            task_obs_log_file.flush();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error logging task_obs: " << e.what() << std::endl;
    }
}

// 关闭任务观测日志文件
void rl_deploy_cpg::CloseTaskObsLogging() {
    if (task_obs_log_file.is_open()) {
        task_obs_log_file.close();
        std::cout << "Task observation logging closed." << std::endl;
    }
}

// 从CSV文件加载运动库（Motion Library）
Eigen::MatrixXd rl_deploy_cpg::LoadMotionLib(const std::string& filename, bool skipHeader) {
    std::ifstream file(filename);
    std::string line;
    std::vector<std::vector<double>> data;

    // read w/o process
    if (skipHeader) {
        std::getline(file, line);
    }

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        std::vector<double> row;

        while (std::getline(ss, value, ',')) {
            try {
                row.push_back(std::stod(value));
            } catch (...) {
                row.push_back(0.0);  // 可以自定义处理方式
            }
        }

        if (!row.empty())
            data.push_back(row);
    }

    if (data.empty()) {
        throw std::runtime_error("No data found in file: " + filename);
    }

    int rows = data.size();
    int cols = data[0].size();
    num_motion_frames = rows;
    Eigen::MatrixXd mat(rows, cols);

    for (int i = 0; i < rows; ++i)
        mat.row(i) = Eigen::VectorXd::Map(&data[i][0], cols);
    std::cout << "motion lib loaded !!!" << std::endl;

    return mat;
}

// 通用数据日志：将Eigen向量写入文本文件（空格分隔）
bool rl_deploy_cpg::dataLog(Eigen::VectorXd &v, std::ofstream &f) {
    for (int i = 0; i < v.size(); i++) {
        f << v[i] << " ";
    }
    f << std::endl;
    return true;
}