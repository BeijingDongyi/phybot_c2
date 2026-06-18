#include "RL_deploy_cpg/include/rl_deploy.h"

#include <cmath>

rl_deploy_cpg::rl_deploy_cpg()
{
    torch::autograd::GradMode::set_enabled(false);

    YAML::Node config = YAML::LoadFile("../RL_deploy_cpg/config/rl_params.yaml");
    policy_path = config["policy_path"].as<std::string>();
    policy = torch::jit::load(policy_path, torch::kCPU);

    decimation = config["decimation"].as<int>();
    num_proprio = config["num_proprio"].as<int>();
    num_of_dofs = config["num_of_dofs"].as<int>();
    history_len = config["history_len"].as<int>();
    lin_vel_scale = config["lin_vel_scale"].as<double>();
    ang_vel_scale = config["ang_vel_scale"].as<double>();
    dof_pos_scale = config["dof_pos_scale"].as<double>();
    dof_vel_scale = config["dof_vel_scale"].as<double>();
    num_observations = num_proprio + num_of_dofs * 5 * 2;

    std::vector<double> default_dof_pos_vec = ReadVectorFromYaml<double>(config["default_dof_pos"]);
    default_dof_pos_eigen = Eigen::Map<Eigen::VectorXd>(default_dof_pos_vec.data(), default_dof_pos_vec.size());

    std::vector<double> rl_p_vec = ReadVectorFromYaml<double>(config["rl_p"]);
    rl_p = Eigen::Map<Eigen::VectorXd>(rl_p_vec.data(), rl_p_vec.size());

    std::vector<double> rl_d_vec = ReadVectorFromYaml<double>(config["rl_d"]);
    rl_d = Eigen::Map<Eigen::VectorXd>(rl_d_vec.data(), rl_d_vec.size());

    std::vector<double> action_scale_vec = ReadVectorFromYaml<double>(config["action_scale"]);
    action_scale = Eigen::Map<Eigen::VectorXd>(action_scale_vec.data(), action_scale_vec.size());

    last_actions_eigen = Eigen::VectorXd::Zero(num_of_dofs);
    inputdata_eigen = Eigen::VectorXd::Zero(num_observations);
    outputdata_eigen = Eigen::VectorXd::Zero(num_of_dofs);
    pos_hist_buf_eigen = Eigen::VectorXd::Zero(history_len * num_of_dofs);
    vel_hist_buf_eigen = Eigen::VectorXd::Zero(history_len * num_of_dofs);
    q_origin = Eigen::VectorXd::Zero(num_of_dofs);
    dot_q_origin = Eigen::VectorXd::Zero(num_of_dofs);
    imu_angular_vel = Eigen::VectorXd::Zero(3);
    commands = Eigen::VectorXd::Zero(3);
}

rl_deploy_cpg::~rl_deploy_cpg()
{
}

void rl_deploy_cpg::GetDataFromPackage(DataPackage &data)
{
    imu_angular_vel = data.imu_angular_vel;
    gravity_vec_eigen = Euler_ZYXToGravityVec(data.imu_zyx);
    q_origin = data.motor_pos;
    dot_q_origin = data.motor_vel;
    js_vx_desire = data.js_vx_desire;
    js_vy_desire = data.js_vy_desire;
    js_OmegaZ_desire = data.js_OmegaZ_desire;
}

void rl_deploy_cpg::SetDataToPackage(DataPackage &data)
{
    data.torq_desire.setZero();
    data.motor_Vel_desire.setZero();

    for (int i = 0; i < num_of_dofs; i++)
    {
        data.motor_Pos_desire(i) = outputdata_eigen(i) * action_scale(i) + default_dof_pos_eigen(i);
    }

    data.motor_Torque_desire.setZero();
    data.sim_P = rl_p;
    data.sim_D = rl_d;
}

void rl_deploy_cpg::Exit()
{
    step_num = 0;
    last_actions_eigen.setZero();
    outputdata_eigen.setZero();
    pos_hist_buf_eigen.setZero();
    vel_hist_buf_eigen.setZero();
}

void rl_deploy_cpg::Step()
{
    step_num++;
    if (step_num % decimation != 0)
    {
        return;
    }

    commands(0) = js_vx_desire * lin_vel_scale;
    commands(1) = js_vy_desire * lin_vel_scale;
    commands(2) = js_OmegaZ_desire * ang_vel_scale;

    inputdata_eigen.segment(0, 3) = imu_angular_vel * ang_vel_scale;
    inputdata_eigen.segment(3, 3) = gravity_vec_eigen;
    inputdata_eigen.segment(6, 3) = commands;
    inputdata_eigen(9) = (std::abs(commands(0)) < 0.3) ? 0.0 : 1.0;
    inputdata_eigen.segment(10, num_of_dofs) = (q_origin - default_dof_pos_eigen) * dof_pos_scale;
    inputdata_eigen.segment(10 + num_of_dofs, num_of_dofs) = dot_q_origin * dof_vel_scale;
    inputdata_eigen.segment(10 + 2 * num_of_dofs, num_of_dofs) = last_actions_eigen;
    inputdata_eigen.segment(10 + 3 * num_of_dofs, 5 * num_of_dofs) = pos_hist_buf_eigen.tail(5 * num_of_dofs);
    inputdata_eigen.segment(10 + 8 * num_of_dofs, 5 * num_of_dofs) = vel_hist_buf_eigen.tail(5 * num_of_dofs);

    std::vector<torch::jit::IValue> inputs;
    torch::Tensor inputdata_tensor = torch::zeros({num_observations}).toType(torch::kFloat);
    auto input_accessor = inputdata_tensor.accessor<float, 1>();
    for (int i = 0; i < num_observations; i++)
    {
        input_accessor[i] = static_cast<float>(inputdata_eigen(i));
    }

    inputs.push_back(inputdata_tensor.unsqueeze(0).to(torch::kCPU));
    torch::Tensor output_tensor = policy.forward(inputs).toTensor();
    if (output_tensor.dim() > 1 && output_tensor.size(0) == 1)
    {
        output_tensor = output_tensor.squeeze(0);
    }

    auto output_accessor = output_tensor.accessor<float, 1>();
    for (int i = 0; i < num_of_dofs; i++)
    {
        outputdata_eigen(i) = static_cast<double>(output_accessor[i]);
    }

    last_actions_eigen = outputdata_eigen;

    pos_hist_buf_eigen.head(pos_hist_buf_eigen.size() - num_of_dofs) = pos_hist_buf_eigen.tail(pos_hist_buf_eigen.size() - num_of_dofs);
    pos_hist_buf_eigen.tail(num_of_dofs) = q_origin * dof_pos_scale;

    vel_hist_buf_eigen.head(vel_hist_buf_eigen.size() - num_of_dofs) = vel_hist_buf_eigen.tail(vel_hist_buf_eigen.size() - num_of_dofs);
    vel_hist_buf_eigen.tail(num_of_dofs) = dot_q_origin * dof_vel_scale;
}

Eigen::Vector3d rl_deploy_cpg::Euler_ZYXToGravityVec(const Eigen::Vector3d &euler_a) const
{
    Eigen::Matrix3d R = Eigen::AngleAxisd(euler_a[0], Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                        Eigen::AngleAxisd(euler_a[1], Eigen::Vector3d::UnitY()).toRotationMatrix() *
                        Eigen::AngleAxisd(euler_a[2], Eigen::Vector3d::UnitX()).toRotationMatrix();
    return -R.transpose().col(2);
}
