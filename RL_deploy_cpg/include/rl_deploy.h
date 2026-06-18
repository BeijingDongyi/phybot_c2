#pragma once

#include "DataPackage/include/DataPackage.h"
#include <torch/script.h>

#include <string>
#include <vector>

class rl_deploy_cpg
{
public:
    rl_deploy_cpg();
    ~rl_deploy_cpg();

    void GetDataFromPackage(DataPackage &data);
    void SetDataToPackage(DataPackage &data);
    void Step();
    void Exit();

private:
    torch::jit::script::Module policy;
    std::string policy_path;

    Eigen::Vector3d gravity_vec_eigen{Eigen::Vector3d::Zero()};
    Eigen::VectorXd default_dof_pos_eigen;
    Eigen::VectorXd last_actions_eigen;
    Eigen::VectorXd inputdata_eigen;
    Eigen::VectorXd outputdata_eigen;
    Eigen::VectorXd pos_hist_buf_eigen;
    Eigen::VectorXd vel_hist_buf_eigen;
    Eigen::VectorXd q_origin;
    Eigen::VectorXd dot_q_origin;
    Eigen::VectorXd imu_angular_vel;
    Eigen::VectorXd commands;
    Eigen::VectorXd rl_p;
    Eigen::VectorXd rl_d;
    Eigen::VectorXd action_scale;

    int decimation{1};
    int num_proprio{0};
    int num_observations{0};
    int num_of_dofs{0};
    int history_len{0};
    int step_num{0};

    double lin_vel_scale{1.0};
    double ang_vel_scale{1.0};
    double dof_pos_scale{1.0};
    double dof_vel_scale{1.0};
    double js_vx_desire{0.0};
    double js_vy_desire{0.0};
    double js_OmegaZ_desire{0.0};

    template <typename T>
    std::vector<T> ReadVectorFromYaml(const YAML::Node &node)
    {
        std::vector<T> values;
        for (const auto &val : node)
        {
            values.push_back(val.as<T>());
        }
        return values;
    }

    Eigen::Vector3d Euler_ZYXToGravityVec(const Eigen::Vector3d &euler_zyx) const;
};
