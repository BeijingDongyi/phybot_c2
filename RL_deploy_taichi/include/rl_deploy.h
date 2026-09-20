

#include "DataPackage/include/DataPackage.h"
#include <torch/script.h>
#include <fstream>
#include <iomanip>

class rl_deploy_taichi{

public:

    rl_deploy_taichi();
    ~rl_deploy_taichi();

    // void Init();

    void GetDataFromPackage(DataPackage &data);
    void SetDataToPackage(DataPackage &data);
    void Step();
    void Exit();
    Eigen::MatrixXd LoadMotionLib(const std::string& filename, bool skipHeader);
    Eigen::Vector3d Euler_ZYXToGravityVec(Eigen::Vector3d euler_zyx);


private:
    torch::jit::script::Module policy;
    Eigen::VectorXd default_dof_pos_eigen;
    Eigen::VectorXd last_actions_eigen;
    Eigen::VectorXd inputdata_hist_cur;
    Eigen::VectorXd inputdata_hist_all;
    Eigen::VectorXd inputdata_future_all;



    Eigen::VectorXd inputdata_future_lin_vel;
    Eigen::VectorXd inputdata_future_ang_vel;
    Eigen::VectorXd inputdata_future_gravity;
    Eigen::VectorXd inputdata_future_dof_pos;
    Eigen::VectorXd inputdata_future_dof_vel;



    Eigen::VectorXd inputdata_all;
    Eigen::VectorXd outputdata_eigen;


    Eigen::VectorXd tor_origin;
    Eigen::VectorXd sim_tor;

    Eigen::VectorXd pos_hist_buf_eigen;
    Eigen::VectorXd vel_hist_buf_eigen;
    Eigen::VectorXd action_hist_buf_eigen;
    Eigen::VectorXd rot_hist_buf_eigen;
    Eigen::VectorXd ang_vel_hist_buf_eigen;

    Eigen::VectorXd q_origin;
    Eigen::VectorXd dot_q_origin;

    Eigen::Vector3d gravity_vec_eigen;
    Eigen::VectorXd lin_vel;           
    Eigen::VectorXd imu_angular_vel;      
    Eigen::VectorXd imu_zyx;      
    Eigen::VectorXd commands;
    // Eigen::VectorXd task_commands;    
    Eigen::VectorXd base_euler;   
    Eigen::VectorXd base_quat;   
    Eigen::VectorXd dof_pos;           
    Eigen::VectorXd dof_vel;           
    Eigen::VectorXd last_actions;
    Eigen::VectorXd task_obs;

    torch::Tensor torque_limits;
    torch::Tensor commands_scale;
    torch::Tensor default_dof_pos;
    torch::Tensor output_dof_pos;


    Eigen::VectorXd desire_q_origin;
    Eigen::VectorXd desire_dot_q_origin;
    Eigen::Vector3d desire_gravity_vec_eigen;
    Eigen::VectorXd desire_lin_vel;           
    Eigen::VectorXd desire_imu_angular_vel;      
    Eigen::VectorXd desire_imu_zyx;      


    int decimation;
    int num_proprio;
    int num_future;
    int num_observations;

    int num_js_commands;
    int num_mimic_commands;
    int num_commands;
    double clip_obs;
    double clip_actions_lower;
    double clip_actions_upper;
    // double action_scale;
    int num_of_dofs;
    int history_len;
    double lin_vel_scale;
    double ang_vel_scale;
    double dof_pos_scale;
    double dof_vel_scale;
    double js_vx_desire;
    double js_vy_desire;
    double js_OmegaZ_desire;
    int step_num = 0;
    int step_num_2 = 0;
    int episode_length = 0;
    int frame_count = 0;
    int num_motion_frames;



    double phase;
    std::string policy_path;
    std::string data_path;
    std::string mimic_policy_path;
    std::string motion_lib_path;
    std::vector<int> indices_to_remove;

    Eigen::VectorXd rl_p;
    Eigen::VectorXd rl_d;
    Eigen::VectorXd action_scale;

    template<typename T>
    std::vector<T> ReadVectorFromYaml(const YAML::Node& node)
    {
        std::vector<T> values;
        for(const auto& val : node)
        {
            values.push_back(val.as<T>());
        }
        return values;
    }

    Eigen::MatrixXd motion_frames;
    Eigen::VectorXd remove_indices(const Eigen::VectorXd& input, const std::vector<int>& indices_to_remove);
    void assign_with_skipped_zero(Eigen::VectorXd& target,const Eigen::VectorXd& source,const std::vector<int>& indices_to_skip);

    // torch::Tensor EulerZYXRotateInverse(torch::Tensor euler, torch::Tensor v);
    // torch::Tensor QuatRotateInverse(torch::Tensor q, torch::Tensor v);

    // Logging members
    std::ofstream task_obs_log_file;
    bool enable_task_obs_logging;
    // Logging functions
    void InitTaskObsLogging(const std::string& log_file_path = "../RL_deploy/logs/task_obs_log.csv");
    void LogTaskObs(const torch::Tensor& task_obs_tensor);
    void CloseTaskObsLogging();

    Eigen::VectorXd dataL;
    std::ofstream foutData;
    bool dataLog(Eigen::VectorXd &v, std::ofstream &f);



        // ---------------------- 全局配置（根据你的Vector维度修改）----------------------
    const int WINDOW_SIZE = 10;            // 滑动窗口大小（3/5可选）
    const int VEC_DIM = Eigen::VectorXd::RowsAtCompileTime;  // 向量维度（自动适配，无需手动改）

    // ---------------------- 全局缓存（滑动平均+一阶IIR）----------------------
    std::vector<Eigen::VectorXd> slide_buffer;  // 滑动平均缓存（存储Vector）
    double b0 = 0.0;                        // 一阶IIR系数（标量，所有维度共用）
    double b1 = 0.0;                        // 一阶IIR系数（标量，所有维度共用）
    Eigen::VectorXd x_prev1;                    // IIR输入历史缓存（Vector）
    Eigen::VectorXd y_prev1;                    // IIR输出历史缓存（Vector）

    Eigen::VectorXd realtime_combined_firstorder(const Eigen::VectorXd& x_current); 
    Eigen::VectorXd slide_average(const Eigen::VectorXd& x_current);
    void init_combined_firstorder(double fs, double cutoff, const Eigen::VectorXd& init_vec);

    Eigen::MatrixXd loadTxt(const std::string& path);

    Eigen::MatrixXd data_motion;

    struct MotionData {
        Eigen::MatrixXd root_rot;         // (T, 4) or (T, 3)
        Eigen::MatrixXd base_lin_vel;     // (T, 3)
        Eigen::MatrixXd base_ang_vel;     // (T, 3)
        Eigen::MatrixXd dof_pos;          // (T, D)
        Eigen::MatrixXd dof_vel;          // (T, D)


    };

    MotionData load_yaml_to_eigen(const std::string& yaml_path);
    Eigen::MatrixXd yaml_to_eigen(const YAML::Node& node);
    // MotionData load_npz_to_eigen(const std::string& npz_path);
    // Eigen::MatrixXd npy2_to_eigen(const cnpy::NpyArray& arr);
    Eigen::MatrixXd quat_xyzw_to_gravity_body(Eigen::MatrixXd& quat_xyzw);
    MotionData motion;


    uint8_t action_cmd;
    int8_t action_Result;
};


