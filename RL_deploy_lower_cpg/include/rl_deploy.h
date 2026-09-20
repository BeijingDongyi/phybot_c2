#include "DataPackage/include/DataPackage.h"
#include <torch/script.h>
#include <fstream>
#include <iomanip>
#include <unordered_map>
#include <string>

class rl_deploy_lower_cpg{

public:

    rl_deploy_lower_cpg();
    ~rl_deploy_lower_cpg();

    // void Init();

    void GetDataFromPackage(DataPackage &data);
    void SetDataToPackage(DataPackage &data);
    void Step();
    void Exit();
    Eigen::MatrixXd LoadMotionLib(const std::string& filename, bool skipHeader);
    Eigen::Vector3d Euler_ZYXToGravityVec(Eigen::Vector3d euler_zyx);



private:
    torch::jit::script::Module policy;
    Eigen::Vector3d gravity_vec_eigen;
    Eigen::VectorXd default_dof_pos_eigen;
    Eigen::VectorXd last_actions_eigen;
    Eigen::VectorXd inputdata_eigen;
    Eigen::VectorXd outputdata_eigen;

    Eigen::VectorXd pos_hist_buf_eigen;
    Eigen::VectorXd vel_hist_buf_eigen;
    Eigen::VectorXd action_hist_buf_eigen;
    Eigen::VectorXd rot_hist_buf_eigen;
    Eigen::VectorXd ang_vel_hist_buf_eigen;

    Eigen::VectorXd q_origin;
    Eigen::VectorXd dot_q_origin;

    Eigen::VectorXd motor_upper_body_buffer;

    Eigen::VectorXd lin_vel;
    Eigen::VectorXd imu_angular_vel;      
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

    int decimation;
    int num_proprio;
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
    double z_height_scale;
    double js_vx_desire;
    double js_vy_desire;
    double js_OmegaZ_desire;
    double js_z_height_desire;
    int control_mode;
    int step_num = 0;
    int episode_length = 0;
    int frame_count = 0;
    int num_motion_frames;



    double phase;
    std::string policy_path;
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

    std::ofstream task_obs_log_file;
    bool enable_task_obs_logging;
    int16_t action_cmd;
    int8_t action_Result;

    void InitTaskObsLogging(const std::string& log_file_path = "../RL_deploy/logs/task_obs_log.csv");
    void LogTaskObs(const torch::Tensor& task_obs_tensor);
    void CloseTaskObsLogging();

    // Adding

    #define COUPLING_CHANGE_RATE_IN 4.0
    #define COUPLING_CHANGE_RATE_DE 1.0
    #define AMPLITUDE_CHANGE_RATE_IN 1.05
    #define AMPLITUDE_CHANGE_RATE_DE 0.995

    int num_feet = 4;
    double step_dt;
    double gait_cy;
    Eigen::Vector4d cycle_r;
    double cycle_r_max = 1.0;
    double cycle_r_min = 0.001;
    Eigen::Vector4d px;
    Eigen::Vector4d py;
    double rate = 10.0;
    Eigen::Vector4d phase_offset;
    Eigen::Vector4d phase_offset_target;
    double phase_offset_change_rate;
    Eigen::Vector4d phase_offset_stand;
    Eigen::Vector4d phase_offset_walk;
    Eigen::Vector4d phase_offset_run;
    Eigen::Vector4d coupling_change_rate;

    Eigen::Vector4d contact_ratio;
    double coff_b = 1000.0;
    double coupling_change_rate_increase;
    double coupling_change_rate_decrease;
    Eigen::Vector4d amplitude_change_rate;
    double amplitude_change_rate_increase;
    double amplitude_change_rate_decrease;

    bool walk_flag = false;
    bool stand_flag = false;

    
    void CPGInit(int num_feet, float dt, float frequency);
    Eigen::VectorXd CPGGammaCal();
    Eigen::MatrixXd CPGCoupling();
    void CPGPhaseChange();
    void CPGHopfOscillator();
    void CPGReset(bool flag);
    void CPGSetCy(Eigen::VectorXd Cycle_r);
    void CPGSetCou(Eigen::VectorXd Coupling_change_rate);
    void CPGSetA(Eigen::VectorXd Amplitude_change_rate);
    void CPGSetP(Eigen::VectorXd Phase_offset);
    void CPGSetC(Eigen::VectorXd Contact_ratio);
    void CPGSetncy(double frequency);
    void CPGSetSn();

    void CPGSetWP_1();
    void CPGSetWP_2();
    void CPGSetWP_3();
    void CPGSetWP_4();
    void CPGSetWP_5();
    void CPGSetWP_6();
    void CPGSetWP_7();
    void CPGSetWP_8();

    double flag_1 = 0.05;
    double flag_2 = 0.3;
    double flag_3 = 0.6;
    double flag_4 = 0.9;
    double flag_5 = 1.2;
    double flag_6 = 1.5;
    double flag_7 = 1.8;
    double flag_8 = 2.1;
    double flag_9 = 2.5;

    

    Eigen::VectorXd CPGGetXNorm();
    Eigen::VectorXd CPGGetYNorm();
    void UpdateGaitGeneratorPattern(Eigen::VectorXd Commands);
    void CommandRefinement();
//读bin
    std::vector<float> action_bin_;
    int action_bin_dim_ = 27;
    int action_bin_frames_ = 0;
    int action_bin_idx_ = 0;
    bool use_action_bin_ = false;

    // model -> bin路径 映射表（从yaml的 action_bin_map 加载）
    std::unordered_map<int, std::string> action_bin_map_;

    // model 状态跟踪
    int model_          = 0;    // 当前从 DataPackage 收到的 model 值
    int current_model_  = -1;   // 上一帧已生效的 model（-1表示尚未初始化）

    void LoadActionBin(const std::string& path, int dim = 27);
    // 根据 model 查 action_bin_map_ 并重新加载 bin，找不到则关闭 bin 播放
    void SwitchActionBin(int model);
    const float* NextActionFrameLoop();
    bool action_bin_ended_ = false;

    // 用于检测 control_mode 的 0->1 边沿
    int last_control_mode_ = -1;
    Eigen::VectorXd upper_mid_cmd_;//上肢控制缓存
    Eigen::VectorXd motor_Pos_desire_cmd_;
    // 重置播放
    void ResetActionBinPlayback();
    void UpdateUpperBodyCmd();
    //轨迹播放平滑
    bool upper_transition_active_ = false;
    int  upper_transition_N_ = 500;   // 平滑1s 500 frames ~ 1s
    int  upper_transition_k_ = 0;

    Eigen::VectorXd upper_transition_start_;
    Eigen::VectorXd upper_transition_target_;

    // 进入 mode=1 时：过渡结束后从第1帧开始播（避免重复 frame0）
    bool pending_bin_start_after_transition_ = false;
    int  bin_start_idx_after_transition_ = 1;

    // peek：不推进 idx，用来拿 frame0 当过渡目标
    const float* PeekActionFrame(int idx) const;
};

