#include "DataPackage/include/DataPackage.h"
#include <torch/script.h>
#include <cassert>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_set>
#include <vector>

class rl_deploy_ase{

public:

    explicit rl_deploy_ase(const std::string& profile_name = "");
    ~rl_deploy_ase();

    // void Init();

    void GetDataFromPackage(DataPackage &data);
    void SetDataToPackage(DataPackage &data);
    void Step();
    void Exit();
    Eigen::Vector3d Euler_ZYXToGravityVec(Eigen::Vector3d euler_zyx);

private:
    torch::jit::script::Module ase_policy;
    torch::jit::script::Module hlc_policy;
    torch::jit::script::Module llc_policy;
    
    Eigen::VectorXd default_dof_pos_eigen;
    Eigen::VectorXd last_actions_eigen;
    Eigen::VectorXd inputdata_eigen;
    Eigen::VectorXd outputdata_eigen;
    Eigen::VectorXd current_latent_eigen;
    Eigen::VectorXd hlc_obs_eigen;
    Eigen::MatrixXd skill_control_latents_eigen;

    Eigen::VectorXd hist_obs_buf_eigen;
    Eigen::VectorXd imu_ang_vel;      
    Eigen::Vector3d base_lin_vel_eigen;
    Eigen::Vector3d gravity_vec_eigen;      
    Eigen::VectorXd commands;
    Eigen::VectorXd q_origin;           
    Eigen::VectorXd dot_q_origin;           
    Eigen::VectorXd dof_pos_obs_eigen;
    Eigen::VectorXd dof_vel_obs_eigen;

    torch::Tensor torque_limits;
    torch::Tensor skill_id_tensor;
    torch::Tensor phase_id_tensor;

    int decimation;
    int num_proprio;
    int num_observations;
    int obs_frame_size;
    double clip_obs;
    double clip_actions_lower;
    double clip_actions_upper;
    Eigen::VectorXd action_scale_eigen;
    Eigen::VectorXd action_output_mask_eigen;
    int num_of_dofs;
    int history_len;
    int num_commands;
    int num_required_proprio;
    double lin_vel_scale;
    double ang_vel_scale;
    double command_lin_scale;
    double command_ang_scale;
    double projected_gravity_scale;
    double dof_pos_scale;
    double dof_vel_scale;
    double action_obs_scale;
    double js_vx_desire;
    double js_vy_desire;
    double js_OmegaZ_desire;
    double js_z_height_desire;
    int step_num = 0;
    bool is_first_step = true;
    bool use_command_obs = false;
    bool debug_print_obs = false;
    double debug_print_hz = 10.0;
    int policy_step_count = 0;
    int debug_print_every_policy_steps = 1;
    bool debug_print_robot_state = false;
    double debug_print_robot_state_hz = 1.0;
    int debug_print_robot_state_every_policy_steps = 1;
    double control_period = 0.002;
    bool use_hlc_residual = false;
    bool use_h1_trace = false;
    bool latent_initialized = false;
    int latent_skill_id = -1;
    bool h1_trace_loop = false;
    int h1_phase_increment = 1;
    int h1_phase_id = 0;
    int h1_active_motion_id = -1;
    int h1_active_trigger_count = -1;
    double h1_trace_step_seconds = 0.0;

    std::string ase_control_mode = "h0_center";
    std::string profile_name;
    std::string ase_policy_path;
    std::string skill_centers_path;
    std::string h1_policy_path;
    std::string motion_traces_path;
    std::string hlc_manifest_path;
    std::string hlc_policy_path;
    std::string llc_policy_path;
    std::vector<int> indices_to_remove;
    std::vector<int> hw_to_policy_idx;
    std::vector<int> policy_to_hw_idx;
    std::vector<std::string> policy_joint_names;
    std::vector<std::string> skill_names;
    std::vector<int> motion_trace_lengths;
    bool use_joint_remap = false;
    int ase_skill_id = 0;
    int ase_skill_loop = 0;
    int ase_skill_trigger_count = 0;
    int num_skills = 1;
    int last_logged_skill_id = -1;
    int last_logged_skill_loop = -1;
    int last_logged_skill_trigger_count = -1;
    int latent_dim = 0;
    int llc_obs_dim = 0;
    int llc_input_dim = 0;
    int llc_action_dim = 0;
    int high_level_obs_dim = 0;
    int hlc_obs_dim = 0;
    int hlc_steps = 1;
    double hlc_skill_residual_scale = 0.0;

    Eigen::VectorXd rl_p;
    Eigen::VectorXd rl_d;

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

    Eigen::VectorXd remove_indices(const Eigen::VectorXd& input, const std::vector<int>& indices_to_remove);
    Eigen::VectorXd reorder_hw_to_policy(const Eigen::VectorXd& hw_order_vec) const;
    Eigen::VectorXd reorder_policy_to_hw(const Eigen::VectorXd& policy_order_vec) const;
    void assign_with_skipped_zero(Eigen::VectorXd& target,const Eigen::VectorXd& source,const std::vector<int>& indices_to_skip);
    bool dataLog(Eigen::VectorXd &v, std::ofstream &f);
    void load_h0_runtime(const YAML::Node& config, const std::filesystem::path& config_dir);
    void load_h1_runtime(const YAML::Node& config, const std::filesystem::path& config_dir);
    void load_hlc_runtime(const YAML::Node& config, const std::filesystem::path& config_dir);
    void update_hlc_latent();

    std::ofstream foutData;
    Eigen::VectorXd dataL;
    Eigen::VectorXd tor_origin;
    Eigen::VectorXd sim_tor;
    Eigen::VectorXd sim_tor_p;
    Eigen::VectorXd sim_tor_d;
};

class rl_deploy_ase_fk : public rl_deploy_ase {
public:
    rl_deploy_ase_fk() : rl_deploy_ase("fast_kick") {}
};