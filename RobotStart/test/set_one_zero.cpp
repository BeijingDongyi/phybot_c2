#include"../MotorList/include/MotorList.hpp"
#include"../MotorList/include/realtime_controller.hpp"
#include "ZeroState/include/ZeroState.h"
#include <fstream>  // 用于文件操作
#include "DataPackage/include/DataPackage.h"
#include"../MotorList/include/MotorList.hpp"


// 修复：返回值为int16_t（数字类型），适配Set_One_Zero的参数要求
std::vector<std::int16_t> readMotorIdsFromYaml(const std::string& yamlPath) {
    std::vector<std::int16_t> motorIds;
    try {
        // 加载YAML文件
        YAML::Node config = YAML::LoadFile(yamlPath);
        
        // 读取ID字段（仅处理序列格式：ID: - 2 - 4）
        if (config["set_zero_ID"]) {
            if (config["set_zero_ID"].IsSequence()) {
                for (const auto& idNode : config["set_zero_ID"]) {
                    // 直接读取为整数（适配YAML的数字格式）
                    try {
                        std::int16_t id = idNode.as<std::int16_t>();
                        motorIds.push_back(id);
                    } catch (const YAML::BadConversion& e) {
                        std::cerr << "警告：ID节点无法转换为整数，跳过该节点" << std::endl;
                        continue;
                    }
                }
            } else {
                throw std::runtime_error("YAML文件中ID字段不是序列格式（应为 ID: - 2 - 4）");
            }
        } else {
            throw std::runtime_error("YAML文件中未找到ID字段");
        }

        if (motorIds.empty()) {
            throw std::runtime_error("YAML文件中未读取到有效电机ID");
        }

        // 打印读取结果（调试用）
        std::cout << "从YAML读取到电机ID列表：" << std::endl;
        for (const auto& id : motorIds) {
            std::cout << " - " << id << std::endl;
        }

    } catch (const YAML::BadFile& e) {
        std::cerr << "错误：无法打开YAML文件 " << yamlPath << "，原因：" << e.what() << std::endl;
        exit(EXIT_FAILURE);
    } catch (const std::exception& e) {
        std::cerr << "错误：读取YAML文件失败，原因：" << e.what() << std::endl;
        exit(EXIT_FAILURE);
    }

    return motorIds;
}

int main()
{
    DataPackage package;
    package.init();
    std::string config_yaml1 = "../MotorList/config/phybot_mini_1.yaml";
    std::string config_yaml2 = "../MotorList/config/phybot_mini_2.yaml";
    const std::string motorIdYamlPath = "../MotorList/config/set_one_zero.yaml";
    
    MotorList motorlist;
    motorlist.Init(config_yaml1, config_yaml2, package);
    int motorNum = motorlist.num_motors;

    // 初始化Eigen向量（保持原有逻辑）
    Eigen::VectorXd pos_actual = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd vel_actual = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd tor_actual = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd init_pos_actual = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd init_vel_actual = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd pos_desire = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd vel_desire = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd tor_desire = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd zero_vector = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd ones_vector = Eigen::VectorXd::Ones(motorNum) * 0.1;
    Eigen::VectorXd direction_vector = Eigen::VectorXd::Ones(motorNum);

    // 修复：用int16_t类型接收ID列表（匹配函数返回值）
    std::vector<std::int16_t> targetMotorIds = readMotorIdsFromYaml(motorIdYamlPath);

    std::cout << "Motorlist init complete! " << std::endl;
    period_info P_info;

    // 修复：遍历int16_t类型的ID，传入Set_One_Zero
    for (const auto& targetId : targetMotorIds) {
        // 若Set_One_Zero需要字符串参数，可转换：std::to_string(targetId)
        if (!motorlist.Set_One_Zero(targetId)) { 
            std::cerr << "Motor setzero failed for ID: " << targetId << ", exiting program..." << std::endl;
            return EXIT_FAILURE;
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    std::cout << "init_pos_actual: " << init_pos_actual << std::endl;
    return 0;
}
