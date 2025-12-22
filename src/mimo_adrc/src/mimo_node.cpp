// #include "rclcpp/rclcpp.hpp"
// #include "mimo_adrc/MimoADRC.h"
// #include "std_msgs/msg/float64_multi_array.hpp"
// #include <vector>

// /**
//  * @brief 全向无人机 6-DOF ADRC 仿真节点
//  * * 实现了位置(X, Y, Z)和姿态(Roll, Pitch, Yaw)的完整解耦控制。
//  */
// class MimoNode : public rclcpp::Node {
// public:
//     MimoNode() : Node("adrc_observer_node") {
//         // === 1. 初始化位置控制器 (X, Y, Z) ===
//         adrc_pos_ = std::make_shared<MimoADRC>();
//         // b0 约为 1/m = 1/3.0 = 0.33
//         adrc_pos_->init(0.01f, 10.0f, 0.33333333f); 
//         adrc_pos_->setGains(100.0f, 300.0f, 1000.0f, 25.0f, 10.0f, 0.8f);

//         // === 2. 初始化姿态控制器 (Roll, Pitch, Yaw) ===
//         adrc_att_ = std::make_shared<MimoADRC>();
//         // b0 约为 1/I = 1/0.04 = 25.0
//         adrc_att_->init(0.01f, 50.0f, 25.0f); 
//         adrc_att_->setGains(100.0f, 300.0f, 1000.0f, 100.0f, 20.0f, 0.8f);

//         // === 3. 系统状态初始化 (6 自由度) ===
//         state_pos_.assign(6, 0.0f); // [x, y, z, roll, pitch, yaw]
//         state_vel_.assign(6, 0.0f); // [vx, vy, vz, d_roll, d_pitch, d_yaw]
//         u_prev_pos_.assign(8, 0.0f); // 位置控制输出 (Fx, Fy, Fz)
//         u_prev_att_.assign(8, 0.0f); // 姿态控制输出 (Mx, My, Mz)

//         // 物理参数定义
//         mass_ = 3.0f;           // 质量 kg
//         I_ = {0.04f, 0.04f, 0.08f}; // 惯量 Jxx, Jyy, Jzz

//         pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/adrc/full_debug", 10);
//         start_t_ = this->now();
//         timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MimoNode::step, this));
        
//         RCLCPP_INFO(this->get_logger(), "6-DOF ADRC Simulation Started.");
//     }

// private:
//     void step() {
//         double t = (this->now() - start_t_).seconds();
//         // === 新增：自动停止逻辑 ===
//         // 我们设定在 18.0 秒（即所有扰动结束后的 1 秒）自动关闭节点
//         if (t > 18.0) {
//             RCLCPP_INFO(this->get_logger(), "仿真任务已完成（18.0s），正在自动关闭节点...");
        
//             // 调用 shutdown 会通知 rclcpp 停止运行
//             // 之后 main 函数中的 rclcpp::spin() 会返回，程序正常退出
//             rclcpp::shutdown();
//             return; 
//         }
//         // ========================
//         float h = 0.01f;

//         // --- 1. 注入外部干扰 (应用于 Y 轴和 Pitch 轴进行验证) ---
//         float dist_force_y = (t > 5.0 && t < 10.0) ? 3.0f : ((t >= 10.0 && t < 15.0) ? 2.0f * sin(2.0 * t) : 0.0f);
//         float dist_torque_p = (t > 12.0 && t < 17.0) ? 0.5f : 0.0f;




//         // --- 2. 无人机 6-DOF 动力学仿真 ---
//         // A. 位置动力学 (简化全向模型: a = (F_control + F_dist) / m - g)
//         float acc[3];
//         acc[0] = u_prev_pos_[0] / mass_;
//         acc[1] = (u_prev_pos_[1] + dist_force_y) / mass_;
//         acc[2] = u_prev_pos_[2] / mass_ - 9.81f; // 考虑重力

//         for (int i = 0; i < 3; i++) {
//             state_vel_[i] += acc[i] * h;
//             state_pos_[i] += state_vel_[i] * h;
//         }

//         // B. 姿态动力学 (alpha = (M_control + M_dist) / I)
//         float alpha[3];
//         alpha[0] = u_prev_att_[0] / I_[0];
//         alpha[1] = (u_prev_att_[1] + dist_torque_p) / I_[1];
//         alpha[2] = u_prev_att_[2] / I_[2];

//         for (int i = 0; i < 3; i++) {
//             state_vel_[i+3] += alpha[i] * h;
//             state_pos_[i+3] += state_vel_[i+3] * h;
//         }

//         // --- 3. ADRC 控制计算 ---
//         // A. 位置环目标 (经典的 8 字航线)
//         float v0_pos[8] = {2.0f * (float)sin(0.5f * t), 2.0f * (float)sin(1.0f * t), 1.0f, 0,0,0,0,0};
//         float y_pos[8] = {state_pos_[0], state_pos_[1], state_pos_[2], 0,0,0,0,0};
        
//         adrc_pos_->observe(y_pos, u_prev_pos_.data());
//         float u0_pos[8], z3_pos[8];
//         adrc_pos_->calculateU0(v0_pos, u0_pos);
//         adrc_pos_->getZ3(z3_pos);

//         // 更新推力输出 (Fx, Fy, Fz)
//         for (int i = 0; i < 3; i++) {
//             u_prev_pos_[i] = (u0_pos[i] - z3_pos[i]) / 0.33f;
//         }

//         // B. 姿态环目标 (保持水平或追踪特定角度)
//         float v0_att[8] = {0.0f, 0.0f, 0.0f, 0,0,0,0,0}; 
//         float y_att[8] = {state_pos_[3], state_pos_[4], state_pos_[5], 0,0,0,0,0};

//         adrc_att_->observe(y_att, u_prev_att_.data());
//         float u0_att[8], z3_att[8];
//         adrc_att_->calculateU0(v0_att, u0_att);
//         adrc_att_->getZ3(z3_att);

//         // 更新力矩输出 (Mx, My, Mz)
//         for (int i = 0; i < 3; i++) {
//             u_prev_att_[i] = (u0_att[i] - z3_att[i]) / 25.0f;
//         }

//         // --- 4. 发布调试数据 ---
//         // 为了看清 Y 轴观测效果，我们选取 Y 轴通道的数据发布
//         float s_y[3]; adrc_pos_->getStates(1, s_y);
//         std_msgs::msg::Float64MultiArray msg;
//         msg.data = {
//             t,                // [0] 时间
//             state_pos_[1],    // [1] 实际 Y 位置
//             s_y[0],           // [2] 观测 z1
//             state_vel_[1],    // [3] 实际 Y 速度
//             s_y[1],           // [4] 观测 z2
//             dist_force_y/3.0, // [5] 实际干扰加速度
//             s_y[2]            // [6] 观测 z3
//         };
//         pub_->publish(msg);
//     }

//     // 成员变量
//     std::shared_ptr<MimoADRC> adrc_pos_, adrc_att_;
//     std::vector<float> state_pos_, state_vel_, u_prev_pos_, u_prev_att_;
//     float mass_;
//     std::vector<float> I_;
    
//     rclcpp::Time start_t_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<MimoNode>());
//     rclcpp::shutdown();
//     return 0;
// }

#include "rclcpp/rclcpp.hpp"
#include "mimo_adrc/MimoADRC.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <vector>

/**
 * @brief 全向无人机 6-DOF ADRC 仿真节点
 * - 实现了位置(X, Y, Z)和姿态(Roll, Pitch, Yaw)的完整解耦控制。
 * - 采样频率：500Hz (2ms)
 * - 运行区间：10s - 30s
 */
class MimoNode : public rclcpp::Node {
public:
    MimoNode() : Node("adrc_observer_node") {
        // === 1. 初始化位置控制器 (X, Y, Z) ===
        adrc_pos_ = std::make_shared<MimoADRC>();
        // 采样步长改为 0.002f (500Hz)
        adrc_pos_->init(0.002f, 10.0f, 0.33333333f); 
        adrc_pos_->setGains(100.0f, 300.0f, 1000.0f, 25.0f, 10.0f, 0.8f);

        // === 2. 初始化姿态控制器 (Roll, Pitch, Yaw) ===
        adrc_att_ = std::make_shared<MimoADRC>();
        // 采样步长改为 0.002f (500Hz)
        adrc_att_->init(0.002f, 50.0f, 25.0f); 
        adrc_att_->setGains(100.0f, 300.0f, 1000.0f, 100.0f, 20.0f, 0.8f);

        // === 3. 系统状态初始化 (6 自由度) ===
        state_pos_.assign(6, 0.0f);
        state_vel_.assign(6, 0.0f);
        u_prev_pos_.assign(8, 0.0f);
        u_prev_att_.assign(8, 0.0f);

        mass_ = 3.0f;           // 质量 kg
        I_ = {0.04f, 0.04f, 0.08f}; // 惯量

        pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/adrc/full_debug", 10);
        start_t_ = this->now();

        // === 修改：定时器改为 2ms (500Hz) ===
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2), 
            std::bind(&MimoNode::step, this));
        
        RCLCPP_INFO(this->get_logger(), "6-DOF ADRC Simulation Started at 500Hz. Active from 10s to 30s.");
    }

private:
    /**
 * @brief 10通道 ADRC 仿真步进函数
 * 结构：4路已知标量扰动 + 2个 (3x1) 未知扰动向量
 */
void MimoNode::step() {
    double t = (this->now() - start_t_).seconds();

    // === 1. 自动停止逻辑 ===
    if (t > 30.0) {
        RCLCPP_INFO(this->get_logger(), "仿真结束，正在关闭节点...");
        rclcpp::shutdown();
        return; 
    }

    // === 2. 等待阶段 ===
    if (t < 10.0) return; 

    float h = 0.002f; // 500Hz 采样步长

    // --- 3. 构造仿真干扰源 (用于模拟物理环境) ---
    
    // A. 前4路：已知的标量干扰 (例如：系统已知的常值偏置或预设程序)
    float f_known_sc[4] = { 1.2f, -0.8f, 0.5f, 0.0f };

    // B. 中间3路 (Vector 1)：未知的 3x1 扰动向量 (需要 z3 准确跟踪)
    float real_dist_v1[3] = {
        2.0f * (float)sin(0.5f * t),          // X轴：慢变正弦
        (t > 15.0 && t < 25.0) ? 4.0f : 0.0f,  // Y轴：阶跃干扰
        -1.5f                                  // Z轴：常值干扰
    };

    // C. 最后3路 (Vector 2)：未知的 3x1 扰动向量 (需要 z3 准确跟踪)
    float real_dist_v2[3] = {
        0.5f * (float)cos(2.0f * t),          // 较快变化的干扰
        0.1f * (float)t,                      // 斜坡干扰
        0.0f
    };

    // --- 4. 动力学仿真更新 (10通道系统) ---
    // 假设系统增益 b0 = 1/mass, 这里 mass 取 3.0
    float inv_mass = 1.0f / 3.0f;

    for (int i = 0; i < 10; i++) {
        float current_total_dist = 0.0f;
        if (i < 4) {
            current_total_dist = f_known_sc[i];
        } else if (i < 7) {
            current_total_dist = real_dist_v1[i-4];
        } else {
            current_total_dist = real_dist_v2[i-7];
        }

        // 物理更新：acc = (控制力 + 真实总干扰) / 质量
        float acc = (u_prev_all_[i] + current_total_dist) * inv_mass;
        state_vel_[i] += acc * h;
        state_pos_[i] += state_vel_[i] * h;
    }

    // --- 5. ADRC 观测器计算 ---
    // 获取当前 10 个通道的测量值 (位置)
    float y_all[10];
    for(int i = 0; i < 10; i++) y_all[i] = state_pos_[i];

    // 调用 MimoADRC 观察器
    // 传入：测量向量、上一次控制向量、前4路已知扰动
    adrc_controller_->observe(y_all, u_prev_all_.data(), f_known_sc);

    // 计算控制律 u0
    float v0_all[10] = {0.0f}; // 假设目标是维持在 0 点，以观察干扰消除能力
    float u0_all[10];
    adrc_controller_->calculateU0_All(v0_all, u0_all);

    // --- 6. 获取观测结果并实现补偿 ---
    float obs_v1[3], obs_v2[3];
    adrc_controller_->getObservedVectors(obs_v1, obs_v2);

    for (int i = 0; i < 10; i++) {
        float compensation = 0.0f;
        if (i < 4) {
            // 已知项：直接前馈补偿
            compensation = f_known_sc[i];
        } else if (i < 7) {
            // 未知 Vector 1：由 ESO 跟踪得到的 z3 补偿
            compensation = obs_v1[i-4];
        } else {
            // 未知 Vector 2：由 ESO 跟踪得到的 z3 补偿
            compensation = obs_v2[i-7];
        }
        
        // 最终输出：(期望加速度 - 总观测干扰) / b0
        u_prev_all_[i] = (u0_all[i] - compensation) / inv_mass;
    }

    // --- 7. 发布调试数据 (验证 Vector 1 第 2 分量(Y轴) 的跟踪效果) ---
    std_msgs::msg::Float64MultiArray debug_msg;
    debug_msg.data = {
        t,                // [0]
        real_dist_v1[1],  // [1] 实际干扰值 (阶跃)
        obs_v1[1],        // [2] ESO z3 跟踪到的干扰值
        real_dist_v1[0],  // [3] 实际干扰值 (正弦)
        obs_v1[0]         // [4] ESO z3 跟踪到的干扰值
    };
    pub_->publish(debug_msg);
}

    std::shared_ptr<MimoADRC> adrc_pos_, adrc_att_;
    std::vector<float> state_pos_, state_vel_, u_prev_pos_, u_prev_att_;
    float mass_;
    std::vector<float> I_;
    
    rclcpp::Time start_t_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MimoNode>());
    rclcpp::shutdown();
    return 0;
}