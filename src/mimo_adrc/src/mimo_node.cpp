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
    void step() {
        double t = (this->now() - start_t_).seconds();

        // === 1. 自动停止逻辑：30秒结束 ===
        if (t > 30.0) {
            RCLCPP_INFO(this->get_logger(), "仿真任务已完成（30.0s），正在自动关闭节点...");
            rclcpp::shutdown();
            return; 
        }

        // === 2. 过程开始逻辑：10秒前处于静止等待状态 ===
        if (t < 10.0) {
            return; 
        }

        // 步长设定为 0.002 (对应 500Hz)
        float h = 0.002f;

        // --- 3. 注入外部干扰 (调整时间区间以适应 10s-30s 运行过程) ---
        // Y轴推力干扰：15s-20s 恒定，20s-25s 正弦
        float dist_force_y = (t > 15.0 && t < 20.0) ? 3.0f : ((t >= 20.0 && t < 25.0) ? 2.0f * sin(2.0 * t) : 0.0f);
        // Pitch轴力矩干扰：22s-27s
        float dist_torque_p = (t > 22.0 && t < 27.0) ? 0.5f : 0.0f;

        // --- 4. 无人机 6-DOF 动力学仿真 ---
        float acc[3];
        acc[0] = u_prev_pos_[0] / mass_;
        acc[1] = (u_prev_pos_[1] + dist_force_y) / mass_;
        acc[2] = u_prev_pos_[2] / mass_ - 9.81f; 

        for (int i = 0; i < 3; i++) {
            state_vel_[i] += acc[i] * h;
            state_pos_[i] += state_vel_[i] * h;
        }

        float alpha[3];
        alpha[0] = u_prev_att_[0] / I_[0];
        alpha[1] = (u_prev_att_[1] + dist_torque_p) / I_[1];
        alpha[2] = u_prev_att_[2] / I_[2];

        for (int i = 0; i < 3; i++) {
            state_vel_[i+3] += alpha[i] * h;
            state_pos_[i+3] += state_vel_[i+3] * h;
        }

        // --- 5. ADRC 控制计算 ---
        // 位置环
        float v0_pos[8] = {2.0f * (float)sin(0.5f * t), 2.0f * (float)sin(1.0f * t), 1.0f, 0,0,0,0,0};
        float y_pos[8] = {state_pos_[0], state_pos_[1], state_pos_[2], 0,0,0,0,0};
        
        adrc_pos_->observe(y_pos, u_prev_pos_.data());
        float u0_pos[8], z3_pos[8];
        adrc_pos_->calculateU0(v0_pos, u0_pos);
        adrc_pos_->getZ3(z3_pos);

        for (int i = 0; i < 3; i++) {
            // 注意：补偿时建议使用更精确的质量倒数，这里保持原样或改为 1.0f/mass_
            u_prev_pos_[i] = (u0_pos[i] - z3_pos[i]) / 0.33333333f;
        }

        // 姿态环
        float v0_att[8] = {0.0f, 0.0f, 0.0f, 0,0,0,0,0}; 
        float y_att[8] = {state_pos_[3], state_pos_[4], state_pos_[5], 0,0,0,0,0};

        adrc_att_->observe(y_att, u_prev_att_.data());
        float u0_att[8], z3_att[8];
        adrc_att_->calculateU0(v0_att, u0_att);
        adrc_att_->getZ3(z3_att);

        for (int i = 0; i < 3; i++) {
            // b0 对应 1/I
            float b0_inv = (i == 2) ? 0.08f : 0.04f; 
            u_prev_att_[i] = (u0_att[i] - z3_att[i]) / (1.0f / b0_inv);
        }

        // --- 6. 发布调试数据 ---
        float s_y[3]; adrc_pos_->getStates(1, s_y);
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {
            t,                // [0] 时间
            state_pos_[1],    // [1] 实际 Y 位置
            s_y[0],           // [2] 观测 z1
            state_vel_[1],    // [3] 实际 Y 速度
            s_y[1],           // [4] 观测 z2
            dist_force_y/mass_, // [5] 实际干扰加速度
            s_y[2]            // [6] 观测 z3
        };
        pub_->publish(msg);
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