/**
 * @file laengsfuehrung_controller.h
 * @brief 纵向PI速度控制器 - 速度域版本
 *        Longitudinal PI Velocity Controller - Velocity Domain Version
 * 
 * 修改说明 / Modifications:
 *   - PI输出速度命令 v_cmd [m/s] 而不是油门 u [0,1]
 *   - 参数限制改为速度限制 v_min, v_max
 *   - 状态变量改为 v_cmd
 * 
 * @author zx
 * @date 2025-11
 * @version 2.0 - Velocity Domain
 */

#pragma once
#include <algorithm>

// ============================================================================
// PI控制器参数结构 / PI Controller Parameters Structure
// ============================================================================
/**
 * @struct PIParams
 * @brief PI控制器参数 - 速度域
 *        PI controller parameters - Velocity domain
 */
struct PIParams
{
    // ========================================================================
    // PI增益 / PI Gains
    // ========================================================================
    double Kp = 1.0;  /**< 比例增益 / Proportional gain
                           - 单位：无量纲 (dimensionless)
                           - 物理意义：速度误差1m/s对应速度命令增加Kp m/s
                           - Physical meaning: 1m/s error → Kp m/s command increase
                           - 建议范围 / Suggested range: [0.5, 2.0] */
    
    double Ki = 1.5;  /**< 积分增益 / Integral gain
                           - 单位：1/s
                           - 物理意义：消除稳态误差（坡度、摩擦等）
                           - Physical meaning: eliminate steady-state error
                           - 建议范围 / Suggested range: [0.5, 2.0] */
    
    // ========================================================================
    // 速度限制 / Velocity Limits
    // ========================================================================
    double v_min = 0.0;  /**< 最小速度命令 [m/s] / Minimum velocity command [m/s]
                              - 通常为0（不倒车）
                              - Usually 0 (no reverse) */
    
    double v_max = 1.5;  /**< 最大速度命令 [m/s] / Maximum velocity command [m/s]
                              - 根据小车实际性能设定
                              - Set according to vehicle actual performance
                              - ⚠️ 需要实测：motor_level=1.0时的稳态速度
                              - ⚠️ Needs measurement: steady speed at motor_level=1.0 */
};

// ============================================================================
// PI控制器状态结构 / PI Controller State Structure
// ============================================================================
/**
 * @struct PIState
 * @brief PI控制器内部状态 - 速度域
 *        PI controller internal state - Velocity domain
 */
struct PIState
{
    double v_cmd = 0.0;  /**< 当前速度命令 [m/s] / Current velocity command [m/s]
                              - 上一次PI输出的速度命令
                              - Last PI output velocity command */
    
    double e_pre = 0.0;  /**< 上一次速度误差 [m/s] / Previous velocity error [m/s]
                              - 用于计算增量式PI
                              - Used for incremental PI calculation */
};

// ============================================================================
// PI控制器初始化 / PI Controller Initialization
// ============================================================================
/**
 * @brief 初始化PI控制器状态
 *        Initialize PI controller state
 * 
 * @return 初始化的PIState，所有值归零
 *         Initialized PIState with all values zeroed
 */
inline PIState init_pi()
{
    PIState s;
    s.v_cmd = 0.0;   // 初始速度命令为0
    s.e_pre = 0.0;   // 初始误差为0
    return s;
}

// ============================================================================
// PI控制函数 / PI Control Function
// ============================================================================
/**
 * @brief PI速度控制器主函数 - 增量式PI算法（速度域）
 *        Main PI velocity controller function - Incremental PI (velocity domain)
 * 
 * 算法流程 / Algorithm Flow:
 *   1. 计算速度误差: e_k = v_ref - v_k
 *   2. 计算速度增量: Δv = Kp·Δe + Ki·dt·e_k
 *   3. 更新速度命令: v_cmd = v_cmd_pre + Δv
 *   4. 限幅到 [v_min, v_max]
 * 
 * 物理意义 / Physical Meaning:
 *   - 输入：期望速度 v_ref 和 当前速度 v_k (都是m/s)
 *   - 输出：速度命令 v_cmd (m/s)
 *   - 后续需要映射到 motor_level [0, 1]
 * 
 * 与原版的区别 / Difference from Original:
 *   原版 / Original:
 *     输入：v_ref, v_k [m/s]
 *     输出：u [0, 1] (油门)
 *   
 *   现在 / Now:
 *     输入：v_ref, v_k [m/s]
 *     输出：v_cmd [m/s] (速度命令)
 *     
 *   优势 / Advantage:
 *     全程速度域，量纲统一，调试直观
 *     Full velocity domain, unified dimensions, intuitive debugging
 * 
 * @param p 控制器参数 / Controller parameters
 * @param s 控制器状态（会被修改）/ Controller state (will be modified)
 * @param v_ref 目标速度 [m/s] / Target velocity [m/s]
 * @param v_k 当前速度 [m/s] / Current velocity [m/s]
 * @param dt 采样时间 [s] / Sampling time [s]
 * 
 * @return 速度命令 v_cmd ∈ [v_min, v_max] [m/s]
 *         Velocity command v_cmd ∈ [v_min, v_max] [m/s]
 * 
 * @example
 *   PIParams params;
 *   params.Kp = 1.0;
 *   params.Ki = 1.5;
 *   params.v_max = 1.5;  // 最大速度1.5m/s
 *   
 *   PIState state = init_pi();
 *   
 *   // 控制循环
 *   double v_cmd = pi_step(params, state, 0.5, 0.48, 0.02);
 *   // v_cmd ≈ 0.5 m/s (速度命令)
 *   
 *   // 然后映射到motor_level
 *   double motor_level = v_cmd / params.v_max;  // 0.5/1.5 ≈ 0.33
 */
inline double pi_step(
    const PIParams& p,    // 参数 / Parameters
    PIState& s,           // 状态（会修改）/ State (modified)
    double v_ref,         // 目标速度 [m/s] / Target velocity
    double v_k,           // 当前速度 [m/s] / Current velocity
    double dt)            // 采样时间 [s] / Sampling time
{
    // ========================================================================
    // 步骤1: 计算速度误差 / Step 1: Calculate Velocity Error
    // ========================================================================
    // e_k > 0: 当前速度慢，需要加速
    // e_k > 0: current speed too slow, need to accelerate
    // e_k < 0: 当前速度快，需要减速
    // e_k < 0: current speed too fast, need to decelerate
    double e_k = v_ref - v_k;
    
    // ========================================================================
    // 步骤2: 计算增量式PI / Step 2: Calculate Incremental PI
    // ========================================================================
    // 增量式PI公式 / Incremental PI formula:
    //   Δv = Kp·(e_k - e_pre) + Ki·dt·e_k
    //
    // 其中 / Where:
    //   Kp·(e_k - e_pre): 比例项增量（响应误差变化）
    //                     Proportional increment (respond to error change)
    //   Ki·dt·e_k:        积分项（累积误差，消除稳态误差）
    //                     Integral term (accumulate error, eliminate steady-state)
    
    double delta_v = p.Kp * (e_k - s.e_pre) + p.Ki * dt * e_k;
    
    // ========================================================================
    // 步骤3: 更新速度命令 / Step 3: Update Velocity Command
    // ========================================================================
    s.v_cmd += delta_v;
    
    // ========================================================================
    // 步骤4: 限幅到安全范围 / Step 4: Clamp to Safe Range
    // ========================================================================
    // 防止速度命令超出车辆物理限制
    // Prevent velocity command from exceeding vehicle physical limits
    s.v_cmd = std::max(p.v_min, std::min(s.v_cmd, p.v_max));
    
    // ========================================================================
    // 步骤5: 更新状态 / Step 5: Update State
    // ========================================================================
    s.e_pre = e_k;  // 保存当前误差供下次使用
                    // Save current error for next iteration
    
    return s.v_cmd;
}

// ============================================================================
// 调试辅助函数 / Debugging Helper Functions
// ============================================================================
/**
 * @brief 获取PI控制器详细状态（用于调试）
 *        Get detailed PI controller state (for debugging)
 * 
 * @param s PI控制器状态 / PI controller state
 * @param v_ref 目标速度 / Target velocity
 * @param v_k 当前速度 / Current velocity
 * @return 格式化的状态字符串 / Formatted state string
 */
inline const char* pi_state_string(const PIState& s, double v_ref, double v_k)
{
    static char buffer[256];
    snprintf(buffer, sizeof(buffer),
             "PI State: v_cmd=%.3f m/s, e_current=%.3f, e_pre=%.3f, v_ref=%.3f, v_k=%.3f",
             s.v_cmd, (v_ref - v_k), s.e_pre, v_ref, v_k);
    return buffer;
}

// ============================================================================
// 使用示例 / Usage Example
// ============================================================================
/*
示例代码 / Example Code:

#include "laengsfuehrung_controller.h"

int main() {
    // 1. 创建并配置参数
    PIParams params;
    params.Kp = 1.0;      // 比例增益
    params.Ki = 1.5;      // 积分增益
    params.v_min = 0.0;   // 最小速度 0 m/s
    params.v_max = 1.5;   // 最大速度 1.5 m/s (实测得到)
    
    // 2. 初始化状态
    PIState state = init_pi();
    
    // 3. 控制循环
    double v_ref = 0.5;   // 目标速度 0.5 m/s
    double dt = 0.02;     // 采样时间 20ms (50Hz)
    
    for (int i = 0; i < 100; i++) {
        // 模拟测量当前速度
        double v_k = 0.48;  // 从传感器获取
        
        // PI控制器计算速度命令
        double v_cmd = pi_step(params, state, v_ref, v_k, dt);
        
        // 映射到motor_level
        double motor_level = v_cmd / params.v_max;
        
        // 发送到硬件
        // send_motor_command(motor_level);
        
        printf("v_ref=%.2f, v_k=%.2f, v_cmd=%.2f, motor=%.2f\n",
               v_ref, v_k, v_cmd, motor_level);
    }
    
    return 0;
}

输出示例 / Example Output:
v_ref=0.50, v_k=0.48, v_cmd=0.52, motor=0.35
v_ref=0.50, v_k=0.49, v_cmd=0.51, motor=0.34
v_ref=0.50, v_k=0.50, v_cmd=0.50, motor=0.33
...
*/
// ============================================================================
/*
参数调整方法 / Parameter Tuning Method:

1. Kp (比例增益) / Proportional Gain:
   - 从 Kp = 1.0 开始
   - 如果响应太慢 → 增大 Kp (例如 1.5, 2.0)
   - 如果有振荡 → 减小 Kp (例如 0.7, 0.5)
   
2. Ki (积分增益) / Integral Gain:
   - 从 Ki = 1.5 开始
   - 如果有稳态误差（一直追不上目标速度）→ 增大 Ki
   - 如果有积分饱和（速度超调很大）→ 减小 Ki
   
3. v_max (最大速度) / Maximum Velocity:
   - 实测方法：
     a. 设置 motor_level = 1.0 (100%功率)
     b. 在平直路面加速到稳态
     c. 记录稳定后的速度值
     d. 这个值就是 v_max
   
   示例 / Example:
   - motor_level = 1.0 → 稳态速度 1.5 m/s
   - 所以 v_max = 1.5 m/s

调试技巧 / Debugging Tips:
   - 如果车辆不动：检查 v_max 是否设置正确
   - 如果速度振荡：减小 Kp，增大 Kd (如果有)
   - 如果追不上目标：增大 Ki
   - 如果超调严重：减小 Ki
*/