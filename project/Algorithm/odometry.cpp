#include "odometry.h"
#include "math.h"

__attribute__((section(".dtcm_data"))) Odometry chassis_odometry;

Odometry::Odometry() : global_x(0.0f), global_y(0.0f) {}


// 麦轮正运动学解算与全局积分
__attribute__((section(".ramfunc"))) void Odometry::update_global_position(const int16_t* encoder_counts, float current_yaw_rad) {

    // 将编码器计数转换为轮子移动的距离（cm）
    float d_lf = encoder_counts[0] * PLUSE_TO_CM;
    float d_lb = encoder_counts[1] * PLUSE_TO_CM;
    float d_rf = encoder_counts[2] * PLUSE_TO_CM;
    float d_rb = encoder_counts[3] * PLUSE_TO_CM;

    // 计算机器人在局部坐标系中的位移
    float dy_local = (d_lf + d_lb + d_rf + d_rb) / 4.0f;
    float dx_local = (d_lf - d_lb - d_rf + d_rb) / 4.0f;

    // 将局部坐标系的位移转换到全局坐标系
    float cos_yaw = cosf(current_yaw_rad);
    float sin_yaw = sinf(current_yaw_rad);

    float dx_global = dx_local * cos_yaw + dy_local * sin_yaw;
    float dy_global = -dx_local * sin_yaw + dy_local * cos_yaw;

    global_x += dx_global;
    global_y += dy_global;
}
