#include "kinematics.h"

// 麦轮标准逆运动学方程
__attribute__((section(".ramfunc"))) WheelSpeed4 Kinematics::inverse_kinematics(float vx, float vy, float vw) {
    WheelSpeed4 speeds;
    speeds.lf = vy + vx + vw;
    speeds.lb = vy - vx + vw;
    speeds.rf = vy - vx - vw;
    speeds.rb = vy + vx - vw;
    return speeds;
}