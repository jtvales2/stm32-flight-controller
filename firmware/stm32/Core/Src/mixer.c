#include "mixer.h"

/*
 * 坐标/电机布局（机体 NED：x 前、y 右、z 下；右手系）
 *   motor[0] 前右 FR
 *   motor[1] 后右 RR
 *   motor[2] 后左 RL
 *   motor[3] 前左 FL
 *
 * 这里的 roll/pitch/yaw 指的是“混控输入命令”的符号约定（与下方公式一致）：
 *   roll  > 0 ：向右滚转（右侧下、左侧上）
 *              → 左侧(2/3)加推力，右侧(0/1)减推力
 *   pitch > 0 ：向前俯仰（机头下）
 *              → 后侧(1/2)加推力，前侧(0/3)减推力
 *   yaw   > 0 ：偏航正方向由电机旋向决定；
 *              本混控令(0/2)为 +y，(1/3)为 -y
 *
 * 对应公式：
 *   m0 = t - p - r + y
 *   m1 = t + p - r - y
 *   m2 = t + p + r + y
 *   m3 = t - p + r - y
 */
void mixer_quadx(const mixer_cmd_t *u, float out[4])
{
    const float t = u->throttle;
    const float r = u->roll;
    const float p = u->pitch;
    const float y = u->yaw;

    // 纯混控：不做任何限幅/缩放（统一交给上层 motor_limit）
    out[0] = t - p - r + y;
    out[1] = t + p - r - y;
    out[2] = t + p + r + y;
    out[3] = t - p + r - y;
}

