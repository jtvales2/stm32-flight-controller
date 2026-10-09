#include "fc_mode.h"

#include "fc_context.h"

static uint8_t mode_angle_latched = 1u;  // 1=ANGLE, 0=ACRO
static uint8_t mode_inited = 0u;

void fc_mode_update(void)
{
    if (!mode_inited) {
        mode_angle_latched = s.rc.mode_sw ? 1u : 0u;
        mode_inited = 1u;
    }

    /*
     * 模式锁存：防止空中误切
     * 允许更新锁存的条件：
     * A) 未解锁
     * B) 已解锁但还没起飞过
     * C) 起飞过后：必须当前不在空中 + 低油门
     */
    {
        uint8_t allow_mode_change =
            (!s.arm.armed) ||
            (!s.air.ever_on) ||
            ((!s.air.on) && s.rc.thr_low);

        if (allow_mode_change) {
            mode_angle_latched = s.rc.mode_sw ? 1u : 0u;
        }
    }
}

fc_mode_t fc_mode_get(void)
{
    return mode_angle_latched ? FC_MODE_ANGLE : FC_MODE_ACRO;
}

const char *fc_mode_str(fc_mode_t mode)
{
    return (mode == FC_MODE_ANGLE) ? "ANGLE" : "ACRO";
}
