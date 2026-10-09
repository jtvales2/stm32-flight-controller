#include "fc_mixer_out.h"

#include "mixer.h"
#include "motors.h"

static void fc_motor_limit(float m[4], float out_min, float out_max)
{
    float mn, mx;
    float range, span;
    int i;

    if (out_max < 0.0f) out_max = 0.0f;
    if (out_max > 1.0f) out_max = 1.0f;
    if (out_min < 0.0f) out_min = 0.0f;
    if (out_min > out_max) out_min = out_max;

    mn = m[0];
    mx = m[0];

    for (i = 1; i < 4; i++) {
        if (m[i] < mn) mn = m[i];
        if (m[i] > mx) mx = m[i];
    }

    range = out_max - out_min;
    span  = mx - mn;

    if (span < 1e-6f) {
        float v = m[0];

        if (v < out_min) v = out_min;
        if (v > out_max) v = out_max;

        for (i = 0; i < 4; i++) {
            m[i] = v;
        }

        return;
    }

    if (span > range + 1e-6f) {
        float k = (range > 1e-6f) ? (range / span) : 0.0f;

        for (i = 0; i < 4; i++) {
            m[i] = (m[i] - mn) * k + out_min;
        }
    } else {
        if (mn < out_min) {
            float d = out_min - mn;

            for (i = 0; i < 4; i++) {
                m[i] += d;
            }

            mx += d;
        }

        if (mx > out_max) {
            float d = out_max - mx;

            for (i = 0; i < 4; i++) {
                m[i] += d;
            }
        }
    }

    for (i = 0; i < 4; i++) {
        if (m[i] < out_min) m[i] = out_min;
        if (m[i] > out_max) m[i] = out_max;
    }
}

void fc_mixer_output(const fc_mixer_cmd_t *u,
                     float out_min,
                     float out_max)
{
    float m[4];

    mixer_cmd_t mu;

    mu.throttle = u->throttle;
    mu.roll     = u->roll;
    mu.pitch    = u->pitch;
    mu.yaw      = u->yaw;

    mixer_quadx(&mu, m);

    fc_motor_limit(m, out_min, out_max);

    motors_write(m);
}
