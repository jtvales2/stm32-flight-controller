#include <math.h>
#include "stm32f4xx_hal.h"
#include "autocal.h"

extern int vofa_fw_send_line(const char *tag, const char *fmt, ...);

#define GZ_STEP_LIMIT_DPS   (0.5f)

#define STILL_GYR_TOL_DPS     (0.5f)
#define STILL_ANORM_TOL_G     (0.04f)
#define STILL_MIN_MS          (2500u)
#define STILL_REARM_MS        (3000u)
#define STILL_MIN_SAMPLES     (180u)

#define STILL_LP_ALPHA        (0.02f)
#define STILL_GYR_TOL2_DPS2   (STILL_GYR_TOL_DPS * STILL_GYR_TOL_DPS)
#define STILL_ANORM_TOL2_G2   (STILL_ANORM_TOL_G * STILL_ANORM_TOL_G)

static IMU_BMI088_FE *s_fe  = NULL;
static Mahony        *s_mah = NULL;
static uint8_t        s_enabled = 0u;

static uint32_t s_still_t0_ms       = 0;
static uint32_t s_still_last_set_ms = 0;
static int      s_still_n           = 0;
static int      s_avg_n             = 0;
static float    s_still_gz_sum      = 0.0f;

static float s_lp_g2     = 0.0f;
static float s_lp_anerr2 = 0.0f;
static float s_lp_gz     = 0.0f;

static void autocal_reset_state(void)
{
    s_still_t0_ms       = 0;
    s_still_last_set_ms = 0;
    s_still_n           = 0;
    s_avg_n             = 0;
    s_still_gz_sum      = 0.0f;

    s_lp_g2     = 0.0f;
    s_lp_anerr2 = 0.0f;
    s_lp_gz     = 0.0f;
}

static inline uint32_t irq_save(void)
{
    uint32_t p = __get_PRIMASK();
    __disable_irq();
    return p;
}
static inline void irq_restore(uint32_t p)
{
    __set_PRIMASK(p);
}

void imu_autocal_init(IMU_BMI088_FE *fe, Mahony *mah)
{
    s_fe  = fe;
    s_mah = mah;
    s_enabled = 0u;
    autocal_reset_state();
}

void imu_autocal_set_enabled(uint8_t enabled)
{
    enabled = enabled ? 1u : 0u;
    if (!enabled && s_enabled) {
        autocal_reset_state();
    }
    s_enabled = enabled;
}

void maybe_static_calibrate_gz(const stamped_vec3_t *A,
                               const stamped_vec3_t *G)
{
    if (!s_enabled) {
        return;
    }

    if (!s_fe || !s_mah) {
        return; // 还没 init，直接跳过
    }

    float g2 = G->v[0]*G->v[0] + G->v[1]*G->v[1] + G->v[2]*G->v[2];
    float a2 = A->v[0]*A->v[0] + A->v[1]*A->v[1] + A->v[2]*A->v[2];
    float e2 = (a2 - 1.0f);  e2 *= e2;

    s_lp_g2     += STILL_LP_ALPHA * (g2 - s_lp_g2);
    s_lp_anerr2 += STILL_LP_ALPHA * (e2 - s_lp_anerr2);
    s_lp_gz     += STILL_LP_ALPHA * (G->v[2] - s_lp_gz);

    int still_ok = (s_lp_g2     < STILL_GYR_TOL2_DPS2) &&
                   (s_lp_anerr2 < STILL_ANORM_TOL2_G2);

    static uint32_t dbg_gate_ms = 0;
    uint32_t now = HAL_GetTick();
    if (now - dbg_gate_ms >= 120u) {
        float gmag_dbg  = sqrtf(s_lp_g2);
        float anerr_dbg = sqrtf(s_lp_anerr2);
/*        vofa_fw_send_line("still_gate",
                          "acc_ok=%d gyr_ok=%d gmag=%.3f anerr=%.3f n=%d",
                          (s_lp_anerr2 < STILL_ANORM_TOL2_G2),
                          (s_lp_g2     < STILL_GYR_TOL2_DPS2),
                          gmag_dbg, anerr_dbg, s_still_n);*/
        dbg_gate_ms = now;
    }

    if (still_ok) {
        if (s_still_n == 0) {
            s_still_t0_ms  = now;
            s_still_gz_sum = 0.0f;
            s_avg_n        = 0;
        }
        s_still_n++;
        s_avg_n++;
        s_still_gz_sum += s_lp_gz;

        uint32_t elapsed = now - s_still_t0_ms;

        static uint32_t dbg_elapsed_ms = 0;
        if (now - dbg_elapsed_ms >= 200u) {
            /*vofa_fw_send_line("still_elapsed", "%lu",
                              (unsigned long)elapsed);*/
            dbg_elapsed_ms = now;
        }

        if (((elapsed >= STILL_MIN_MS) ||
             ((uint32_t)s_still_n >= STILL_MIN_SAMPLES)) &&
            (now - s_still_last_set_ms) >= STILL_REARM_MS) {

            const float mean_gz = s_still_gz_sum / (float)s_avg_n;

            float adj = mean_gz;
            if (adj >  GZ_STEP_LIMIT_DPS) adj =  GZ_STEP_LIMIT_DPS;
            if (adj < -GZ_STEP_LIMIT_DPS) adj = -GZ_STEP_LIMIT_DPS;

            IMU_BMI088_FE *fe  = s_fe;
            Mahony        *mah = s_mah;

            float gyr_b[3] = {
                fe->cfg.gyr_bias_dps[0],
                fe->cfg.gyr_bias_dps[1],
                fe->cfg.gyr_bias_dps[2] + adj
            };
            float acc_b[3] = {
                fe->cfg.acc_bias_g[0],
                fe->cfg.acc_bias_g[1],
                fe->cfg.acc_bias_g[2]
            };

            uint32_t p = irq_save();
						IMU_BMI088_FE_SetBias(fe, acc_b, gyr_b);
						irq_restore(p);

            mah->st.gyro_bias[2] = 0.0f;
            s_still_last_set_ms = now;

            /*vofa_fw_send_line("calib",
                              "gz_bias+=%.4f(mean=%.4f) -> %.4f dps",
                              adj, mean_gz, gyr_b[2]);*/

            s_still_n      = 0;
            s_still_gz_sum = 0.0f;
            s_avg_n        = 0;
            s_still_t0_ms  = now;
        }
    } else {
        s_still_n      = 0;
        s_still_gz_sum = 0.0f;
        s_avg_n        = 0;
        s_still_t0_ms  = now;
    }
}
