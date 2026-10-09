#include "fc_baro.h"
#include "timebase.h"
#include "ms5611.h"

#include <math.h>

#ifndef FC_BARO_POLL_US
#define FC_BARO_POLL_US      2000u
#endif

#ifndef FC_BARO_P0_SAMPLES
#define FC_BARO_P0_SAMPLES   80
#endif

#ifndef FC_BARO_ALT_TAU_SEC
#define FC_BARO_ALT_TAU_SEC  0.60f
#endif

#ifndef FC_BARO_VZ_TAU_SEC
#define FC_BARO_VZ_TAU_SEC   0.80f
#endif

#ifndef FC_BARO_P_LPF_TAU_SEC
#define FC_BARO_P_LPF_TAU_SEC 0.35f
#endif

#ifndef FC_BARO_PRESS_MIN_PA
#define FC_BARO_PRESS_MIN_PA 30000
#endif

#ifndef FC_BARO_PRESS_MAX_PA
#define FC_BARO_PRESS_MAX_PA 110000
#endif

#ifndef FC_BARO_TEMP_MIN_CENTI
#define FC_BARO_TEMP_MIN_CENTI (-4000)
#endif

#ifndef FC_BARO_TEMP_MAX_CENTI
#define FC_BARO_TEMP_MAX_CENTI 8500
#endif

#ifndef FC_BARO_P0_SPREAD_REJECT_PA
#define FC_BARO_P0_SPREAD_REJECT_PA 300
#endif

#ifndef FC_BARO_PRESS_SPIKE_REJECT_PA
#define FC_BARO_PRESS_SPIKE_REJECT_PA 600
#endif

#ifndef FC_BARO_ALT_JUMP_REJECT_M
#define FC_BARO_ALT_JUMP_REJECT_M 8.0f
#endif

#define FC_BARO_MED_N 5u

static ms5611_t s_dev;
static uint8_t  s_inited = 0;

static uint32_t s_next_ts = 0;
static uint32_t s_last_ts = 0;

static uint8_t  s_p0_ready = 0;
static int      s_p0_n = 0;
static int64_t  s_p0_acc = 0;

static float    s_alt_f = 0.0f;
static float    s_vz_f  = 0.0f;
static float    s_alt_prev = 0.0f;
static float    s_alt_raw_prev = 0.0f;
static uint8_t  s_have_alt = 0;
static uint8_t  s_filter_ready = 0;

static int32_t  s_p_hist[FC_BARO_MED_N];
static uint8_t  s_p_hist_n = 0;
static uint8_t  s_p_hist_i = 0;
static float    s_p_f = 0.0f;
static uint8_t  s_p_filter_ready = 0;
static int32_t  s_last_good_p_pa = 0;

static float alpha_tau(float dt, float tau)
{
    if (tau <= 0.0f) return 1.0f;
    return dt / (tau + dt);
}

static int32_t iabs32(int32_t x)
{
    return (x < 0) ? -x : x;
}

static uint8_t baro_raw_sane(int32_t p_pa, int32_t t_centi)
{
    if (p_pa < FC_BARO_PRESS_MIN_PA || p_pa > FC_BARO_PRESS_MAX_PA) return 0u;
    if (t_centi < FC_BARO_TEMP_MIN_CENTI || t_centi > FC_BARO_TEMP_MAX_CENTI) return 0u;
    return 1u;
}

static void baro_clear_p0(void)
{
    s_p0_ready = 0u;
    s_p0_n = 0;
    s_p0_acc = 0;
}

static void baro_clear_filter(void)
{
    s_alt_f = 0.0f;
    s_vz_f  = 0.0f;
    s_alt_prev = 0.0f;
    s_alt_raw_prev = 0.0f;
    s_have_alt = 0u;
    s_filter_ready = 0u;
    s_last_ts = 0u;

    s_p_hist_n = 0u;
    s_p_hist_i = 0u;
    s_p_f = 0.0f;
    s_p_filter_ready = 0u;
}

static int32_t median_i32(const int32_t *v, uint8_t n)
{
    int32_t a[FC_BARO_MED_N];
    uint8_t i, j;

    if (n == 0u) return 0;
    if (n > FC_BARO_MED_N) n = FC_BARO_MED_N;

    for (i = 0u; i < n; i++) {
        a[i] = v[i];
    }

    for (i = 1u; i < n; i++) {
        int32_t key = a[i];
        j = i;
        while (j > 0u && a[j - 1u] > key) {
            a[j] = a[j - 1u];
            j--;
        }
        a[j] = key;
    }

    return a[n / 2u];
}

static uint8_t baro_pressure_filter_step(int32_t p_raw, float dt, int32_t *p_out)
{
    if (p_out == 0) return 0u;

    if (s_p_filter_ready) {
        int32_t pf_i = (int32_t)(s_p_f + ((s_p_f >= 0.0f) ? 0.5f : -0.5f));
        if (iabs32(p_raw - pf_i) > FC_BARO_PRESS_SPIKE_REJECT_PA) {
            return 0u;
        }
    }

    s_p_hist[s_p_hist_i] = p_raw;
    s_p_hist_i = (uint8_t)((s_p_hist_i + 1u) % FC_BARO_MED_N);
    if (s_p_hist_n < FC_BARO_MED_N) {
        s_p_hist_n++;
    }

    int32_t p_med = median_i32(s_p_hist, s_p_hist_n);

    if (!s_p_filter_ready) {
        s_p_f = (float)p_med;
        s_p_filter_ready = 1u;
    } else {
        float a = alpha_tau(dt, FC_BARO_P_LPF_TAU_SEC);
        s_p_f += a * ((float)p_med - s_p_f);
    }

    *p_out = (int32_t)(s_p_f + ((s_p_f >= 0.0f) ? 0.5f : -0.5f));
    return 1u;
}

HAL_StatusTypeDef fc_baro_init_spi(SPI_HandleTypeDef *hspi,
                                   GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    if (ms5611_init_spi(&s_dev, hspi, cs_port, cs_pin, MS5611_OSR_4096) != HAL_OK) {
        s_inited = 0u;
        return HAL_ERROR;
    }

    fc_baro_reset();
    s_inited  = 1u;
    s_next_ts = now_ticks();
    return HAL_OK;
}

void fc_baro_reset(void)
{
    baro_clear_p0();
    baro_clear_filter();
    s_last_good_p_pa = 0;
}

uint8_t fc_baro_zero_to_current(void)
{
    int32_t p0;

    if (!s_inited || !s_p0_ready || !s_p_filter_ready) {
        return 0u;
    }

    if (s_last_good_p_pa != 0) {
        p0 = s_last_good_p_pa;
    } else {
        p0 = (int32_t)(s_p_f + ((s_p_f >= 0.0f) ? 0.5f : -0.5f));
    }

    if (p0 < FC_BARO_PRESS_MIN_PA || p0 > FC_BARO_PRESS_MAX_PA) {
        return 0u;
    }

    ms56xx_set_p0(&s_dev, p0);
    baro_clear_filter();
    s_p0_ready = 1u;
    s_last_good_p_pa = p0;
    return 1u;
}

static uint8_t fc_baro_service(fc_baro_out_t *out)
{
    int32_t p_pa = 0;
    int32_t t_centi = 0;

    if (out == 0) {
        return 0u;
    }

    if (!ms5611_poll_spi(&s_dev, &p_pa, &t_centi)) {
        return 0u;
    }

    if (!baro_raw_sane(p_pa, t_centi)) {
        return 0u;
    }

    out->ts_ticks   = now_ticks();
    out->press_pa   = p_pa;
    out->temp_centi = t_centi;

    if (!s_p0_ready) {
        if (s_p0_n >= 8) {
            int32_t p_avg = (int32_t)(s_p0_acc / s_p0_n);
            if (iabs32(p_pa - p_avg) > FC_BARO_P0_SPREAD_REJECT_PA) {
                out->valid     = 0u;
                out->alt_rel_m = 0.0f;
                out->vz_mps    = 0.0f;
                return 1u;
            }
        }

        s_p0_acc += p_pa;
        s_p0_n++;

        if (s_p0_n >= FC_BARO_P0_SAMPLES) {
            int32_t p0 = (int32_t)(s_p0_acc / s_p0_n);
            ms56xx_set_p0(&s_dev, p0);
            s_p0_ready = 1u;
            baro_clear_filter();
        }

        out->valid     = 0u;
        out->alt_rel_m = 0.0f;
        out->vz_mps    = 0.0f;
        return 1u;
    }

    {
        uint32_t ts = out->ts_ticks;
        float dt = 0.01f;
        int32_t p_use = p_pa;

        if (s_last_ts != 0u) {
            dt = ticks_to_sec((uint32_t)(ts - s_last_ts));
            if (dt < 0.001f) dt = 0.001f;
            if (dt > 0.2f)   dt = 0.2f;
        }
        s_last_ts = ts;

        if (!baro_pressure_filter_step(p_pa, dt, &p_use)) {
            return 0u;
        }

        float alt = ms56xx_altitude_rel(&s_dev, p_use);

        if (!isfinite((double)alt)) {
            return 0u;
        }

        if (s_have_alt && fabsf(alt - s_alt_raw_prev) > FC_BARO_ALT_JUMP_REJECT_M) {
            return 0u;
        }
        s_alt_raw_prev = alt;
        s_have_alt = 1u;

        if (!s_filter_ready) {
            s_alt_f = alt;
            s_alt_prev = alt;
            s_vz_f = 0.0f;
            s_filter_ready = 1u;
        } else {
            float a_alt = alpha_tau(dt, FC_BARO_ALT_TAU_SEC);
            float vz;
            float a_vz;

            s_alt_f += a_alt * (alt - s_alt_f);

            vz = (s_alt_f - s_alt_prev) / dt;
            s_alt_prev = s_alt_f;

            a_vz = alpha_tau(dt, FC_BARO_VZ_TAU_SEC);
            s_vz_f += a_vz * (vz - s_vz_f);
        }

        out->valid     = 1u;
        out->press_pa  = p_use;
        out->alt_rel_m = s_alt_f;
        out->vz_mps    = s_vz_f;
        s_last_good_p_pa = p_use;
        return 1u;
    }
}

uint8_t fc_baro_service_sched(fc_baro_out_t *out)
{
    if (!s_inited) return 0u;

    uint32_t now = now_ticks();
    if ((int32_t)(now - s_next_ts) < 0) {
        return 0u;
    }
    s_next_ts = now + FC_BARO_POLL_US;

    return fc_baro_service(out);
}

