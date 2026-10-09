#include "pipeline.h"
#include "bmi08x_defs.h"          // BMI08X_*_X_LSB_REG, BMI08X_SPI_RD_MASK
#include "imu_bmi088_frontend.h"  // IMU_BMI088_FE + cfg/lpf/bias
#include "fc_context.h"
#include "fc_evt.h"
#include <string.h>

/* ====== DMA 仲裁：避免 ACC 被 GYR 饿死 ====== */
#define GYRO_STREAK_MAX 3u   /* 每连续 3 次 GYR，若 ACC pending 则插 1 次 ACC */

typedef struct {
    rb_vec_t *acc_data, *gyr_data;

    /* 去抖阈值 & 上一次 irq ts */
    uint32_t acc_thr_ticks, gyr_thr_ticks;
    uint32_t last_acc_ts, last_gyr_ts;
    IMU_PipelineStats stats;

    /* ===== DMA pending 状态机 ===== */
    uint8_t dma_mode;               /* 0=旧模式; 1=DMA pending 模式 */
    IMU_BMI088_HALCtx *hal;
    IMU_BMI088_FE     *fe;

    volatile uint8_t dma_busy;      /* 0=空闲; 1=DMA in-flight */
    volatile uint8_t kick_req;      /* EXTI 置位，poll/done 里 kick */

    pl_inflight_t inflight;
    uint32_t inflight_ts;

    volatile uint16_t acc_pending_cnt;
    volatile uint16_t gyr_pending_cnt;
    volatile uint32_t acc_pending_ts;
    volatile uint32_t gyr_pending_ts;

    uint8_t tx[IMU_DMA_MAX_XFER];
    uint8_t rx[IMU_DMA_MAX_XFER];
    uint8_t xfer_len;

    volatile uint32_t spi_err_cnt;

    uint8_t  streak_pending;
    uint8_t  streak_next;
		
		uint32_t dma_start_ms;
    uint32_t dma_timeout_win_t0_ms;
    uint16_t dma_timeout_win_cnt;

} _pl_t;

static _pl_t S;
static volatile uint8_t g_streak = 0u;

/* ====================== 小工具 ====================== */
static inline int16_t _le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t _irq_save(void)
{
    uint32_t p = __get_PRIMASK();
    __disable_irq();
    return p;
}

static inline void _irq_restore(uint32_t p)
{
    if (!p) __enable_irq();
}

static inline void _cs_set(pl_inflight_t which, GPIO_PinState s)
{
    if (!S.hal) return;
    if (which == PL_IF_ACC) {
        HAL_GPIO_WritePin(S.hal->acc_cs_port, S.hal->acc_cs_pin, s);
    } else if (which == PL_IF_GYR) {
        HAL_GPIO_WritePin(S.hal->gyr_cs_port, S.hal->gyr_cs_pin, s);
    }
}

/* ====================== pipeline 基础接口 ====================== */
void imu_pipeline_bind(rb_vec_t *acc_data, rb_vec_t *gyr_data)
{
    memset(&S, 0, sizeof(S));

    S.acc_data = acc_data;
    S.gyr_data = gyr_data;

    S.stats.min_acc_irq_dt = 0xFFFFFFFFu;
    S.stats.min_gyr_irq_dt = 0xFFFFFFFFu;

    S.dma_mode = 1u;
    S.hal = NULL;
    S.fe  = NULL;
}

void imu_pipeline_set_deglitch_ticks(uint32_t acc_ticks, uint32_t gyr_ticks)
{
    S.acc_thr_ticks = acc_ticks;
    S.gyr_thr_ticks = gyr_ticks;
}

void imu_pipeline_set_deglitch_by_fs(float acc_fs, float gyr_fs)
{
    uint32_t acc_t = (acc_fs > 0.f) ? sec_to_ticks(ACC_DEGLITCH_FRAC * (1.0f/acc_fs)) : 0u;
    uint32_t gyr_t = (gyr_fs > 0.f) ? sec_to_ticks(GYR_DEGLITCH_FRAC * (1.0f/gyr_fs)) : 0u;
    imu_pipeline_set_deglitch_ticks(acc_t, gyr_t);
}

const IMU_PipelineStats* imu_pipeline_stats(void) { return &S.stats; }

void imu_pipeline_reset_stats_window(void)
{
    uint32_t ps = _irq_save();
    S.stats.acc_dt_sum = 0u;
    S.stats.gyr_dt_sum = 0u;
    S.stats.acc_dt_n   = 0u;
    S.stats.gyr_dt_n   = 0u;
    S.stats.min_acc_irq_dt = 0xFFFFFFFFu;
    S.stats.min_gyr_irq_dt = 0xFFFFFFFFu;
    _irq_restore(ps);
}

uint32_t imu_pipeline_acc_deglitch_ticks(void){ return S.acc_thr_ticks; }
uint32_t imu_pipeline_gyr_deglitch_ticks(void){ return S.gyr_thr_ticks; }

/* ====================== IRQ 入口：去抖 + 统计 + (TS ring 或 pending) ====================== */
void imu_drdy_on_acc(uint32_t ts)
{
    uint32_t dt = S.last_acc_ts ? (ts - S.last_acc_ts) : 0xFFFFFFFFu;

    if (S.acc_thr_ticks != 0u) {
        if (dt <= S.acc_thr_ticks) return;
    }

    if (S.last_acc_ts) { S.stats.acc_dt_sum += dt; S.stats.acc_dt_n++; }
    S.last_acc_ts = ts;
    if (dt < S.stats.min_acc_irq_dt) S.stats.min_acc_irq_dt = dt;
    S.stats.acc_irq_cnt++;

    S.acc_pending_ts = ts;
    if (S.acc_pending_cnt != 0xFFFFu) S.acc_pending_cnt++;
    S.kick_req = 1u;
}

void imu_drdy_on_gyr(uint32_t ts)
{
    uint32_t dt = S.last_gyr_ts ? (ts - S.last_gyr_ts) : 0xFFFFFFFFu;

    if (S.gyr_thr_ticks != 0u) {
        if (dt <= S.gyr_thr_ticks) return;
    }

    if (S.last_gyr_ts) { S.stats.gyr_dt_sum += dt; S.stats.gyr_dt_n++; }
    S.last_gyr_ts = ts;
    if (dt < S.stats.min_gyr_irq_dt) S.stats.min_gyr_irq_dt = dt;
    S.stats.gyr_irq_cnt++;

    S.gyr_pending_ts = ts;
    if (S.gyr_pending_cnt != 0xFFFFu) S.gyr_pending_cnt++;
    S.kick_req = 1u;
}

/* ====================== 清道夫：积压控制 ====================== */
uint32_t imu_sweep_backlog(uint32_t *drop_acc, uint32_t *drop_gyr)
{
    uint32_t da = 0u, dg = 0u;

    // 防御性：未 bind 就直接返回
    if (!S.acc_data || !S.gyr_data) {
        if (drop_acc) *drop_acc = 0u;
        if (drop_gyr) *drop_gyr = 0u;
        return 0u;
    }

    // 和 DMA done/EXTI 并发时，避免 head/tail 快照撕裂
    uint32_t ps = _irq_save();

    // ---- ACC ----
    {
        // cap 是 buffer 容量（mask+1），用 32-bit 运算
        uint32_t capA = (uint32_t)(S.acc_data->mask + 1u);
        // head-tail 是 16-bit 环计数差，先取 uint16 再升到 32-bit
        uint32_t lenA = (uint32_t)(uint16_t)(S.acc_data->head - S.acc_data->tail);

        if (lenA > (capA * 3u / 4u)) {
            uint32_t keep = capA / 2u;
            uint32_t drop = lenA - keep;

            // 把 tail 拉到 head-keep（注意 head/tail 是 uint16_t）
            S.acc_data->tail = (uint16_t)(S.acc_data->head - (uint16_t)keep);

            da = drop;
            S.stats.acc_sweep_drop += drop;
        }
    }

    // ---- GYR ----
    {
        uint32_t capG = (uint32_t)(S.gyr_data->mask + 1u);
        uint32_t lenG = (uint32_t)(uint16_t)(S.gyr_data->head - S.gyr_data->tail);

        if (lenG > (capG * 3u / 4u)) {
            uint32_t keep = capG / 2u;
            uint32_t drop = lenG - keep;

            S.gyr_data->tail = (uint16_t)(S.gyr_data->head - (uint16_t)keep);

            dg = drop;
            S.stats.gyr_sweep_drop += drop;
        }
    }

    _irq_restore(ps);

    if (drop_acc) *drop_acc = da;
    if (drop_gyr) *drop_gyr = dg;
    return (da + dg);
}

/* ====================== DMA 状态机核心 ====================== */
static void _kick_dma_if_idle(void)
{
    if (!S.dma_mode || !S.hal || !S.hal->hspi || !S.fe) return;
    if (S.dma_busy) return;

    if (HAL_SPI_GetState(S.hal->hspi) != HAL_SPI_STATE_READY) return;

    pl_inflight_t sel = PL_IF_NONE;
    uint32_t ts = 0;
    uint16_t cnt = 0;

    uint32_t ps = _irq_save();

    if (S.dma_busy) { _irq_restore(ps); return; }

    uint16_t ac = S.acc_pending_cnt;
    uint16_t gc = S.gyr_pending_cnt;

    if (ac == 0u && gc == 0u) {
        S.kick_req = 0u;
        _irq_restore(ps);
        return;
    }

    if (ac && gc) {
        sel = (g_streak >= GYRO_STREAK_MAX) ? PL_IF_ACC : PL_IF_GYR;
    } else {
        sel = gc ? PL_IF_GYR : PL_IF_ACC;
    }

    if (sel == PL_IF_ACC) {
        cnt = ac;
        ts  = S.acc_pending_ts;
        S.acc_pending_cnt = 0u;
        if (cnt > 1u) S.stats.acc_merge_drop += (uint32_t)(cnt - 1u);
    } else {
        cnt = gc;
        ts  = S.gyr_pending_ts;
        S.gyr_pending_cnt = 0u;
        if (cnt > 1u) S.stats.gyr_merge_drop += (uint32_t)(cnt - 1u);
    }

    S.dma_busy    = 1u;
    S.inflight    = sel;
    S.inflight_ts = ts;

    _irq_restore(ps);

    uint8_t len;
    if (sel == PL_IF_GYR) {
        S.tx[0] = (uint8_t)(BMI08X_GYRO_X_LSB_REG | BMI08X_SPI_RD_MASK);
        len = 7u; /* 1(addr)+6(data) */
    } else {
        S.tx[0] = (uint8_t)(BMI08X_ACCEL_X_LSB_REG | BMI08X_SPI_RD_MASK);
        len = 8u; /* 1(addr)+1(dummy)+6(data) */
    }
    S.xfer_len = len;

    memset(S.tx + 1, 0, (size_t)(len - 1u));
    memset(S.rx, 0, (size_t)len);

    _cs_set(sel, GPIO_PIN_RESET);
		
		S.dma_start_ms = HAL_GetTick();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive_DMA(S.hal->hspi, S.tx, S.rx, len);
    if (st != HAL_OK) {
				
        _cs_set(sel, GPIO_PIN_SET);
        S.spi_err_cnt++;

        uint32_t ps2 = _irq_save();
        if (sel == PL_IF_ACC) {
            if (S.acc_pending_cnt == 0u) { S.acc_pending_cnt = 1u; S.acc_pending_ts = ts; }
            else { S.stats.acc_merge_drop += 1u; }
        } else {
            if (S.gyr_pending_cnt == 0u) { S.gyr_pending_cnt = 1u; S.gyr_pending_ts = ts; }
            else { S.stats.gyr_merge_drop += 1u; }
        }
        _irq_restore(ps2);

        S.dma_busy = 0u;
        S.inflight = PL_IF_NONE;
        S.xfer_len = 0u;
        S.streak_pending = 0u;
        return;
    }

    uint8_t next = 0u;
    if (sel == PL_IF_GYR) {
        next = (g_streak < GYRO_STREAK_MAX) ? (uint8_t)(g_streak + 1u) : (uint8_t)GYRO_STREAK_MAX;
    } else {
        next = 0u;
    }
    S.streak_pending = 1u;
    S.streak_next    = next;
}

#define IMU_DMA_TIMEOUT_MS  10u
#define IMU_DMA_TIMEOUT_HARD_PER_SEC 3u

static void imu_dma_timeout_note(uint32_t dt_ms)
{
    uint32_t now = HAL_GetTick();

    S.stats.dma_timeout_cnt++;

    if (S.dma_timeout_win_t0_ms == 0u ||
        (uint32_t)(now - S.dma_timeout_win_t0_ms) >= 1000u) {
        S.dma_timeout_win_t0_ms = now;
        S.dma_timeout_win_cnt = 1u;
    } else if (S.dma_timeout_win_cnt != 0xFFFFu) {
        S.dma_timeout_win_cnt++;
    }

    fc_evt_push(FC_EVT_IMU_DMA_TIMEOUT,
                (int16_t)S.dma_timeout_win_cnt,
                (int16_t)((S.stats.dma_timeout_cnt > 32767u) ? 32767u : S.stats.dma_timeout_cnt),
                (int16_t)((dt_ms > 32767u) ? 32767u : dt_ms),
                (int16_t)(s.arm.armed ? 1 : 0));

    if (s.arm.armed && S.dma_timeout_win_cnt > IMU_DMA_TIMEOUT_HARD_PER_SEC) {
        fc_emergency_stop(FC_FS_IMU);
    }
}

void imu_pipeline_dma_poll(void)
{
    if (!S.dma_mode) return;

    //如果 DMA 卡死，强制中止并恢复
    if (S.dma_busy) {
        uint32_t dt = HAL_GetTick() - S.dma_start_ms;
        if (dt > IMU_DMA_TIMEOUT_MS) {
            SPI_HandleTypeDef *h = S.hal ? S.hal->hspi : NULL;

            imu_dma_timeout_note(dt);

            _cs_set(S.inflight, GPIO_PIN_SET);
            if (h) {
                (void)HAL_SPI_DMAStop(h);
                __HAL_SPI_DISABLE(h);
                __HAL_SPI_ENABLE(h);
                h->ErrorCode = HAL_SPI_ERROR_NONE;
                h->State     = HAL_SPI_STATE_READY;
            }

            S.dma_busy = 0u;
            S.inflight = PL_IF_NONE;
            S.xfer_len = 0u;
            S.streak_pending = 0u;
            S.kick_req = 1u;   // 让主循环尽快重新 kick
        }
        return;
    }

    if (S.kick_req == 0u) return;
    _kick_dma_if_idle();
    if (S.dma_busy) S.kick_req = 0u;
}


void imu_pipeline_dma_on_spi_done(SPI_HandleTypeDef *hspi)
{
    if (!S.dma_mode || !S.hal || hspi != S.hal->hspi) return;

    pl_inflight_t which = S.inflight;
    uint32_t ts = S.inflight_ts;

    _cs_set(which, GPIO_PIN_SET);

    float v[3] = {0};

    if (which == PL_IF_GYR) {
        /* rx[0]=junk, rx[1..6]=data */
        int16_t gx = _le16(&S.rx[1]);
        int16_t gy = _le16(&S.rx[3]);
        int16_t gz = _le16(&S.rx[5]);

        IMU_BMI088_FE_ProcessGyrRaw(S.fe, gx, gy, gz, v);
        rbv_push(S.gyr_data, ts, v);
    } else if (which == PL_IF_ACC) {
        /* rx[0]=junk, rx[1]=dummy, rx[2..7]=data */
        int16_t ax = _le16(&S.rx[2]);
        int16_t ay = _le16(&S.rx[4]);
        int16_t az = _le16(&S.rx[6]);

        IMU_BMI088_FE_ProcessAccRaw(S.fe, ax, ay, az, v);
        rbv_push(S.acc_data, ts, v);
    }

    if (S.streak_pending) g_streak = S.streak_next;
    S.streak_pending = 0u;

    S.dma_busy = 0u;
    S.inflight = PL_IF_NONE;
    S.xfer_len = 0u;

    S.kick_req = 1u;

}

void imu_pipeline_dma_on_spi_error(SPI_HandleTypeDef *hspi)
{
    if (!S.dma_mode || !S.hal || hspi != S.hal->hspi) return;

    S.spi_err_cnt++;

    _cs_set(S.inflight, GPIO_PIN_SET);
    (void)HAL_SPI_DMAStop(hspi);

    S.dma_busy = 0u;
    S.inflight = PL_IF_NONE;
    S.xfer_len = 0u;

    S.streak_pending = 0u;
    S.kick_req = 1u;
}

void imu_pipeline_dma_attach(IMU_BMI088_HALCtx *hal, IMU_BMI088_FE *fe)
{
    uint32_t ps = _irq_save();

    S.dma_mode = 0u;
    S.hal = hal;
    S.fe  = fe;

    S.dma_busy = 0u;
    S.kick_req = 0u;
    S.inflight = PL_IF_NONE;
    S.inflight_ts = 0u;

    S.acc_pending_cnt = 0u;
    S.gyr_pending_cnt = 0u;
    S.acc_pending_ts  = 0u;
    S.gyr_pending_ts  = 0u;

    S.xfer_len = 0u;
    S.spi_err_cnt = 0u;
    S.stats.dma_timeout_cnt = 0u;
    S.dma_timeout_win_t0_ms = 0u;
    S.dma_timeout_win_cnt = 0u;

    S.streak_pending = 0u;
    S.streak_next    = 0u;
    g_streak = 0u;

    if (S.acc_data) S.acc_data->tail = S.acc_data->head;
    if (S.gyr_data) S.gyr_data->tail = S.gyr_data->head;

    _irq_restore(ps);

    if (hal && hal->hspi) {
        (void)HAL_SPI_DMAStop(hal->hspi);
        HAL_GPIO_WritePin(hal->acc_cs_port, hal->acc_cs_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(hal->gyr_cs_port, hal->gyr_cs_pin, GPIO_PIN_SET);
    }

    S.dma_mode = (hal && hal->hspi && fe) ? 1u : 0u;

    imu_pipeline_dma_poll();
}

