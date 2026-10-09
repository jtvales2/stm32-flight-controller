/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdarg.h>
#include "sbus.h"

#include "imu_bmi088_bus.h"
#include "imu_bus_hal.h"
#include "imu_bmi088_frontend.h"
#include "fusion_mahony.h"
#include "imu_bringup.h"
#include "bmi08x.h"
#include "bmi088.h"      // bmi088_configure_data_synchronization(...)
#include "ringbuf_spsc.h"
#include "timebase.h"  // 寮曞叆鏃堕棿鍩哄噯鍔熻兘
#include "pipeline.h"
#include "sync_pair.h"
#include "autocal.h"
#include "fc_baro.h"
#include "mixer.h"
#include "motors.h"
#include "fc_core.h"
#include "fc_evt.h"
#include "log_uart.h"
#include "fc_context.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ==== App 鍙傛暟锛堥泦涓叆鍙ｏ級====
// 鎵撳嵃棰戠巼
#ifndef PRINT_EVERY_MS
#define PRINT_EVERY_MS        40     // 25Hz 鐘舵€佽
#endif

// 瀹氫箟鐜舰闃熷垪鐨勫閲?
#define RB_CAP_DATA 64    // 鏁版嵁鐜閲?

// flush/fuse 鐨勬壒閲忎笂闄?
#define FUSE_MAX_PER_LOOP     8      // 姣忔涓诲惊鐜渶澶氳瀺鍚?4 瀵?

// IRQ 缁熻绐楀彛
#define IRQ_STATS_WINDOW_MS   1000u

// 涓插彛鏃ュ織缂撳啿锛堜笌 DMA 涓€鑷达級
#define LOG_LINE_MAX          160
#define UART_DMA_BUF_SZ       192

#ifndef VOFA_UART
#define VOFA_UART huart1         // 涓婁綅鏈鸿緭鍑虹鍙?
#endif

// ===== gyro-driven: 涓㈠け鈥滈檧铻烘牱鏈€濈粺璁￠槇鍊?=====
#define FC_MISS_WIN_MS               50u   // 缁熻绐楀彛 50ms
#define FC_MISS_THRESH_SAMPLES       10u   // 50ms 鍐呬涪 >10 涓?gyro 鏍锋湰 => degraded
#define FC_MISS_HARDSTOP_SAMPLES     30u   // 50ms 鍐呬涪 >30 涓?gyro 鏍锋湰 => hardstop

// ===== 鎺у埗寤惰繜涓婇檺锛坙ag cap锛?====
// 瓒呰繃 soft锛氳繘鍏ヨ拷璧舵€侊紙鍙瀺鍚堬紝涓嶉€愬抚璺戞帶鍒讹紱鏈€鍚庣敤鏈€鏂版牱鏈?鍚堝苟dt璺戜竴娆★級
// 瓒呰繃 hard锛氳涓虹郴缁熶弗閲嶄笉瀹炴椂锛岀洿鎺ヨЕ鍙?IMU hard stop
#define FC_CTRL_LAG_SOFT_SEC   0.005f   // 5ms锛堝缓璁?3~8ms锛?
#define FC_CTRL_LAG_HARD_SEC   0.020f   // 20ms锛堝缓璁笌 fc 鐨?IMU failsafe 涓€鑷达級

// ===== gyro 鎵瑰鐞嗛绠楋紙鎸夋按浣嶅姩鎬佽皟鑺傦級=====
#ifndef FC_GYRO_BUDGET_LOW
#define FC_GYRO_BUDGET_LOW    64
#endif
#ifndef FC_GYRO_BUDGET_MID
#define FC_GYRO_BUDGET_MID    96
#endif
#ifndef FC_GYRO_BUDGET_HIGH
#define FC_GYRO_BUDGET_HIGH   128
#endif
#ifndef FC_GYRO_BUDGET_PANIC
#define FC_GYRO_BUDGET_PANIC  192
#endif

// ===== panic锛氱垎浠撴椂灏戝仛鏉備簨锛堥檷杞斤級=====
#ifndef FC_PANIC_HOLD_MS
#define FC_PANIC_HOLD_MS        200u
#endif
#ifndef FC_PANIC_SKIP_LOG_POLL_DIV
#define FC_PANIC_SKIP_LOG_POLL_DIV  4u
#endif

// ESC 娌归棬鏍″噯妯″紡锛?=鍚敤鏍″噯锛?=姝ｅ父椋炶
#define ESC_CALIB_MODE   0
#define MOTOR_TEST_MODE  0   // 鍏堝紑娴嬭瘯锛岀敤瀹屾敼鍥?0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static IMU_BMI088_Bus bus;
static IMU_BMI088_FE  fe;
Mahony         mah;

/* 鏁版嵁鐜紙TS->鏁版嵁锛?*/
static stamped_vec3_t acc_buf[RB_CAP_DATA], gyr_buf[RB_CAP_DATA];
static rb_vec_t       acc_rb, gyr_rb;

static volatile uint8_t uart_dma_busy = 0;
static uint8_t uart_dma_buf[UART_DMA_BUF_SZ];

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  // 璁?log_uart 鐨?DMA/IT 鍙戦€佺姸鎬佹満寰€鍓嶈蛋
  log_uart_on_tx_cplt(huart);

  // 浣犲師鏉?VOFA 鐨?busy 鏍囧織
  if (huart == &VOFA_UART) uart_dma_busy = 0;
}
volatile float g_gyr_dps_latest[3] = {0};

volatile uint32_t g_exti_acc = 0, g_exti_gyr = 0;
volatile uint32_t g_spi1_done = 0, g_spi1_err = 0;

volatile uint8_t g_imu_drdy_pending = 0;
static uint32_t t_done = 0;
static uint32_t last_done = 0;

static uint32_t s_miss_total = 0;
static uint32_t s_miss_win_t0_ms = 0;
static uint32_t s_miss_win_cnt = 0;
static uint8_t  s_ctrl_degraded = 0;
static uint8_t  s_ctrl_hardstop = 0;
static uint32_t s_miss_warn_last_ms = 0;

static float    s_gyr_Ts_sec      = 0.001f;   // 鏈熸湜闄€铻哄懆鏈?绉?锛岀敤浜庝及绠椾涪鏍锋湰

static uint32_t sp_err_cnt = 0;
static uint32_t sp_ts_order_cnt = 0;
static uint32_t sp_err_last_ms = 0;

static uint32_t s_lag_soft_hits = 0;
static uint32_t s_lag_hard_hits = 0;

// 璁板綍涓婁竴娆＄殑绱 drop锛堢敤浜庣畻 delta锛?
static uint32_t s_last_gyr_merge_drop = 0;

// ===== 1Hz 鎵撳嵃鐢細绱 drop 鐨勫熀绾匡紙鐢ㄤ簬绠?delta锛?====
static uint32_t s_1hz_last_mergeA = 0;
static uint32_t s_1hz_last_mergeG = 0;
static uint32_t s_1hz_last_sweepA = 0;
static uint32_t s_1hz_last_sweepG = 0;

// ===== pipeline 绱 drop 鐨勨€滄竻闆跺熀绾库€濓紙鐢ㄤ簬鎶婄疮璁℃€绘暟鏄剧ず鎴愪粠0寮€濮嬶級=====
static uint32_t s_base_mergeA = 0;
static uint32_t s_base_mergeG = 0;
static uint32_t s_base_sweepA = 0;
static uint32_t s_base_sweepG = 0;

static uint32_t s_panic_until_ms = 0;   // >now 鍒欏浜?panic
static uint32_t s_loop_ctr = 0;         // 鐢ㄤ簬 panic 涓嬪仛闄嶉

static uint32_t s_irq_stats_t0 = 0;   // DWT 棰戠巼缁熻绐楀彛璧风偣锛堝彲鍦?init 瀵归綈锛?

static fc_baro_out_t s_baro_out;
static uint32_t s_baro_frames = 0;
static uint32_t s_baro_last_ms = 0;
static fc_baro_out_t s_baro_last;   // 鍙€夛細淇濆瓨鏈€鍚庝竴甯у唴瀹?

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int  vofa_fw_send_line(const char *tag, const char *fmt, ...);

static void baseline_align(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*int fputc(int ch, FILE *f)
{
    while (uart_dma_busy || VOFA_UART.gState != HAL_UART_STATE_READY) {  }
    if (ch == '\n') { uint8_t cr = '\r'; HAL_UART_Transmit(&VOFA_UART, &cr, 1, 1000); }
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&VOFA_UART, &c, 1, 1000);
    return ch;
}*/

static inline void baro_pump(void)
{
    if (fc_baro_service_sched(&s_baro_out)) {
        // 缁熻锛氭湁鏂版牱鏈?
        s_baro_frames++;
        s_baro_last_ms = HAL_GetTick();
        s_baro_last = s_baro_out;

        // 鍠傜粰 FC
        fc_core_baro_feed(s_baro_out.valid, s_baro_out.ts_ticks,
                          s_baro_out.press_pa, s_baro_out.temp_centi,
                          s_baro_out.alt_rel_m, s_baro_out.vz_mps);
    }
}

#ifndef FC_BARO_PREARM_ZERO_MS
#define FC_BARO_PREARM_ZERO_MS          1000u
#endif

#ifndef FC_BARO_PREARM_ZERO_ALT_ERR_M
#define FC_BARO_PREARM_ZERO_ALT_ERR_M   0.20f
#endif

#ifndef FC_BARO_PREARM_ZERO_GYR_MAX_DPS
#define FC_BARO_PREARM_ZERO_GYR_MAX_DPS 0.80f
#endif

static uint8_t baro_prearm_static_ok(void)
{
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    imu_get_gyro_dps(&gx, &gy, &gz);

    float g2 = gx * gx + gy * gy + gz * gz;
    return (uint8_t)(g2 <= (FC_BARO_PREARM_ZERO_GYR_MAX_DPS *
                            FC_BARO_PREARM_ZERO_GYR_MAX_DPS));
}

static void baro_prearm_zero_service(void)
{
    static uint32_t last_try_ms = 0u;
    static uint8_t zeroed = 0u;

    uint8_t gate_ok = (uint8_t)(
        !s.arm.armed &&
        !s.rc.arm_sw &&
        s.rc.thr_low &&
        s.rc.stable &&
        s.baro.valid
    );

    if (!gate_ok) {
        zeroed = 0u;
        return;
    }

    uint32_t now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - last_try_ms) < FC_BARO_PREARM_ZERO_MS) {
        return;
    }
    last_try_ms = now_ms;

    if (zeroed && fabsf(s.baro.alt_rel_m) < FC_BARO_PREARM_ZERO_ALT_ERR_M) {
        return;
    }

    if (!baro_prearm_static_ok()) {
        return;
    }

    float z_before = s.baro.alt_rel_m;
    int32_t p_before = s.baro.press_pa;
    int32_t t_before = s.baro.temp_centi;

    if (fc_baro_zero_to_current()) {
        uint32_t ts = now_ticks();
        fc_core_baro_feed(1u, ts, p_before, t_before, 0.0f, 0.0f);

        s_baro_last.ts_ticks = ts;
        s_baro_last.valid = 1u;
        s_baro_last.press_pa = p_before;
        s_baro_last.temp_centi = t_before;
        s_baro_last.alt_rel_m = 0.0f;
        s_baro_last.vz_mps = 0.0f;
        s_baro_last_ms = now_ms;

        fc_evt_push(FC_EVT_BARO_ZERO,
                    (int16_t)(z_before * 100.0f),
                    (int16_t)(p_before & 0xFFFF),
                    (int16_t)((uint32_t)p_before >> 16),
                    0);
        zeroed = 1u;
    }
}

int vofa_fw_send_line(const char *tag, const char *fmt, ...)
{
	static char line[LOG_LINE_MAX];
    int n = 0;
    if (tag && tag[0]) {
        n = snprintf(line, sizeof(line), "%s: ", tag);
        if (n < 0) return n;
    }
    va_list ap; va_start(ap, fmt);
    int m = vsnprintf(line + n, sizeof(line) - n, fmt, ap);
    va_end(ap);
    if (m < 0) return m; n += m;
    if (n >= (int)sizeof(line) - 2) n = sizeof(line) - 2;
    line[n++] = '\n'; line[n] = 0;

    if (VOFA_UART.gState != HAL_UART_STATE_READY || uart_dma_busy)
        return -11;                         // 蹇欙紝鐩存帴涓?

    memcpy(uart_dma_buf, line, n);
    uart_dma_busy = 1;
    if (HAL_UART_Transmit_DMA(&VOFA_UART, uart_dma_buf, n) == HAL_OK)
        return n;
    uart_dma_busy = 0;
    return -2;
}

// 鍙粰 VOFA 鐪嬪Э鎬侊細绾?CSV锛屼笉甯?tag
static void vofa_send_att_csv(float roll_deg, float pitch_deg, float yaw_deg)
{
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "%.2f,%.2f,%.2f\r\n",
                     roll_deg, pitch_deg, yaw_deg);
    if (n > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, (uint16_t)n, 100);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t ts = now_ticks();

    if (GPIO_Pin == INIT1_Pin) {
        g_exti_acc++;
        imu_drdy_on_acc(ts);
    } else if (GPIO_Pin == INIT3_Pin) {
        g_exti_gyr++;
        imu_drdy_on_gyr(ts);
    }

    //涓嶈鍦?EXTI 閲岃皟鐢?imu_pipeline_dma_poll()
}

static void freq_quick_check_timebase(void)
{
    (void)now_ticks();
    const IMU_PipelineStats *s = imu_pipeline_stats();

    // 瀵归綈鍒颁笅涓€娆?ACC 娌匡細杩欓噷涔熷埆骞茬瓑
    uint32_t a_sync = s->acc_irq_cnt;
    while (s->acc_irq_cnt == a_sync) {
        imu_pipeline_dma_poll();  // <<< 鍔犺繖涓?
    }

    uint32_t t0 = now_ticks();
    uint32_t a0 = s->acc_irq_cnt;
    uint32_t g0 = s->gyr_irq_cnt;

    // 1 绉掔獥鍙ｏ細鍒?busy wait锛屾寔缁湇鍔?DMA
    while ((uint32_t)(now_ticks() - t0) < 1000000u) {
        imu_pipeline_dma_poll();  // <<< 鍔犺繖涓?
    }

    uint32_t a1 = s->acc_irq_cnt;
    uint32_t g1 = s->gyr_irq_cnt;

    // printf(...) 鎸夐渶鎵撳紑
}

/* === 1s 鍧囧€兼祴閫?+ 鍘绘姈鍙鍖栵紙浣跨敤 pipeline 鐨勭粺璁★級 === */
static void irq_stats_now(void)
{
	const IMU_PipelineStats *s = imu_pipeline_stats();
	
	float acc_sec = (s->acc_dt_sum) ? ticks_to_sec(s->acc_dt_sum) : 0.0f;
  float gyr_sec = (s->gyr_dt_sum) ? ticks_to_sec(s->gyr_dt_sum) : 0.0f;

  float acc_hz_mean = (s->acc_dt_n && acc_sec > 0.0f) ? ((float)s->acc_dt_n / acc_sec) : 0.0f;
  float gyr_hz_mean = (s->gyr_dt_n && gyr_sec > 0.0f) ? ((float)s->gyr_dt_n / gyr_sec) : 0.0f;

  float acc_min_us = 1e6f * ticks_to_sec(s->min_acc_irq_dt == 0xFFFFFFFFu ? 0 : s->min_acc_irq_dt);
  float gyr_min_us = 1e6f * ticks_to_sec(s->min_gyr_irq_dt == 0xFFFFFFFFu ? 0 : s->min_gyr_irq_dt);

    /* 2) 鎵撳嵃瑙傛祴鍊硷紙鍒嗛€氶亾锛?*/
    /*vofa_fw_send_line("irq_stats",
        "ACC mean=%.2f Hz, min=%.2fus | GYR mean=%.2f Hz, min=%.2fus",
        acc_hz_mean, acc_min_us, gyr_hz_mean, gyr_min_us);*/

    /* 3) 鍙鍖栧綋鍓嶉槇鍊硷紙寰锛夛紝鐩磋鐪嬫槸鍚?> ACC 鐨?min */
    /*vofa_fw_send_line("debounce_us",
        "ACC_thr=%.2fus, GYR_thr=%.2fus",
        1e6f * ticks_to_sec(imu_pipeline_acc_deglitch_ticks()),
        1e6f * ticks_to_sec(imu_pipeline_gyr_deglitch_ticks()));*/

    /* 4) 娓呯獥鍙ｏ細涓嬩竴绉掗噸鏂扮Н绱?*/
    imu_pipeline_reset_stats_window();
}

// ===== panic helpers =====
static inline void fc_panic_kick(uint32_t now_ms)
{
    uint32_t until = now_ms + FC_PANIC_HOLD_MS;
    if ((int32_t)(until - s_panic_until_ms) > 0) {
        s_panic_until_ms = until;
    }
}

static inline uint8_t fc_in_panic(uint32_t now_ms)
{
    return ((int32_t)(s_panic_until_ms - now_ms) > 0) ? 1u : 0u;
}

// ===== dropped gyro accounting =====
static void fc_note_ticks_dropped(uint32_t dropped)
{
    if (dropped == 0u) return;

    uint32_t now = HAL_GetTick();
    if (now < 2000u) return; // 鍚姩鏈熶笉缁熻

    s_miss_total += dropped;

    if (s_miss_win_t0_ms == 0u) s_miss_win_t0_ms = now;

    if ((uint32_t)(now - s_miss_win_t0_ms) >= FC_MISS_WIN_MS) {
        s_miss_win_t0_ms = now;
        s_miss_win_cnt   = dropped;
    } else {
        s_miss_win_cnt  += dropped;
    }

    if (!s_ctrl_degraded && (s_miss_win_cnt > FC_MISS_THRESH_SAMPLES)) {
        s_ctrl_degraded = 1u;
        fc_panic_kick(now);              // degraded 涔熻Е鍙戜竴娈甸檷杞?
    }

    if (!s_ctrl_hardstop && (s_miss_win_cnt > FC_MISS_HARDSTOP_SAMPLES)) {
        s_ctrl_hardstop = 1u;
        fc_panic_kick(now);              // hardstop 鏇磋闄嶈浇
        fc_evt_push(FC_EVT_IMU_HEALTH,
                    FC_IMU_HEALTH_DROP_HARDSTOP,
                    (int16_t)((s_miss_win_cnt > 32767u) ? 32767u : s_miss_win_cnt),
                    (int16_t)FC_MISS_WIN_MS,
                    (int16_t)(s.arm.armed ? 1 : 0));

        if (s.arm.armed) {
            fc_emergency_stop(FC_FS_IMU);
        }
    }

    // 闄愰鍛婅锛歱anic 鏃朵笉鍋?format 鎵撳嵃
    if ((uint32_t)(now - s_miss_warn_last_ms) >= 1000u) {
        s_miss_warn_last_ms = now;
        if (!fc_in_panic(now)) {
            log_uart_printf("WARN: gyro_drop win=%lu/%ums total=%lu degraded=%u hardstop=%u\r\n",
                            (unsigned long)s_miss_win_cnt,
                            (unsigned)FC_MISS_WIN_MS,
                            (unsigned long)s_miss_total,
                            (unsigned)s_ctrl_degraded,
                            (unsigned)s_ctrl_hardstop);
        }
    }
}

// ===== pipeline drop 璁℃暟蹇収锛堥伩鍏?ISR 姝ｅ湪鏇存柊鏃舵挄瑁傦級=====
static IMU_PipelineStats imu_stats_snapshot(void)
{
    IMU_PipelineStats snap;
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    snap = *imu_pipeline_stats();   // 缁撴瀯浣撴嫹璐濓紙绐楀彛缁熻+drop绱锛?
    if (!prim) __enable_irq();
    return snap;
}

static void delay_with_imu_poll(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - t0) < ms) {
        imu_pipeline_dma_poll();
        sbus_uart6_dma_poll();
			baro_pump();
    }
}

static void baseline_align(void)
{
    // 1) 鍏堥鐑竴灏忔锛岃 DRDY/DMA/gyro_ts 璺戣捣鏉ワ紙閬垮厤 lag_hard 琚惎鍔ㄦ湡姹℃煋锛?
    delay_with_imu_poll(200);

    // 2) 鍙€夛細鎶?warmup 鏈熼棿鍫嗗湪 ring 閲岀殑鏍锋湰涓㈡帀锛堜粠骞插噣闃熷垪寮€濮嬶級
    {
        uint32_t prim = __get_PRIMASK();
        __disable_irq();
        rbv_init(&acc_rb, acc_buf, RB_CAP_DATA);
        rbv_init(&gyr_rb, gyr_buf, RB_CAP_DATA);
        if (!prim) __enable_irq();
    }

    // 3) sync_pair reset锛氶伩鍏嶉甯?dt 鐢ㄥ埌鏃?ts
    imu_sync_pair_reset();
		
		delay_with_imu_poll(5);
    // 4) snapshot锛氫綔涓哄熀绾?
    IMU_PipelineStats st0 = imu_stats_snapshot();

    // ---- 5ms miss 缁熻鐢細merge_drop 鍩虹嚎 ----
    s_last_gyr_merge_drop = st0.gyr_merge_drop;

    // ---- pipeline 鈥滀粠0寮€濮嬫樉绀衡€濈殑鍩虹嚎 ----
    s_base_mergeA = st0.acc_merge_drop;
    s_base_mergeG = st0.gyr_merge_drop;
    s_base_sweepA = st0.acc_sweep_drop;
    s_base_sweepG = st0.gyr_sweep_drop;

    // ---- 1Hz 鎵撳嵃鐢細delta 鍩虹嚎 ----
    s_1hz_last_mergeA = st0.acc_merge_drop;
    s_1hz_last_mergeG = st0.gyr_merge_drop;
    s_1hz_last_sweepA = st0.acc_sweep_drop;
    s_1hz_last_sweepG = st0.gyr_sweep_drop;

    // ---- 1Hz spi_done delta 鍩虹嚎 + 1Hz 瀹氭椂璧风偣
    last_done = g_spi1_done;
    t_done    = HAL_GetTick();

    // ---- 娓呪€滃惎鍔ㄦ湡姹℃煋鈥濈殑杞欢缁熻 ----
    {
        uint32_t now_ms = HAL_GetTick();

        s_miss_total = 0;
        s_miss_win_cnt = 0;
        s_miss_win_t0_ms = now_ms;
        s_ctrl_degraded = 0;
        s_ctrl_hardstop = 0;
        s_miss_warn_last_ms = now_ms;

        sp_err_cnt = 0;
        sp_ts_order_cnt = 0;
        sp_err_last_ms = now_ms;

        s_lag_soft_hits = 0;
        s_lag_hard_hits = 0;

        s_panic_until_ms = 0;
        s_loop_ctr = 0;
    }

    // ---- 娓?pipeline 鐨勨€?s 缁熻绐楀彛鈥濓紙鍧囧€?鏈€灏忛棿闅旈偅濂楋級----
    imu_pipeline_reset_stats_window();

    // ---- 瀵归綈 DWT 棰戠巼缁熻绐楀彛璧风偣锛堟浛浠?while 閲岀殑 static stats_t0锛?---
    s_irq_stats_t0 = now_ticks();
}

// ===== gyro 鎵瑰鐞嗛绠楋細鎸夋按浣嶅姩鎬佽皟鑺?=====
static inline int fc_pick_gyro_budget(uint32_t gsz)
{
    const uint32_t cap = (uint32_t)RB_CAP_DATA;

    if (gsz >= (cap * 7u / 8u)) return FC_GYRO_BUDGET_PANIC; // >=87.5%
    if (gsz >= (cap * 3u / 4u)) return FC_GYRO_BUDGET_HIGH;  // >=75%
    if (gsz >= (cap / 2u))      return FC_GYRO_BUDGET_MID;   // >=50%
    return FC_GYRO_BUDGET_LOW;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	__HAL_TIM_SET_COUNTER(&htim5, 0);
  __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
  HAL_TIM_Base_Start_IT(&htim5);// timebase: TIM5 1MHz free-run + overflow IRQ
	
	log_uart_init(&huart1);
	fc_evt_init();
	log_uart_printf("LOG OK\r\n");
	
	if (fc_baro_init_spi(&hspi3, ms5611_cs_GPIO_Port, ms5611_cs_Pin) != HAL_OK) {
    log_uart_printf("BARO init FAIL\r\n");
	} else {
    log_uart_printf("BARO init OK\r\n");
	}

	/* Rings init (data only) */
	rbv_init(&acc_rb, acc_buf, RB_CAP_DATA);
	rbv_init(&gyr_rb, gyr_buf, RB_CAP_DATA);

  /* pipeline 鍙粦瀹氭暟鎹幆锛圖MA pending -> data ring锛?*/
  imu_pipeline_bind(&acc_rb, &gyr_rb);

  /* IMU + Mahony */
  if (imu_bringup_init(&bus, &fe) != 0) { Error_Handler(); }
	imu_pipeline_dma_attach((IMU_BMI088_HALCtx*)bus.user, &fe);

	IMU_BMI088_Status st; IMU_BMI088_FE_GetStatus(&fe, &st);
	imu_pipeline_set_deglitch_by_fs(st.runtime_acc_sample_hz, st.runtime_gyr_sample_hz);
	
	s_gyr_Ts_sec = (st.runtime_gyr_sample_hz > 1.0f) ? (1.0f / st.runtime_gyr_sample_hz) : 0.001f;
	
	/* 鎶婂墠绔?+ Mahony + 鏁版嵁鐜氦缁欏悓姝?閰嶅妯″潡 */
  imu_sync_pair_bind(&fe, &mah, &acc_rb, &gyr_rb);
  imu_sync_pair_set_fs(st.runtime_acc_sample_hz, st.runtime_gyr_sample_hz);

	/* 棰勭儹 DWT 骞跺仛 1s 棰戠巼蹇锛堟鏃?DRDY 宸插紑鍚級 */
  (void)now_ticks();
  delay_with_imu_poll(10);
  freq_quick_check_timebase();
	irq_stats_now();
	
  MahonyConfig mc = {
    .mode = MAHONY_MODE_6DOF,
    .kp_acc = 2.0f, .kp_mag = 0.0f, .ki = 0.03f,
    .acc_g_min = 0.7f, .acc_g_max = 1.3f, .acc_gray_k = 2.0f,
    .mag_ref_uT = 45.0f, .mag_uT_min = 20.0f, .mag_uT_max = 70.0f, .mag_gray_k = 1.5f,
    .dt_min = 1e-4f, .dt_max = 0.05f
  };
  mahony_init(&mah, &mc, NULL);
	
	imu_autocal_init(&fe, &mah);

  /* 鍚姩鐢垫満 PWM锛屽叏閮?1ms 闈欐 */
  motors_init();
  delay_with_imu_poll(10);
	
	HAL_TIM_Base_Start_IT(&htim2);
	baseline_align();

#if ESC_CALIB_MODE
  motors_set_armed(1);

  float m[4];

  // 1) 鍏堟寔缁緭鍑烘渶澶ф补闂ㄤ竴娈佃冻澶熼暱鐨勬椂闂?
  m[0]=m[1]=m[2]=m[3]=1.0f;   // 2000us
  motors_write(m);
  HAL_Delay(12000);           // 寤鸿 10~15s锛屽埆 4s 澶揣

  // 2) 鍐嶈緭鍑烘渶灏忔补闂?
  m[0]=m[1]=m[2]=m[3]=0.0f;   // 1000us
  motors_write(m);
  HAL_Delay(8000);

  motors_set_armed(0);
  while (1) { HAL_Delay(1000); }
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t print_ms = HAL_GetTick();
	uint32_t sweep_ms = HAL_GetTick();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		#if MOTOR_TEST_MODE
    // === 鐢垫満缂栧彿娴嬭瘯妯″紡锛氫笉璧?IMU銆佷笉璧?PID銆佷笉璧?mixer ===
    static uint8_t  idx = 0;
    static uint32_t last_switch_ms = 0;
    float m[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    uint32_t now = HAL_GetTick();
    if (now - last_switch_ms > 2000) {
        last_switch_ms = now;
        idx = (idx + 1) & 0x03;
    }
    m[idx] = 0.20f;
    motors_write(m);
    continue;
#endif
		
		imu_pipeline_dma_poll();
		sbus_uart6_dma_poll();
		baro_pump();
		fc_core_service_async();
		baro_prearm_zero_service();
		
		uint32_t now_ms = HAL_GetTick();
		uint32_t gsz0   = (uint32_t)rbv_size(&gyr_rb);
		int budget      = fc_pick_gyro_budget(gsz0);

		/* 姘翠綅瑙﹀彂 panic锛氱垎浠撹竟缂樺氨閿佸瓨涓€娈垫椂闂?*/
		if (gsz0 >= ((uint32_t)RB_CAP_DATA * 7u / 8u)) {
			fc_panic_kick(now_ms);
		}
		uint8_t panic = fc_in_panic(now_ms);
		
		if (panic && budget < FC_GYRO_BUDGET_HIGH) {
			budget = FC_GYRO_BUDGET_PANIC;
		}
		
		float dt = 0.0f;
		float gyr[3];
		
		/* 杩借刀鎬侊細绱 dt锛岀敤鏈€鏂版牱鏈渶鍚庡彧璺戜竴娆℃帶鍒?*/
		uint8_t catching_up = 0;
		float   dt_acc = 0.0f;
		float   last_gyr[3] = {0};
		uint8_t have_last = 0;

		int eaten = 0;
		while (budget-- > 0) {
      if (((eaten++) & 7) == 0) {
        imu_pipeline_dma_poll();
        sbus_uart6_dma_poll();
      }
			
			if ((eaten & 31) == 0) {     // 姣?32 娆¤ˉ涓€娆★紙鈮堜笉浼氬お棰戠箒锛?
        baro_pump();
        fc_core_service_async();
			}
			
      int r = imu_sync_pair_step_one(gyr, &dt);
      if (r == IMU_SP_NO_DATA) break;

      if (r < 0) {
        sp_err_cnt++;
        if (r == IMU_SP_E_TS_ORDER) sp_ts_order_cnt++;

        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - sp_err_last_ms) >= 1000u) {
            sp_err_last_ms = now;
            if (!panic) {
                log_uart_printf("WARN: sync_pair err=%lu ts_order=%lu\r\n",
                                (unsigned long)sp_err_cnt,
                                (unsigned long)sp_ts_order_cnt);
            }
        }
        continue;
      }  

      // dt 浼扮畻涓㈡牱鏈?
      if (s_gyr_Ts_sec > 0.0f && dt > 1.5f * s_gyr_Ts_sec) {
        uint32_t miss = (uint32_t)(dt / s_gyr_Ts_sec + 0.5f);
        if (miss > 0) miss -= 1;
        fc_note_ticks_dropped(miss);
      }

      uint32_t g_ts = imu_get_gyro_ts();
      if (g_ts != 0u) {
        float lag = ticks_to_sec((uint32_t)(now_ticks() - g_ts));

        if (lag > FC_CTRL_LAG_HARD_SEC) {
            s_lag_hard_hits++;
            fc_panic_kick(HAL_GetTick());
            panic = 1u;

            fc_core_step(gyr, FC_CTRL_LAG_HARD_SEC + 0.001f);
            catching_up = 0;
            dt_acc = 0.0f;
            break;
        }

        if (lag > FC_CTRL_LAG_SOFT_SEC) {
            if (!catching_up) s_lag_soft_hits++;
            catching_up = 1;
            dt_acc += dt;
            last_gyr[0] = gyr[0];
            last_gyr[1] = gyr[1];
            last_gyr[2] = gyr[2];
            have_last = 1;
            continue;
        }
      }

      // 涓嶇 g_ts 鏄惁鏈夋晥锛屽彧瑕佹病 continue/break锛屽氨瑕佹甯歌窇鎺у埗
      if (catching_up) {
        dt_acc += dt;
        fc_core_step(gyr, dt_acc);
        catching_up = 0;
        dt_acc = 0.0f;
      } else {
        fc_core_step(gyr, dt);
      }
		}
		uint32_t gsz_now = (uint32_t)rbv_size(&gyr_rb);

		/* 濡傛灉棰勭畻鑰楀敖浣嗕粛鍦ㄨ拷璧舵€侊細鑷冲皯鐢ㄦ渶鏂版牱鏈窇涓€娆♀€滃悎骞?dt鈥濈殑鎺у埗 */
		if (catching_up && have_last) {
			fc_core_step(last_gyr, dt_acc);
		}
		
		// 3) 浣庨鑳屽帇娓呴亾澶?
		if ((uint32_t)(HAL_GetTick() - sweep_ms) >= 5u) {
			sweep_ms += 5u;
			//merge_drop锛氭棤鏉′欢姣?5ms 缁熻锛堥伩鍏?panic 婕忚锛?
			IMU_PipelineStats st = imu_stats_snapshot();
			uint32_t d_gyr_merge = st.gyr_merge_drop - s_last_gyr_merge_drop;
			s_last_gyr_merge_drop = st.gyr_merge_drop;
			if (d_gyr_merge) fc_note_ticks_dropped(d_gyr_merge);

			//sweep锛氭寜浣犵殑绛栫暐 gating锛坧anic 鏃舵洿淇濆畧锛?
			if (!panic || (gsz_now >= ((uint32_t)RB_CAP_DATA * 7u / 8u))) {
        uint32_t da = 0, dg = 0;
        (void)imu_sweep_backlog(&da, &dg);
        if (dg) fc_note_ticks_dropped(dg);
			}
		}


    if (!panic) {
      fc_core_log_poll();
      log_uart_poll();
		} else {// panic锛氬彧淇濈暀鏈€蹇呰鐨勪覆鍙ｅ嚭闃燂紙閬垮厤鏃ュ織ring鐖嗭級锛屽苟闄嶉
      if ((++s_loop_ctr % FC_PANIC_SKIP_LOG_POLL_DIV) == 0u) {
        log_uart_poll();
			}// fc_acro_log_poll() 鍏堜笉璺戯紙閫氬父閲岄潰浼氬仛鏍煎紡鍖?鎵撳寘锛?
		}
		
		/* ===== 5) 1Hz锛氶獙鏀舵墦鍗?===== */
		while (!panic && (uint32_t)(HAL_GetTick() - t_done) >= 1000u){
			t_done += 1000;

      uint32_t now_done = g_spi1_done;

      IMU_PipelineStats st = imu_stats_snapshot();
			
			uint32_t tot_mergeA = st.acc_merge_drop - s_base_mergeA;
			uint32_t tot_mergeG = st.gyr_merge_drop - s_base_mergeG;
			uint32_t tot_sweepA = st.acc_sweep_drop - s_base_sweepA;
			uint32_t tot_sweepG = st.gyr_sweep_drop - s_base_sweepG;

			uint32_t d_mergeA = st.acc_merge_drop - s_1hz_last_mergeA;
			uint32_t d_mergeG = st.gyr_merge_drop - s_1hz_last_mergeG;
			uint32_t d_sweepA = st.acc_sweep_drop - s_1hz_last_sweepA;
			uint32_t d_sweepG = st.gyr_sweep_drop - s_1hz_last_sweepG;

			s_1hz_last_mergeA = st.acc_merge_drop;
			s_1hz_last_mergeG = st.gyr_merge_drop;
			s_1hz_last_sweepA = st.acc_sweep_drop;
			s_1hz_last_sweepG = st.gyr_sweep_drop;

      log_uart_printf("[1Hz] spi_done=%lu (+%lu) | rbA=%u rbG=%u | spi_err=%lu dma_to=%lu | "
			"miss_total=%lu win_miss=%lu deg=%u hard=%u | lag_soft=%lu lag_hard=%lu | "
      "drop_merge(A,G)=%lu,%lu (+%lu,+%lu) | drop_sweep(A,G)=%lu,%lu (+%lu,+%lu)\r\n",
      (unsigned long)now_done,
      (unsigned long)(now_done - last_done),
      (unsigned)rbv_size(&acc_rb),
      (unsigned)rbv_size(&gyr_rb),
      (unsigned long)g_spi1_err,
      (unsigned long)st.dma_timeout_cnt,
      (unsigned long)s_miss_total,
      (unsigned long)s_miss_win_cnt,
      (unsigned)s_ctrl_degraded,
      (unsigned)s_ctrl_hardstop,
      (unsigned long)s_lag_soft_hits,
      (unsigned long)s_lag_hard_hits,
        (unsigned long)tot_mergeA,
			  (unsigned long)tot_mergeG,
      (unsigned long)d_mergeA,
      (unsigned long)d_mergeG,
        (unsigned long)tot_sweepA,
				(unsigned long)tot_sweepG,
      (unsigned long)d_sweepA,
      (unsigned long)d_sweepG
    );
			
			static uint32_t s_baro_frames_last = 0;
			static uint32_t s_baro_hz = 0;
			
			uint32_t d = s_baro_frames - s_baro_frames_last;
s_baro_frames_last = s_baro_frames;
s_baro_hz = d;  // 鍥犱负浣犳槸 1Hz 鎵撳嵃锛屾墍浠?d 灏辨槸 Hz

			uint32_t baro_age = (s_baro_last_ms == 0) ? 0xFFFFFFFFu : (HAL_GetTick() - s_baro_last_ms);
			log_uart_printf(" | baro:Hz=%lu age=%lums v=%u p=%ld alt=%.2f vz=%.2f\r\n",
                (unsigned long)s_baro_hz,
                (unsigned long)baro_age,
                (unsigned)s_baro_last.valid,
                (long)s_baro_last.press_pa,
                s_baro_last.alt_rel_m,
                s_baro_last.vz_mps);

		uint32_t age_us = 0xFFFFFFFFu;
if (s.baro.valid && s.baro.ts_ticks != 0u) {
  age_us = (uint32_t)(now_ticks() - s.baro.ts_ticks);
}

log_uart_printf(" | gates: armed=%u stable=%u alt_sw=%u ever_on=%u baro_v=%u age_us=%lu alt_active=%u\r\n",
  (unsigned)s.arm.armed,
  (unsigned)s.rc.stable,
  (unsigned)s.rc.alt_sw,
  (unsigned)s.air.ever_on,
  (unsigned)s.baro.valid,
  (unsigned long)age_us,
  (unsigned)s.ctrl.alt.active);

{
  sbus_uart6_dma_stats_t sbus_dma;
  sbus_uart6_dma_get_stats(&sbus_dma);
  log_uart_printf(" | sbus_dma: bytes=%lu evt=%lu idle=%lu poll=%lu err=%lu ore=%lu rst=%lu pos=%u\r\n",
                  (unsigned long)sbus_dma.rx_bytes,
                  (unsigned long)sbus_dma.rx_events,
                  (unsigned long)sbus_dma.idle_events,
                  (unsigned long)sbus_dma.poll_events,
                  (unsigned long)sbus_dma.errors,
                  (unsigned long)sbus_dma.overruns,
                  (unsigned long)sbus_dma.restarts,
                  (unsigned)sbus_dma.last_pos);
}



    last_done = now_done;
		}

    /* ===== 6) 姣?1.000s锛欴WT 棰戠巼缁熻绐楀彛锛堜綘鍘熸潵鐨勯偅濂楋級 ===== */
    if (s_irq_stats_t0 == 0) s_irq_stats_t0 = now_ticks();
		if ((uint32_t)(now_ticks() - s_irq_stats_t0) >= 1000000u) {
			s_irq_stats_t0 += 1000000u;
			irq_stats_now();
		}

    /* ===== 7) 浣庨濮挎€佹墦鍗帮紙鍙€夛級 ===== */
    if ((HAL_GetTick() - print_ms) >= PRINT_EVERY_MS) {
        print_ms = HAL_GetTick();
        float r, p, y;
        mahony_get_euler(&mah, &r, &p, &y);
        (void)r; (void)p; (void)y;
        // 浣犺 VOFA 灏卞湪杩欓噷鍙?
    }
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        g_spi1_done++;
        imu_pipeline_dma_on_spi_done(hspi);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        g_spi1_err++;
        imu_pipeline_dma_on_spi_error(hspi);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5) {
        timebase_tim5_overflow_isr();   // hi32++
        return;
    }

    if (htim->Instance == TIM2) {
        fc_core_watchdog_1khz();
        return;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
