#include "fc_log.h"

#include "fc_evt.h"
#include "log_uart.h"
#include "timebase.h"
#include "fc_core.h"
// 仅日志层需要读内部状态（例如 latched failsafe 位）
#include "fc_context.h"

#include <stdio.h>

#define FC_LOG_BUDGET_PER_POLL  8   // 每次最多处理 N 条事件

static uint8_t s_alt_dbg_have = 0u;
static int16_t s_alt_dbg_seq  = 0;
static int16_t s_alt_dbg_z    = 0;
static int16_t s_alt_dbg_z_sp = 0;
static int16_t s_alt_dbg_vz   = 0;

void fc_log_poll(void)
{
  // 50Hz 限频
  static uint32_t last = 0;
  uint32_t now = now_ticks();
  if (last != 0 && (uint32_t)(now - last) < sec_to_ticks(0.02f)) return;
  last = now;

  // drop 计数：快照读取，避免清零时丢增量
  uint32_t drop = fc_evt_take_drop_count();
  if (drop) {
    log_uart_printf("LOG_DROP=%lu\r\n", (unsigned long)drop);
  }

  // 每次最多处理 N 条
  fc_evt_t e;
  int budget = FC_LOG_BUDGET_PER_POLL;
  while (budget-- > 0 && fc_evt_pop(&e)) {
    switch (e.id) {

      case FC_EVT_ARM_STATE: {
        int arm_sw = e.a;
        int armed  = e.b;
        float thr  = (float)e.c / 1000.0f;
        int d = e.d;
        int thr_low  = (d & 1) ? 1 : 0;
        int level_ok = (d & 2) ? 1 : 0;
        int fsbits   = (d >> 2) & 0x7;
        log_uart_printf("ARM_STATE: arm_sw=%d thr=%.2f thr_low=%d level_ok=%d fs=0x%x -> armed=%d\r\n",
                        arm_sw, (double)thr, thr_low, level_ok, fsbits, armed);
      } break;

      case FC_EVT_LEVEL_CAL: {
        float r = (float)e.a / 100.0f;
        float p = (float)e.b / 100.0f;
        log_uart_printf("LEVEL_CAL OK roll=%.2f pitch=%.2f\r\n", (double)r, (double)p);
      } break;

      case FC_EVT_TILT_LATCH: {
        float r  = (float)e.a / 10.0f;
        float p  = (float)e.b / 10.0f;
        float tg = (float)e.c / 100.0f;
        log_uart_printf("TILT_LATCH thr_gate=%.2f r=%.1f p=%.1f\r\n", (double)tg, (double)r, (double)p);
      } break;

      case FC_EVT_AHRS_RESET: {
        float r = (float)e.a / 10.0f;
        float p = (float)e.b / 10.0f;
        log_uart_printf("AHRS_RESET serviced r=%.1f p=%.1f\r\n", (double)r, (double)p);
      } break;

      case FC_EVT_ARM_BLOCK: {
        int st = e.a;
        const char *msg =
          (st == 0) ? "ARM_BLOCK: cleared" :
          (st == 1) ? "ARM_BLOCK: waiting AHRS reset (keep arm=0, thr low, level)" :
                      "ARM_BLOCK: BLOCKED (set arm=0, thr low, level)";
        log_uart_printf("%s\r\n", msg);
      } break;
			
			case FC_EVT_MODE: {
				int mode = e.a;      // 0/1
				int reason = e.b;    // INIT/SW/ARM
				
				const char *m = (mode == 1) ? "ANGLE" : "ACRO";
				const char *r =
					(reason == FC_MODE_REASON_INIT)   ? "INIT" :
				(reason == FC_MODE_REASON_SWITCH) ? "SW"   :
				(reason == FC_MODE_REASON_ARM)    ? "ARM"  : "UNK";
				
				log_uart_printf("MODE=%s(%d) reason=%s\r\n", m, mode, r);
			} break;
			
      case FC_EVT_LEVEL_SW: {
        int16_t flags = e.a;
        int oldv    = (flags & 1) ? 1 : 0;
        int newv    = (flags & 2) ? 1 : 0;
        int arm     = (flags & 4) ? 1 : 0;
        int thr_low = (flags & 8) ? 1 : 0;

        float thr = (float)e.b / 1000.0f;
        float rr  = (float)e.c / 10.0f;
        float pp  = (float)e.d / 10.0f;

        log_uart_printf("LEVEL_SW %d->%d arm=%d thr=%.3f thr_low=%d raw=(%.1f,%.1f)\r\n",
                        oldv, newv, arm, (double)thr, thr_low, (double)rr, (double)pp);
      } break;

      case FC_EVT_LEVEL_CAL_BLOCKED: {
        int16_t flags = e.a;
        int arm     = (flags & 1) ? 1 : 0;
        int thr_low = (flags & 2) ? 1 : 0;
        float thr   = (float)e.b / 1000.0f;
        log_uart_printf("LEVEL_CAL BLOCKED arm=%d thr_low=%d thr=%.3f\r\n",
                        arm, thr_low, (double)thr);
      } break;

      case FC_EVT_YAW_DBG: {
        float thr  = (float)e.a / 1000.0f;
        float gain = (float)e.b / 1000.0f;
        float yi   = (float)e.c / 1000.0f;
        float rc   = (float)e.d / 1000.0f;

        log_uart_printf("YAW_DBG thr=%.3f gain=%.3f yi=%.3f rc=%.3f\r\n",
                        (double)thr, (double)gain, (double)yi, (double)rc);
      } break;

      case FC_EVT_WARN_CTRL2X: {
        uint32_t frame = ((uint32_t)(uint16_t)e.b << 16) | (uint16_t)e.a;
        log_uart_printf("WARN: controller_run twice! frame_id=%lu\r\n",
                        (unsigned long)frame);
      } break;

      case FC_EVT_FAILSAFE: {
        int reason = e.a;      // 1=FS bit, 2=timeout
        int fs     = e.b;
        int lost   = e.c;
        int age_ms = e.d;

        const char *rmsg = (reason == 1) ? "SBUS_FAILSAFE_BIT" : "SBUS_TIMEOUT";
        log_uart_printf("FAILSAFE: %s fs=%d lost=%d age_ms=%d latched=0x%lx\r\n",
                        rmsg, fs, lost, age_ms, (unsigned long)s.fs.latched);
      } break;

      case FC_EVT_PREARM_CAL: {
        if (e.a == FC_PREARM_CAL_DONE) {
          float gx = (float)e.b / 1000.0f;
          float gy = (float)e.c / 1000.0f;
          float gz = (float)e.d / 1000.0f;
          log_uart_printf("PREARM_CAL OK gyro_mean=(%.4f,%.4f,%.4f)dps\r\n",
                          (double)gx, (double)gy, (double)gz);
        }
      } break;

      case FC_EVT_IMU_HEALTH: {
        int reason = e.a;
        const char *r =
          (reason == FC_IMU_HEALTH_GYRO_PTR)   ? "GYRO_PTR" :
          (reason == FC_IMU_HEALTH_GYRO_NAN)   ? "GYRO_NAN" :
          (reason == FC_IMU_HEALTH_GYRO_RANGE) ? "GYRO_RANGE" :
          (reason == FC_IMU_HEALTH_ACC_STALE)  ? "ACC_STALE" :
          (reason == FC_IMU_HEALTH_DT_STALL)   ? "DT_STALL" :
          (reason == FC_IMU_HEALTH_DROP_HARDSTOP) ? "DROP_HARDSTOP" :
                                                 "UNKNOWN";
        log_uart_printf("IMU_HEALTH: %s b=%d c=%d d=%d fs=0x%lx\r\n",
                        r, e.b, e.c, e.d, (unsigned long)s.fs.latched);
      } break;

      case FC_EVT_ALT_ENTER: {
        float thr_mid   = (float)e.a / 1000.0f;
        float thr_hover = (float)e.b / 1000.0f;
        float z         = (float)e.c / 100.0f;
        float vz        = (float)e.d / 100.0f;
        log_uart_printf("ALT_ENTER: thr_mid=%.3f thr_hover=%.3f z=%.2f z_sp=%.2f vz=%.2f\r\n",
                        (double)thr_mid, (double)thr_hover,
                        (double)z, (double)z, (double)vz);
      } break;

      case FC_EVT_ALT_ENTER_BLOCK: {
        const char *r =
          (e.a == FC_ALT_ENTER_BLOCK_RC_UNSTABLE) ? "RC_UNSTABLE" :
          (e.a == FC_ALT_ENTER_BLOCK_VZ)          ? "VZ_NOT_STEADY" :
          (e.a == FC_ALT_ENTER_BLOCK_THR)         ? "THR_NOT_MID" :
          (e.a == FC_ALT_ENTER_BLOCK_LOW_ALT)     ? "LOW_ALT" :
                                                     "UNKNOWN";
        float z   = (float)e.b / 100.0f;
        float vz  = (float)e.c / 100.0f;
        float thr = (float)e.d / 1000.0f;
        log_uart_printf("ALT_ENTER_BLOCK: %s z=%.2f vz=%.2f thr=%.3f\r\n",
                        r, (double)z, (double)vz, (double)thr);
      } break;
      case FC_EVT_ALT_EXIT: {
        const char *r =
          (e.a == FC_ALT_EXIT_BARO_STALE) ? "BARO_STALE" :
          (e.a == FC_ALT_EXIT_SWITCH_OFF) ? "SWITCH_OFF" :
          (e.a == FC_ALT_EXIT_GATE_BAD)   ? "GATE_BAD" :
          (e.a == FC_ALT_EXIT_POGO)       ? "POGO" :
                                            "UNKNOWN";
        float z  = (float)e.b / 100.0f;
        float vz = (float)e.c / 100.0f;
        log_uart_printf("ALT_EXIT: %s z=%.2f vz=%.2f d=%d fs=0x%lx\r\n",
                        r, (double)z, (double)vz, e.d,
                        (unsigned long)s.fs.latched);
      } break;

      case FC_EVT_ALT_DBG: {
        s_alt_dbg_seq  = e.a;
        s_alt_dbg_z    = e.b;
        s_alt_dbg_z_sp = e.c;
        s_alt_dbg_vz   = e.d;
        s_alt_dbg_have = 1u;
      } break;

      case FC_EVT_ALT_DBG2: {
        if (s_alt_dbg_have && s_alt_dbg_seq == e.a) {
          float z          = (float)s_alt_dbg_z / 100.0f;
          float z_sp       = (float)s_alt_dbg_z_sp / 100.0f;
          float vz         = (float)s_alt_dbg_vz / 100.0f;
          float thr_out    = (float)e.b / 1000.0f;
          float manual_thr = (float)e.c / 1000.0f;
          float enter_t    = (float)e.d / 100.0f;

          log_uart_printf("ALT_DBG: z=%.2f z_sp=%.2f vz=%.2f thr_out=%.3f manual=%.3f enter_t=%.2f\r\n",
                          (double)z, (double)z_sp, (double)vz,
                          (double)thr_out, (double)manual_thr, (double)enter_t);
        }
        s_alt_dbg_have = 0u;
      } break;

      case FC_EVT_IMU_DMA_TIMEOUT: {
        log_uart_printf("IMU_DMA_TIMEOUT: win=%d total=%d dt_ms=%d armed=%d fs=0x%lx\r\n",
                        e.a, e.b, e.c, e.d, (unsigned long)s.fs.latched);
      } break;

      case FC_EVT_BARO_ZERO: {
        float z_before = (float)e.a / 100.0f;
        log_uart_printf("BARO_ZERO: z_before=%.2f p=%ld\r\n",
                        (double)z_before, (long)(((uint32_t)(uint16_t)e.c << 16) | (uint16_t)e.b));
      } break;

      case FC_EVT_MOTOR_TEST: {
        const char *act =
          (e.a == FC_MOTOR_TEST_ENTER)  ? "ENTER" :
          (e.a == FC_MOTOR_TEST_SELECT) ? "SELECT" :
          (e.a == FC_MOTOR_TEST_OUTPUT) ? "OUT" :
          (e.a == FC_MOTOR_TEST_EXIT)   ? "EXIT" : "UNK";
        const char *mn =
          (e.b == 0) ? "FR" :
          (e.b == 1) ? "RR" :
          (e.b == 2) ? "RL" :
          (e.b == 3) ? "FL" : "?";
        if (e.a == FC_MOTOR_TEST_EXIT) {
          const char *why =
            (e.c == FC_MOTOR_TOOL_STOP_ARM)   ? "ARM_SW" :
            (e.c == FC_MOTOR_TOOL_STOP_COMBO) ? "COMBO" : "UNK";
          log_uart_printf("MOTOR_TEST: %s m=%d(%s) reason=%s\r\n", act, e.b, mn, why);
        } else {
          float thr = (float)e.c / 1000.0f;
          log_uart_printf("MOTOR_TEST: %s m=%d(%s) thr=%.3f\r\n", act, e.b, mn, (double)thr);
        }
      } break;

      case FC_EVT_ESC_CAL: {
        const char *act =
          (e.a == FC_ESC_CAL_ENTER) ? "ENTER" :
          (e.a == FC_ESC_CAL_HIGH)  ? "HIGH" :
          (e.a == FC_ESC_CAL_LOW)   ? "LOW" :
          (e.a == FC_ESC_CAL_DONE)  ? "DONE" :
          (e.a == FC_ESC_CAL_ABORT) ? "ABORT" : "UNK";
        if (e.a == FC_ESC_CAL_ABORT) {
          const char *why =
            (e.b == FC_MOTOR_TOOL_STOP_ARM)   ? "ARM_SW" :
            (e.b == FC_MOTOR_TOOL_STOP_COMBO) ? "COMBO" : "UNK";
          log_uart_printf("ESC_CAL: %s reason=%s\r\n", act, why);
        } else {
          log_uart_printf("ESC_CAL: %s thr=%.3f hold=%d\r\n",
                          act, (double)((float)e.b / 1000.0f), e.c);
        }
      } break;
      default:
        break;
    }
  }
}


