#ifndef MS56XX_SPI_H
#define MS56XX_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// 通用命令
#define MS5611_CMD_RESET      0x1E
#define MS5611_CMD_ADC_READ   0x00
#define MS5611_CMD_D1         0x40  // 压力转换基址
#define MS5611_CMD_D2         0x50  // 温度转换基址
#define MS5611_CMD_PROM_READ  0xA0  // + (index<<1)

// OSR（分辨率/转换时间）
typedef enum {
  MS5611_OSR_256 = 0,
  MS5611_OSR_512,
  MS5611_OSR_1024,
  MS5611_OSR_2048,
  MS5611_OSR_4096
} ms5611_osr_t;

// 变体（引脚/命令兼容）
typedef enum {
  MSx_5611 = 0,
  MSx_5607 = 1
} ms56xx_variant_t;

// 设备句柄
typedef struct {
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port; 
  uint16_t cs_pin;

  uint16_t C[8];      // PROM 系数 C0..C7（C7 低 4bit 为 CRC）
  uint32_t D1_raw;    // 压力 ADC 24-bit
  uint32_t D2_raw;    // 温度 ADC 24-bit
  int32_t  temp_centi;// 0.01°C
  int32_t  press_pa;  // Pa（注意：0.01 mbar == 1 Pa）
  float    qnh_pa;    // 绝对高度参考气压（QNH），默认 101325 Pa
  int32_t  p0_pa;     // 相对高度基准压力（起飞零点）
  uint8_t  p0_valid;  // 1=已设置 P0 基准


  ms56xx_variant_t variant; // 自动检测后填入
  ms5611_osr_t     osr;     // 当前 OSR

  uint8_t  phase;       // 轮询状态机：0 idle,1 wait D1,2 wait D2
  uint32_t t_start_ms;  // 本次转换起始 tick
} ms5611_t;

// 初始化（复位 + 读 PROM + 自动探测变体）。
HAL_StatusTypeDef ms5611_init_spi(ms5611_t *dev,
                                  SPI_HandleTypeDef *hspi,
                                  GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                  ms5611_osr_t osr);

// 轮询式采集：在主循环高频调用；返回 true 表示有新数据（*p_pa/*p_t_centi 有效）
bool ms5611_poll_spi(ms5611_t *dev, int32_t *p_pa, int32_t *p_t_centi);

// 一次性阻塞读取一组（内部做两次转换并等待）
HAL_StatusTypeDef ms5611_read_once_spi(ms5611_t *dev, int32_t *p_pa, int32_t *p_t_centi);

// 运行时切换 OSR（会在下一次转换生效）
void ms5611_set_osr(ms5611_t *dev, ms5611_osr_t osr);

// 海拔换算（QNH 一般 101325Pa 或地面站广播值）
float ms5611_altitude_m(int32_t press_pa, float qnh_pa);

// —— 新增：QNH/零点/便捷高度 API ——
void ms56xx_set_qnh(ms5611_t *dev, float qnh_pa);
void ms56xx_set_p0(ms5611_t *dev, int32_t p0_pa);
float ms56xx_altitude_abs(const ms5611_t *dev, int32_t press_pa);
float ms56xx_altitude_rel(const ms5611_t *dev, int32_t press_pa);
float ms56xx_qnh_from_known_alt(int32_t press_pa, float known_alt_m);


// PROM CRC4 校验（返回 true 表示通过）
bool ms5611_check_prom_crc4(const ms5611_t *dev);

// —— 自动探测/强制选择 接口 ——
// 以当前一组 D1/D2（若无则阻塞采样一组）分别按 5611/5607 公式计算，
// 选落在 normal_min~max 范围内且更接近 expected_pa 的变体。
void ms56xx_auto_detect(ms5611_t *dev,
                        int32_t expected_pa /*=101325*/,
                        int32_t normal_min  /*=30000*/,
                        int32_t normal_max  /*=110000*/);

// 手动强制设置（会立即影响后续计算）
static inline void ms56xx_force_variant(ms5611_t *dev, ms56xx_variant_t v){ dev->variant = v; }

// 便于打印
const char* ms56xx_variant_str(const ms5611_t *dev);

#ifdef __cplusplus
}
#endif

#endif // MS56XX_SPI_H
