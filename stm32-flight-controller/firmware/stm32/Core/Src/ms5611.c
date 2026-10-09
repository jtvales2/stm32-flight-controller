#include "ms5611.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

// ---------------- 片选与基本工具 ----------------
static inline void CS_L(ms5611_t* d){ HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_RESET); }
static inline void CS_H(ms5611_t* d){ HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_SET); }

static uint8_t osr_to_bits(ms5611_osr_t osr){
  switch(osr){
    case MS5611_OSR_256:  return 0x00;
    case MS5611_OSR_512:  return 0x02;
    case MS5611_OSR_1024: return 0x04;
    case MS5611_OSR_2048: return 0x06;
    default:              return 0x08; // 4096
  }
}
static uint32_t osr_time_ms(ms5611_osr_t osr){
  switch(osr){
    case MS5611_OSR_256:  return 1;  // 0.54 ms typ
    case MS5611_OSR_512:  return 2;  // 1.06 ms typ
    case MS5611_OSR_1024: return 3;  // 2.08 ms typ
    case MS5611_OSR_2048: return 5;  // 4.13 ms typ
    default:              return 12; // 4096：给足裕量（>8.2ms）
  }
}

static HAL_StatusTypeDef cmd_only(ms5611_t* d, uint8_t cmd){
  CS_L(d);
  HAL_StatusTypeDef st = HAL_SPI_Transmit(d->hspi, &cmd, 1, 10);
  CS_H(d);
  return st;
}

static HAL_StatusTypeDef reset_chip(ms5611_t* d){ return cmd_only(d, MS5611_CMD_RESET); }

// —— 关键：命令后用 TransmitReceive 发送 dummy clock 读取 ——
static HAL_StatusTypeDef read_prom_word(ms5611_t* d, uint8_t idx, uint16_t* out){
  uint8_t cmd = (uint8_t)(MS5611_CMD_PROM_READ | (idx<<1));
  uint8_t rx[2] = {0}, dummy[2] = {0,0};
  CS_L(d);
  HAL_StatusTypeDef st = HAL_SPI_Transmit(d->hspi, &cmd, 1, 10);
  if (st == HAL_OK) st = HAL_SPI_TransmitReceive(d->hspi, dummy, rx, 2, 10);
  CS_H(d);
  if (st != HAL_OK) return st;
  *out = (uint16_t)((rx[0]<<8) | rx[1]);
  return HAL_OK;
}

static HAL_StatusTypeDef start_conv(ms5611_t* d, uint8_t base_cmd){
  uint8_t cmd = (uint8_t)(base_cmd | osr_to_bits(d->osr));
  return cmd_only(d, cmd);
}

static HAL_StatusTypeDef read_adc24(ms5611_t* d, uint32_t* out){
  uint8_t cmd = MS5611_CMD_ADC_READ;
  uint8_t rx[3] = {0}, dummy[3] = {0,0,0};
  CS_L(d);
  HAL_StatusTypeDef st = HAL_SPI_Transmit(d->hspi, &cmd, 1, 10);
  if (st == HAL_OK) st = HAL_SPI_TransmitReceive(d->hspi, dummy, rx, 3, 10);
  CS_H(d);
  if (st != HAL_OK) return st;
  *out = ((uint32_t)rx[0]<<16) | ((uint32_t)rx[1]<<8) | rx[2];
  return HAL_OK;
}
// ---------------- 公式计算 ----------------
// MS5611：含低温二阶补偿
static void compensate_calc_MS5611(ms5611_t* d, int32_t* p_pa, int32_t* t_centi){
  const int64_t C1 = d->C[1], C2 = d->C[2], C3 = d->C[3];
  const int64_t C4 = d->C[4], C5 = d->C[5], C6 = d->C[6];

  int64_t dT   = (int64_t)d->D2_raw - (int64_t)(C5 << 8);
  int64_t TEMP = 2000 + ((dT * C6) >> 23);                // 0.01°C
  int64_t OFF  = (C2 << 16) + ((C4 * dT) >> 7);
  int64_t SENS = (C1 << 15) + ((C3 * dT) >> 8);

  int64_t T2=0, OFF2=0, SENS2=0;
  if (TEMP < 2000) {
    int64_t Tlow = TEMP - 2000;
    T2    = (dT * dT) >> 31;
    OFF2  = (5 * Tlow * Tlow) >> 1;      // /2
    SENS2 = (5 * Tlow * Tlow) >> 2;      // /4
    if (TEMP < -1500) {
      int64_t Tv = TEMP + 1500;
      OFF2  += 7  * Tv * Tv;
      SENS2 += (11 * Tv * Tv) >> 1;      // /2
    }
  }
  TEMP -= T2; OFF -= OFF2; SENS -= SENS2;

  int64_t P = (((int64_t)d->D1_raw * SENS) >> 21) - OFF;
  P >>= 15; // 0.01 mbar

  if (t_centi) *t_centi = (int32_t)TEMP;
  if (p_pa)    *p_pa    = (int32_t)P;   // 0.01 mbar == 1 Pa
}

// MS5607：常温下一阶补偿（你的温度 ~27℃，足够）；低温二阶可按需补充
static void compensate_calc_MS5607(ms5611_t* d, int32_t* p_pa, int32_t* t_centi){
  const int64_t C1 = d->C[1], C2 = d->C[2], C3 = d->C[3];
  const int64_t C4 = d->C[4], C5 = d->C[5], C6 = d->C[6];

  int64_t dT   = (int64_t)d->D2_raw - (int64_t)(C5 << 8);
  int64_t TEMP = 2000 + ((dT * C6) >> 23);                // 0.01°C
  int64_t OFF  = (C2 << 17) + ((C4 * dT) >> 6);           // 5607 一阶
  int64_t SENS = (C1 << 16) + ((C3 * dT) >> 7);           // 5607 一阶

  // —— 低温二阶补偿（MS5607，按官方应用笔记）——
  int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
  if (TEMP < 2000) { // < 20°C
    int64_t Tlow = TEMP - 2000;            // 0.01°C
    // T2 = 3 * dT^2 / 2^33
    T2    = (3 * (dT * dT)) >> 33;
    // OFF2 = 3 * (TEMP-2000)^2 / 2
    OFF2  = (3 * Tlow * Tlow) >> 1;
    // SENS2 = 5 * (TEMP-2000)^2 / 8
    SENS2 = (5 * Tlow * Tlow) >> 3;
    if (TEMP < -1500) { // < -15°C 追加项
      int64_t Tv = TEMP + 1500;
      OFF2  += 7 * Tv * Tv;
      SENS2 += 4 * Tv * Tv;
    }
  }
  TEMP -= T2; OFF -= OFF2; SENS -= SENS2;

  int64_t P = (((int64_t)d->D1_raw * SENS) >> 21) - OFF;
  P >>= 15; // 0.01 mbar

  if (t_centi) *t_centi = (int32_t)TEMP;
  if (p_pa)    *p_pa    = (int32_t)P;   // 0.01 mbar == 1 Pa
}

static void compensate_calc_variant(ms5611_t* d, int32_t* p_pa, int32_t* t_centi){
  if (d->variant == MSx_5607) compensate_calc_MS5607(d, p_pa, t_centi);
  else                        compensate_calc_MS5611(d, p_pa, t_centi);
}

// ---------------- 对外 API ----------------
HAL_StatusTypeDef ms5611_init_spi(ms5611_t *dev,
                                  SPI_HandleTypeDef *hspi,
                                  GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                  ms5611_osr_t osr){
  memset(dev, 0, sizeof(*dev));
  dev->hspi = hspi;
  dev->cs_port = cs_port;
  dev->cs_pin = cs_pin;
  dev->osr = osr;
  dev->variant = MSx_5611;  // 默认先按 5611
  dev->qnh_pa  = 101325.0f; // 默认海平面气压
  dev->p0_pa   = 0; dev->p0_valid = 0; // 相对高度未设
  CS_H(dev);

  HAL_StatusTypeDef st = reset_chip(dev);
  if(st != HAL_OK) return st;
  HAL_Delay(3); // PROM 上电装载时间

  for(uint8_t i=0;i<8;i++){
    st = read_prom_word(dev, i, &dev->C[i]);
    if(st != HAL_OK) return st;
  }

  if (!ms5611_check_prom_crc4(dev)) {
    return HAL_ERROR;
  }

  // 自动探测：以默认气压 101325Pa、正常区 30~110kPa 判定
  ms56xx_auto_detect(dev, 101325, 30000, 110000);

  dev->phase = 0;
  dev->t_start_ms = 0;
  return HAL_OK;
}

void ms5611_set_osr(ms5611_t *dev, ms5611_osr_t osr){ dev->osr = osr; }

bool ms5611_poll_spi(ms5611_t *dev, int32_t *p_pa, int32_t *p_t_centi){
  uint32_t now = HAL_GetTick();
  switch(dev->phase){
    case 0: // 启动 D1（压力）
      if(start_conv(dev, MS5611_CMD_D1) != HAL_OK) return false;
      dev->t_start_ms = now;
      dev->phase = 1;
      break;
    case 1: // 等待 D1 完成
      if((now - dev->t_start_ms) >= osr_time_ms(dev->osr)){
        if(read_adc24(dev, &dev->D1_raw) != HAL_OK){ dev->phase = 0; break; }
        if(start_conv(dev, MS5611_CMD_D2) != HAL_OK){ dev->phase = 0; break; }
        dev->t_start_ms = now;
        dev->phase = 2;
      }
      break;
    case 2: // 等待 D2 完成，计算&输出
      if((now - dev->t_start_ms) >= osr_time_ms(dev->osr)){
        if(read_adc24(dev, &dev->D2_raw) != HAL_OK){ dev->phase = 0; break; }
        compensate_calc_variant(dev, &dev->press_pa, &dev->temp_centi);
        if(p_pa) *p_pa = dev->press_pa;
        if(p_t_centi) *p_t_centi = dev->temp_centi;
        dev->phase = 0; // 下一轮
        return true;
      }
      break;
  }
  return false;
}

HAL_StatusTypeDef ms5611_read_once_spi(ms5611_t *dev, int32_t *p_pa, int32_t *p_t_centi){
  HAL_StatusTypeDef st = start_conv(dev, MS5611_CMD_D1);
  if(st != HAL_OK) return st;
  HAL_Delay(osr_time_ms(dev->osr));
  st = read_adc24(dev, &dev->D1_raw);
  if(st != HAL_OK) return st;

  st = start_conv(dev, MS5611_CMD_D2);
  if(st != HAL_OK) return st;
  HAL_Delay(osr_time_ms(dev->osr));
  st = read_adc24(dev, &dev->D2_raw);
  if(st != HAL_OK) return st;

  compensate_calc_variant(dev, &dev->press_pa, &dev->temp_centi);
  if(p_pa) *p_pa = dev->press_pa;
  if(p_t_centi) *p_t_centi = dev->temp_centi;
  return HAL_OK;
}

float ms5611_altitude_m(int32_t press_pa, float qnh_pa){
  if(press_pa <= 0 || qnh_pa <= 0) return 0.0f;
  float ratio = (float)press_pa / qnh_pa;
  return 44330.0f * (1.0f - powf(ratio, 0.190294957f)); // ISA baro
}

bool ms5611_check_prom_crc4(const ms5611_t *dev){
  uint16_t prom[8];
  for(int i=0;i<8;i++) prom[i] = dev->C[i];
  uint8_t crc_read = prom[7] & 0x0F;
  prom[7] &= 0xFF00; // 清掉 CRC nibble

  uint16_t crc = 0;
  for(int i=0;i<16;i++){
    uint16_t word = (i%2==0) ? (prom[i>>1] >> 8) : (prom[i>>1] & 0x00FF);
    crc ^= (uint16_t)word;
    for(int b=0;b<8;b++){
      if(crc & 0x8000) crc = (crc << 1) ^ 0x3000; // 多项式左移到位
      else             crc = (crc << 1);
    }
  }
  uint8_t crc_calc = (uint8_t)((crc >> 12) & 0x0F);
  return (crc_calc == crc_read);
}

// —— 新增：QNH / 零点 / 便捷高度 ——
void ms56xx_set_qnh(ms5611_t *dev, float qnh_pa){ dev->qnh_pa = (qnh_pa>0? qnh_pa:101325.0f); }
void ms56xx_set_p0(ms5611_t *dev, int32_t p0_pa){ dev->p0_pa = p0_pa; dev->p0_valid = 1; }
float ms56xx_altitude_abs(const ms5611_t *dev, int32_t press_pa){ return ms5611_altitude_m(press_pa, dev->qnh_pa); }
float ms56xx_altitude_rel(const ms5611_t *dev, int32_t press_pa){ return dev->p0_valid ? ms5611_altitude_m(press_pa, (float)dev->p0_pa) : 0.0f; }
float ms56xx_qnh_from_known_alt(int32_t press_pa, float known_alt_m){
  if(press_pa<=0) return 101325.0f;
  float k = 1.0f - (known_alt_m / 44330.0f);
  float expo = 1.0f / 0.190294957f;
  return ((float)press_pa) / powf(k, expo);
}

// ---------------- 自动探测实现 ----------------
static void calc_with_variant_once(ms5611_t* d, ms56xx_variant_t v,
                                   int32_t* p_pa_out, int32_t* t_centi_out){
  // 以当前 D1/D2 计算，不改变 d->variant 的持久值
  ms56xx_variant_t old = d->variant;
  d->variant = v;
  compensate_calc_variant(d, p_pa_out, t_centi_out);
  d->variant = old;
}

void ms56xx_auto_detect(ms5611_t *dev, int32_t expected_pa, int32_t normal_min, int32_t normal_max){
  // 若还未有原始数据，则阻塞读取一组（按当前 OSR）
  if (dev->D1_raw == 0 || dev->D2_raw == 0) {
    (void)ms5611_read_once_spi(dev, NULL, NULL);
  }

  int32_t p5611=0, t_dummy=0;
  int32_t p5607=0;
  calc_with_variant_once(dev, MSx_5611, &p5611, &t_dummy);
  calc_with_variant_once(dev, MSx_5607, &p5607, NULL);

  // 评分：出界的给巨大惩罚；否则按与期望气压的差值评分
  int32_t s5611 = (p5611 < normal_min || p5611 > normal_max) ? 0x3fffffff : abs(p5611 - expected_pa);
  int32_t s5607 = (p5607 < normal_min || p5607 > normal_max) ? 0x3fffffff : abs(p5607 - expected_pa);

  dev->variant = (s5607 < s5611) ? MSx_5607 : MSx_5611;
}

const char* ms56xx_variant_str(const ms5611_t *dev){
  return (dev->variant == MSx_5607) ? "MS5607" : "MS5611";
}
