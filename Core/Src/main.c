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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* ---------- Mode ของระบบ ---------- */
typedef enum {
    MODE_AUTO = 0,
    MODE_AWAY,
    MODE_MANUAL
} SystemMode_t;

/* ---------- Noise Level ---------- */
typedef enum {
    NOISE_LOW = 0,
    NOISE_MED,
    NOISE_HIGH
} NoiseLevel_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---------- GPIO Mapping ---------- */
/* LDR ใช้ ADC1_IN0 ที่ PA0 อยู่แล้วจาก CubeMX */
#define LDR_ADC_CHANNEL       ADC_CHANNEL_0   // PA0

#define MIC_GPIO_Port         GPIOB
#define MIC_Pin               GPIO_PIN_0      // Mic D0

#define US_TRIG_GPIO_Port     GPIOB
#define US_TRIG_Pin           GPIO_PIN_1
#define US_ECHO_GPIO_Port     GPIOB
#define US_ECHO_Pin           GPIO_PIN_2

#define LIGHT_GPIO_Port       GPIOB
#define LIGHT_Pin             GPIO_PIN_10    // LED ไฟห้อง (บน breadboard)

/* ใช้ LD2 บนบอร์ดเป็น Mode LED */
#define LED_GPIO_Port         LD2_GPIO_Port
#define LED_Pin               LD2_Pin        // PA5

/* ---------- ค่าปรับ (Tuning Parameters) ---------- */
#define LDR_TH_DARK           1800   // ADC < ค่านี้ = มืด (ต้องเทสต์จริง)
#define LDR_TH_BRIGHT         2300   // ADC > ค่านี้ = สว่าง (ยังไม่ใช้เยอะ)

#define PERSON_DISTANCE_CM    150.0f   // ระยะถือว่ามีคน (เช่น < 1.5 m)

#define BASE_TIMEOUT_MS       (60 * 1000)    // ปิดไฟอัตโนมัติเมื่อไม่มีคน 60 วินาที
#define EXT_TIMEOUT_MS        (180 * 1000)   // ถ้ามีเสียงบ่อย → ขยาย timeout

#define SOUND_ACTIVE_WINDOW_MS  30000        // มีเสียงภายใน 30 วินาที → ถือว่ายัง active

#define CLAP_GAP_MAX_MS       300            // ระยะห่าง clap 2 ครั้งมากสุด
#define CLAP_RESET_MS         800            // ถ้าห่างนานกว่านี้ถือว่าเริ่มนับใหม่

#define NOISE_WINDOW_MS       800      // ดูทุก 0.8 วินาที
#define NOISE_COUNT_HIGH      2        // แค่ 2 ครั้งก็ HIGH แล้ว
#define NOISE_HIGH_HOLD_MS   100     // ภายใน 100 ms หลังเจอเสียง = HIGH (ลด hold ไม่ให้ค้าง)
#define NOISE_MED_HOLD_MS    800     // ภายใน 0.8 วิ หลังเจอเสียง = MED
#define NOISE_COUNT_WINDOW_MS (60 * 1000)  // หน้าต่างนับจำนวนเหตุการณ์เสียง 60 วิ
#define NOISE_MAX_HIGH_PER_WINDOW 3       // อนุญาต HIGH ได้ไม่เกิน 3 ครั้งใน 60 วิ
#define NOISE_MAX_MED_PER_WINDOW  8       // อนุญาต MED ได้ไม่เกิน 8 ครั้งใน 60 วิ

#define STATUS_TX_INTERVAL_MS 500            // ส่งสถานะออก UART ทุก 500ms

#define INTRUSION_HOLD_MS     5000           // โหมด Intrusion alarm (logic ไว้ก่อน เผื่อใช้ทีหลัง)

/* ---------- Debug Mic RMS / dB ---------- */
// เลือกช่อง ADC ของไมค์อนาล็อก (ต่อสายให้ตรงกับ CubeMX ด้วย)
#define MIC_ADC_CHANNEL       ADC_CHANNEL_1   // ปรับตามขาที่ใช้ต่อไมค์ AOUT
#define MIC_ADC_SAMPLE_TIME   ADC_SAMPLETIME_480CYCLES  // เพิ่มเวลา sample ให้สัญญาณนิ่ง
#define MIC_RMS_SAMPLES       64             // จำนวนตัวอย่างต่อเฟรม RMS
#define MIC_DEBUG_INTERVAL_MS 250            // เวลาพิมพ์ debug ต่อครั้ง
#define MIC_DEBUG_ENABLE      0              // 1 = เปิดพิมพ์ debug dB, 0 = ปิด

// ใช้คาลิเบรต dB SPL: วัด vrms_ref ตอนเปิดเสียงที่ spl_ref_dB (เช่น 94 dB @1kHz)
#define MIC_SPL_REF_DB        55.0f          // ใช้ 55 dB เป็นระดับเสียงปกติ (ambient)
#define MIC_VRMS_REF          0.0020f        // Vrms ที่ได้เมื่อเสียง ~55 dB (ปรับตามที่วัดจริง)

// ถ้าขา D0 ไม่เปลี่ยน ให้ใช้ค่าจาก ADC เป็นตัวตัดสินแทน: 1 เมื่อ RMS สูงกว่า threshold นี้ (หน่วย ADC count)
#define MIC_ADC_THRESHOLD     100            // ปรับตามสภาพจริง (ดูจาก MICADC ใน STATUS)
#define MIC_ADC_CONSEC_HIT    3              // ต้องสูงต่อเนื่องกี่ครั้งถึงจะถือว่ามีเสียง (กันสไปก์)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* ---------- ตัวแปรสถานะระบบ ---------- */
static SystemMode_t g_mode = MODE_AUTO;

static uint8_t  g_lightOn = 0;
static uint8_t  g_intrusion = 0;  // เผื่อใช้ในโหมด AWAY ภายหลัง

static uint16_t g_ldrValue = 0;
static uint8_t  g_isDark = 0;
static uint8_t  g_isBright = 0;

static float    g_distanceCm = 999.0f;
static uint8_t  g_hasPerson = 0;

static uint8_t  g_micDigital = 0;
static NoiseLevel_t g_noiseLevel = NOISE_LOW;

static uint32_t g_lastPersonTime = 0;
static uint32_t g_lastSoundTime = 0;
static uint32_t g_lastStatusTxTime = 0;
static uint32_t g_lastNoiseWindowStart = 0;
static uint16_t g_noiseHighCountWindow = 0;
static uint16_t g_noiseMedCountWindow  = 0;

static uint16_t g_micAdcRms = 0;   // ค่าที่อ่านได้จาก Mic ADC (RMS แบบดิบ)
static float    g_micVrms   = 0.0f;
static float    g_micDb     = 0.0f;
static float    g_micDbSpl  = 0.0f; // dB SPL (อ้างอิงค่าคาลิเบรต)


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* ---------- ฟังก์ชันของระบบ ---------- */
static uint16_t LDR_ReadRaw(void);
static void     Sensors_Update(uint32_t now);
static float    Ultrasonic_ReadDistanceCm(void);
static float    Mic_ReadRmsDb(uint16_t *adcRmsOut, float *vrmsOut);
static void     Mic_DebugTick(uint32_t now);
static void     Logic_Update(uint32_t now);
static void     Light_UpdateGPIO(void);
static void     Mode_LED_Update(void);

static void     UART_SendStatus(uint32_t now);
static void     UART_ProcessRx(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Retarget printf ไป UART2 */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* อ่าน ADC ของ LDR */
static uint16_t LDR_ReadRaw(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = LDR_ADC_CHANNEL;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  HAL_ADC_Start(&hadc1);

  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
  {
    uint16_t val = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return val;
  }

  HAL_ADC_Stop(&hadc1);
  return 0;
}

/* ใช้ TIM2 วัดเวลาของ ECHO แบบละเอียดระดับไมโครวินาที */

static float Ultrasonic_ReadDistanceCm(void)
{
  /* 1) ส่ง Trigger ประมาณ 10us */
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);
  HAL_Delay(1); // 1 ms เคลียร์ก่อน
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_SET);
  for (volatile int i = 0; i < 300; i++);   // delay ~10us (หยาบๆ)
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);

  /* 2) รอ Echo ขึ้น HIGH พร้อม timeout กันค้าง */
  uint32_t tickStart = HAL_GetTick();
  while (HAL_GPIO_ReadPin(US_ECHO_GPIO_Port, US_ECHO_Pin) == GPIO_PIN_RESET)
  {
    if ((HAL_GetTick() - tickStart) > 50) {
      return 999.0f;   // รอนานเกิน 50 ms → ถือว่าไม่มีอะไรสะท้อน
    }
  }

  /* 3) วัดความกว้าง pulse แบบหน่วย ms (หยาบ ๆ) */
  uint32_t tStart = HAL_GetTick();
  while (HAL_GPIO_ReadPin(US_ECHO_GPIO_Port, US_ECHO_Pin) == GPIO_PIN_SET)
  {
    if ((HAL_GetTick() - tStart) > 50) {
      return 999.0f;   // pulse ยาวผิดปกติ → ถือว่า error
    }
  }
  uint32_t tEnd = HAL_GetTick();
  uint32_t dt_ms = tEnd - tStart;

  if (dt_ms == 0 || dt_ms > 50) {
    return 999.0f;     // กันกรณีหลุด ๆ
  }

  /* 4) แปลงเป็นระยะ (ประมาณ 17 cm ต่อ 1 ms) */
  float distance = dt_ms * 17.0f;   // หน่วย cm (คร่าว ๆ)

  /* 6) low-pass filter ให้ค่าเนียนเหมือนเดิม */
  static float filtered = -1.0f;
  if (filtered < 0.0f) {
    filtered = distance;
  } else {
    filtered = 0.7f * filtered + 0.3f * distance;
  }

  return filtered;
}

/* อ่านไมค์อนาล็อกเป็น RMS + dB (ใช้สำหรับ debug) */
static float Mic_ReadRmsDb(uint16_t *adcRmsOut, float *vrmsOut)
{
  // ใช้ Welford incremental เพื่อหาค่า RMS แบบตัด DC offset ออก (AC component เท่านั้น)
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Rank = 1;
  sConfig.SamplingTime = MIC_ADC_SAMPLE_TIME;

  uint16_t adcRmsLocal = 0;
  float    vrmsLocal = 0.0f;
  float    dbLocal   = -120.0f;

  // helper อ่าน RMS จาก channel ที่ระบุ
  const uint32_t channelsToTry[2] = { MIC_ADC_CHANNEL, ADC_CHANNEL_0 }; // ลองช่องไมค์ก่อน ตามด้วย CH0 เป็น fallback

  for (int chIdx = 0; chIdx < 2; chIdx++)
  {
    float mean = 0.0f;
    float m2   = 0.0f;
    uint32_t n = 0;

    sConfig.Channel = channelsToTry[chIdx];
    for (int i = 0; i < MIC_RMS_SAMPLES; i++)
    {
      HAL_ADC_ConfigChannel(&hadc1, &sConfig);
      HAL_ADC_Start(&hadc1);
      if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
      {
        uint32_t v = HAL_ADC_GetValue(&hadc1);
        n++;
        float delta = (float)v - mean;
        mean += delta / (float)n;
        float delta2 = (float)v - mean;
        m2 += delta * delta2;
      }
      HAL_ADC_Stop(&hadc1);
    }

    if (n > 0)
    {
      float variance = m2 / (float)n;
      if (variance < 0) variance = 0;
      float adcRmsFloat = sqrtf(variance);
      if (adcRmsFloat < 1e-3f) adcRmsFloat = 1e-3f;
      float vrms = adcRmsFloat * (3.3f / 4095.0f);
      float db   = 20.0f * log10f(vrms / 1.0f);
      adcRmsLocal = (uint16_t)(adcRmsFloat + 0.5f);
      vrmsLocal   = vrms;
      dbLocal     = db;

      // ถ้าไม่ใช่ 0 แล้ว ไม่ต้องลองช่องถัดไป
      if (adcRmsLocal != 0) break;
    }
  }

  if (adcRmsOut) *adcRmsOut = adcRmsLocal;
  if (vrmsOut) *vrmsOut = vrmsLocal;
  return dbLocal;
}


/* อัปเดตค่าจาก sensor ทั้งหมด + clap / noise */
static void Sensors_Update(uint32_t now)
{
  /* ---------- LDR ---------- */
  g_ldrValue = LDR_ReadRaw();
  g_isDark   = (g_ldrValue < LDR_TH_DARK);
  g_isBright = (g_ldrValue > LDR_TH_BRIGHT);

  /* ---------- Mic Digital (อ่านเสียง + clap) ---------- */
  static uint8_t micConsec = 0; // ใช้หน่วงเพื่อลดสไปก์จาก ADC

  // ใช้ขา D0 เป็น optional แต่ให้ยึดค่าจาก ADC เป็นหลัก
  uint8_t raw = (HAL_GPIO_ReadPin(MIC_GPIO_Port, MIC_Pin) == GPIO_PIN_RESET) ? 1 : 0;

  // อ่าน RMS (AC) และถ้าสูงกว่า threshold ให้ถือว่ามีเสียง (override D0)
  uint16_t adcRmsTmp = 0;
  float vrmsTmp = 0.0f;
  float dbTmp = Mic_ReadRmsDb(&adcRmsTmp, &vrmsTmp);
  if (adcRmsTmp > MIC_ADC_THRESHOLD) {
      if (micConsec < 255) micConsec++;
  } else {
      if (micConsec > 0) micConsec--; // ลดลงทีละขั้นเพื่อหน่วง
  }

  if (micConsec >= MIC_ADC_CONSEC_HIT) {
      raw = 1;
      g_lastSoundTime = now; // อัปเดตเวลาเสียงล่าสุด
  } else {
      raw = 0; // ค่าต่ำหรือไม่ต่อเนื่องพอถือว่าเงียบ
  }

  g_micDigital = raw; // ใช้ ADC เป็นตัวตัดสินหลัก (D0 เป็นแค่ตัวเสริม)
  g_micAdcRms = adcRmsTmp;
  g_micVrms   = vrmsTmp;
  g_micDb     = dbTmp;
  g_micDbSpl  = MIC_SPL_REF_DB + 20.0f * log10f((vrmsTmp < 1e-6f ? 1e-6f : vrmsTmp) / MIC_VRMS_REF);

  /* ---------- Ultrasonic ---------- */
  g_distanceCm = Ultrasonic_ReadDistanceCm();
  if (g_distanceCm > 0 && g_distanceCm < PERSON_DISTANCE_CM)
  {
    g_hasPerson = 1;
    g_lastPersonTime = now;
  }
}

static void Logic_Update(uint32_t now)
{
    /* ---------- อัปเดตระดับ NOISE จากค่า MICSPL โดยตรง ---------- */
    if (g_micDbSpl < 60.0f) {
        g_noiseLevel = NOISE_LOW;
    } else if (g_micDbSpl < 75.0f) {  // 60–75 dB = MED
        g_noiseLevel = NOISE_MED;
    } else {
        g_noiseLevel = NOISE_HIGH;    // >= 75 dB
    }

    /* ---------- นับจำนวนเหตุการณ์เสียงในหน้าต่าง 60 วิ ---------- */
    static NoiseLevel_t prevNoiseLevel = NOISE_LOW;
    if (prevNoiseLevel != g_noiseLevel)
    {
        if (g_noiseLevel == NOISE_HIGH) {
            g_noiseHighCountWindow++;
        } else if (g_noiseLevel == NOISE_MED) {
            g_noiseMedCountWindow++;
        }
    }
    prevNoiseLevel = g_noiseLevel;

    if (now - g_lastNoiseWindowStart >= NOISE_COUNT_WINDOW_MS) {
        g_lastNoiseWindowStart = now;
        g_noiseHighCountWindow = 0;
        g_noiseMedCountWindow  = 0;
    }

    /* ---------- เงื่อนไขหลักตามที่ต้องการ ---------- */

    // 1) เช็คว่าตอนนี้ ultrasonic ตรวจจับคนอยู่ไหม
    uint8_t personDetectedNow =
        (g_distanceCm > 0.0f && g_distanceCm < PERSON_DISTANCE_CM);

    // 2) เสียงเกิน threshold หรือยัง?
    uint8_t noiseTooHigh =
        (g_noiseHighCountWindow >= NOISE_MAX_HIGH_PER_WINDOW) ||
        (g_noiseMedCountWindow  >= NOISE_MAX_MED_PER_WINDOW);

    // 3) ถ้าเสียง HIGH/MED เกิน threshold และมีคนอยู่ -> ขอเปิดไฟ
    if (noiseTooHigh && personDetectedNow)
    {
        // เช็คเซนเซอร์แสง ถ้ามืด/แสงไม่พอ -> เปิดไฟ (และในอนาคตค่อยเพิ่มการปรับ brightness)
        if (g_isDark) {
            g_lightOn = 1;

            // TODO: ถ้ามีวงจร dimming จริง ๆ ให้ปรับ PWM ที่นี่
            // เช่น Light_SetBrightness(level) แล้วอ่าน LDR วนจนกว่าจะสว่างพอ
        }
    }

    // 4) ถ้าไฟเปิดอยู่แล้ว และตอนนี้เสียงกลับมา LOW และ count ต่ำกว่า threshold -> ปิดไฟ
    if (g_lightOn &&
        g_noiseLevel == NOISE_LOW &&
        g_noiseHighCountWindow < NOISE_MAX_HIGH_PER_WINDOW &&
        g_noiseMedCountWindow  < NOISE_MAX_MED_PER_WINDOW)
    {
        g_lightOn = 0;
    }

    /* ---------- ส่วนโหมดอื่น ๆ (ถ้ายังอยากใช้) ---------- */
    switch (g_mode)
    {
        case MODE_AWAY:
            // ถ้าไม่ใช้ intrusion แล้ว สามารถลบทิ้งได้
            // หรือจะย้าย intrusion logic มา blend กับกฎด้านบนก็ได้
            break;

        case MODE_MANUAL:
        case MODE_AUTO:
        default:
            // ตอนนี้กฎเปิด/ปิดไฟหลักถูกคุมด้วยเงื่อนไขด้านบนแล้ว
            break;
    }
}

/* พิมพ์ค่าไมค์เป็น RMS/dB สำหรับ debug */
static void Mic_DebugTick(uint32_t now)
{
#if MIC_DEBUG_ENABLE
  static uint32_t lastPrint = 0;
  if (now - lastPrint < MIC_DEBUG_INTERVAL_MS) return;
  lastPrint = now;

  uint16_t adcRms = 0;
  float vrms = 0.0f;
  float db = Mic_ReadRmsDb(&adcRms, &vrms);
  float dbSpl = MIC_SPL_REF_DB + 20.0f * log10f((vrms < 1e-6f ? 1e-6f : vrms) / MIC_VRMS_REF);

  const char *noiseStr = (g_noiseLevel == NOISE_LOW) ? "LOW" :
                         (g_noiseLevel == NOISE_MED) ? "MED" : "HIGH";

  printf("[MICDBG] adc_rms=%u vrms=%.3f dB=%.1f spl=%.1f mic_dig=%d noise=%s\\r\\n",
         adcRms, vrms, db, dbSpl, g_micDigital, noiseStr);
#else
  (void)now;
#endif
}

/* อัปเดต GPIO ของไฟห้อง */
static void Light_UpdateGPIO(void)
{
  if (g_lightOn)
    HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET);
}

/* Mode LED (ใช้ LD2) */
static void Mode_LED_Update(void)
{
  uint32_t now = HAL_GetTick();
  static uint8_t ledOn = 0;

  switch (g_mode)
  {
    case MODE_AUTO:
      if ((now / 500) % 2 == 0) ledOn = 0;
      else ledOn = 1;
      break;
    case MODE_AWAY:
      ledOn = 1;
      break;
    case MODE_MANUAL:
    default:
      ledOn = 0;
      break;
  }

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, ledOn ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ส่งสถานะปัจจุบันออก UART2 */
static void UART_SendStatus(uint32_t now)
{
  if (now - g_lastStatusTxTime < STATUS_TX_INTERVAL_MS) return;
  g_lastStatusTxTime = now;

  // g_micAdcRms / g_micVrms / g_micDb ถูกอัปเดตแล้วใน Sensors_Update (รวม fallback ADC)
  if (g_micVrms < 1e-6f) g_micVrms = 1e-6f; // กัน log(0)

  char buf[196];

  const char *modeStr = (g_mode == MODE_AUTO)   ? "AUTO" :
                        (g_mode == MODE_AWAY)   ? "AWAY" :
                                                 "MANUAL";

  const char *noiseStr = (g_noiseLevel == NOISE_LOW) ? "LOW" :
                         (g_noiseLevel == NOISE_MED) ? "MED" : "HIGH";

  snprintf(buf, sizeof(buf),
           "STATUS;MODE=%s;LIGHT=%d;LDR=%u;DIST=%.1f;NOISE=%s;MIC=%d;INTR=%d;MICADC=%u;MICVR=%.3f;MICDB=%.1f;MICSPL=%.1f\r\n",
           modeStr,
           g_lightOn,
           g_ldrValue,
           g_distanceCm,
           noiseStr,
           g_micDigital,
           g_intrusion,
           g_micAdcRms,
           g_micVrms,
           g_micDb,
           g_micDbSpl);

  HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), 50);
}

/* โครงรับคำสั่งจาก UART (ยังว่างไว้ให้ต่อยอด) */
static void UART_ProcessRx(void)
{
  /* TODO: ถ้าจะคุยกับ NodeMCU / ESP ให้ทำ parser ตรงนี้ */
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  printf("\r\n[SYS] Smart Light Controller start\r\n");
  g_lastNoiseWindowStart = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    Sensors_Update(now);
    Logic_Update(now);
    Light_UpdateGPIO();
    Mode_LED_Update();

    UART_SendStatus(now);
    UART_ProcessRx();

    Mic_DebugTick(now);

    HAL_Delay(1);   // loop ทุก ~10ms
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;           // ใช้ single conversion สลับช่องเอง
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure default channel (LDR) */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
