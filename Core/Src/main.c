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

/* Private define -----------------------------------------------------------*/
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

#define CLAP_GAP_MAX_MS       500            // ระยะห่าง clap 2 ครั้งมากสุด
#define CLAP_RESET_MS         800            // ถ้าห่างนานกว่านี้ถือว่าเริ่มนับใหม่

#define NOISE_WINDOW_MS       800      // ดูทุก 0.8 วินาที
#define NOISE_COUNT_HIGH      2        // แค่ 2 ครั้งก็ HIGH แล้ว
#define NOISE_HIGH_HOLD_MS   200     // ภายใน 200 ms หลังเจอเสียง = HIGH
#define NOISE_MED_HOLD_MS   2000     // ภายใน 2 วิ หลังเจอเสียง = MED

#define STATUS_TX_INTERVAL_MS 500            // ส่งสถานะออก UART ทุก 500ms

#define INTRUSION_HOLD_MS     5000           // โหมด Intrusion alarm (logic ไว้ก่อน เผื่อใช้ทีหลัง)

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
static uint32_t g_lastIntrusionTime = 0;
static uint32_t g_lastStatusTxTime = 0;
static uint32_t g_lastNoiseWindowStart = 0;
static uint16_t g_noiseTriggerCount = 0;

static uint8_t  g_clapCount = 0;
static uint32_t g_lastClapTime = 0;

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

/* อ่าน Ultrasonic – เวอร์ชันมี low-pass filter ให้ค่าเนียนขึ้น */
static float Ultrasonic_ReadDistanceCm(void)
{
  /* 1) ส่ง Trigger ประมาณ 10us */
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_SET);
  for (volatile int i = 0; i < 300; i++);   // delay ~10us (หยาบ ๆ)
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);

  /* 2) รอ Echo ขึ้น HIGH */
  uint32_t start = HAL_GetTick();
  while (HAL_GPIO_ReadPin(US_ECHO_GPIO_Port, US_ECHO_Pin) == GPIO_PIN_RESET)
  {
    if ((HAL_GetTick() - start) > 50) {
      return 999.0f;   // timeout = ไกลมาก / มองไม่เห็นอะไร
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

  /* 5) low-pass filter ให้ค่าเนียนขึ้น */
  static float filtered = -1.0f;
  if (filtered < 0) {
    filtered = distance;                          // อันแรก ตั้งค่าเริ่มต้น
  } else {
    filtered = 0.7f * filtered + 0.3f * distance; // ผสมค่าใหม่เข้าไป 30%
  }

  return filtered;
}


/* อัปเดตค่าจาก sensor ทั้งหมด + clap / noise */
static void Sensors_Update(uint32_t now)
{
  /* ---------- LDR ---------- */
  g_ldrValue = LDR_ReadRaw();
  g_isDark   = (g_ldrValue < LDR_TH_DARK);
  g_isBright = (g_ldrValue > LDR_TH_BRIGHT);

  /* ---------- Mic Digital (อ่านเสียง + clap) ---------- */
  static uint8_t prevMic = 0;

  // เลือก polarity ให้ถูกกับโมดูลไมค์ของเรา
  // OPTION A: ถ้าโมดูลเป็น active-HIGH (เงียบ = 0, มีเสียง = 1)
  uint8_t raw = (HAL_GPIO_ReadPin(MIC_GPIO_Port, MIC_Pin) == GPIO_PIN_SET) ? 1 : 0;

  // OPTION B: ถ้าโมดูลเป็น active-LOW (เงียบ = 1, มีเสียง = 0)
  // ลองสลับมาใช้บรรทัดนี้แทนด้านบน ถ้าพูด/ตบแล้วยังไม่เห็นอะไรเปลี่ยน
  // uint8_t raw = (HAL_GPIO_ReadPin(MIC_GPIO_Port, MIC_Pin) == GPIO_PIN_RESET) ? 1 : 0;

  g_micDigital = raw;

  // ถ้าค่า MIC เปลี่ยน (edge 0<->1) = เพิ่งมีเสียงเกิดขึ้น
  if (raw != prevMic)
  {
      // จดเวลาเสียงล่าสุด
      g_lastSoundTime = now;

      // ---- Clap detection: ถ้า 2 ครั้งห่างกันไม่เกิน CLAP_GAP_MAX_MS ----
      uint32_t dt = now - g_lastClapTime;
      if (dt < CLAP_GAP_MAX_MS) {
          g_clapCount++;
      } else {
          g_clapCount = 1;
      }
      g_lastClapTime = now;

      if (g_clapCount >= 2) {
          // toggle ไฟด้วยการตบมือ ในโหมด AUTO / MANUAL
          if (g_mode == MODE_AUTO || g_mode == MODE_MANUAL) {
              g_lightOn = !g_lightOn;
          }
          g_clapCount = 0;
      }
  }

  // ถ้านานเกิน CLAP_RESET_MS แล้วไม่มี edge ใหม่ → รีเซ็ตการนับ clap
  if (now - g_lastClapTime > CLAP_RESET_MS) {
      g_clapCount = 0;
  }

  prevMic = raw;
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
    uint32_t timeSincePerson = now - g_lastPersonTime;
    uint32_t timeSinceSound  = now - g_lastSoundTime;

    /* ---------- สรุประดับ NOISE จากเวลาที่ได้ยินเสียงล่าสุด ---------- */
    uint32_t dtSound = now - g_lastSoundTime;

    if (dtSound < NOISE_HIGH_HOLD_MS) {
        g_noiseLevel = NOISE_HIGH;    // เพิ่งมีเสียงเปลี่ยนเมื่อไม่กี่ ms
    }
    else if (dtSound < NOISE_MED_HOLD_MS) {
        g_noiseLevel = NOISE_MED;     // มีเสียงภายใน 0.2–2 วิที่ผ่านมา
    }
    else {
        g_noiseLevel = NOISE_LOW;     // เงียบมานานแล้ว
    }

    /* ---------- Adaptive Timeout ใช้ timeSinceSound เดิมได้ตาม logic คุณ ---------- */
    uint32_t activeTimeout = BASE_TIMEOUT_MS;
    if (timeSinceSound < SOUND_ACTIVE_WINDOW_MS) {
        activeTimeout = EXT_TIMEOUT_MS; // ถ้ามีเสียงในช่วง 30 วิ เพิ่ม timeout
    }

    /* ---------- จากตรงนี้ลงไปใช้ switch(g_mode) เดิมของคุณต่อได้เลย ---------- */

    switch (g_mode)
    {
        case MODE_AUTO:
        {
            uint8_t consideredHasPerson = (timeSincePerson < 3000); // ถ้า 3 วิ ไม่เจอ = ไม่มีคน

            if (consideredHasPerson && g_isDark) {
                g_lightOn = 1;
            } else if (!consideredHasPerson && timeSincePerson > activeTimeout) {
                g_lightOn = 0;
            }
        }
        break;

        case MODE_AWAY:
        	uint8_t intrusionCondition = 0;

        	      if (g_distanceCm > 0 && g_distanceCm < PERSON_DISTANCE_CM)
        	        intrusionCondition = 1;

        	      if (g_noiseLevel == NOISE_HIGH)
        	        intrusionCondition = 1;

        	      if (intrusionCondition && (now - g_lastIntrusionTime > INTRUSION_HOLD_MS))
        	      {
        	        g_intrusion = 1;
        	        g_lastIntrusionTime = now;
        	        g_lightOn = 1;  // เปิดไฟขู่
        	      }

        	      if (g_intrusion && (now - g_lastIntrusionTime > INTRUSION_HOLD_MS))
        	      {
        	        g_intrusion = 0;
        	      }
        break;

        case MODE_MANUAL:
        default:
              /* manual mode: g_lightOn จะมาจาก clap หรือคำสั่ง UART */
              break;
    }
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

  char buf[128];

  const char *modeStr = (g_mode == MODE_AUTO)   ? "AUTO" :
                        (g_mode == MODE_AWAY)   ? "AWAY" :
                                                 "MANUAL";

  const char *noiseStr = (g_noiseLevel == NOISE_LOW) ? "LOW" :
                         (g_noiseLevel == NOISE_MED) ? "MED" : "HIGH";

  snprintf(buf, sizeof(buf),
           "STATUS;MODE=%s;LIGHT=%d;LDR=%u;DIST=%.1f;NOISE=%s;MIC=%d;INTR=%d\r\n",
           modeStr,
           g_lightOn,
           g_ldrValue,
           g_distanceCm,
           noiseStr,
           g_micDigital,   // 👈 เพิ่มตรงนี้
           g_intrusion);

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

  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

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
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
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

  sConfig.Channel = ADC_CHANNEL_0;          // PA0
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin (ปุ่มบนบอร์ด) */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin (Mode LED) */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* Configure GPIO pin : LIGHT_Pin (PB10) */
  GPIO_InitStruct.Pin = LIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LIGHT_GPIO_Port, &GPIO_InitStruct);

  /* Configure GPIO pin : US_TRIG_Pin (PB1) */
  GPIO_InitStruct.Pin = US_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(US_TRIG_GPIO_Port, &GPIO_InitStruct);

  /* Configure GPIO pin : MIC_Pin (PB0) */
  GPIO_InitStruct.Pin = MIC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MIC_GPIO_Port, &GPIO_InitStruct);

  /* Configure GPIO pin : US_ECHO_Pin (PB2) */
  GPIO_InitStruct.Pin = US_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(US_ECHO_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* รายงาน error ถ้าต้องการ */
}
#endif /* USE_FULL_ASSERT */
