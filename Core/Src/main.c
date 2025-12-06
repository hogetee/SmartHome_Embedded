/* main.c – Smart Auto Light + Security (Nucleo F411RE)
 *
 * ฟีเจอร์:
 *  - Auto Light ON/OFF จาก LDR + Ultrasonic
 *  - Adaptive Timeout ตามเสียง (Mic)
 *  - Day/Night Sensitivity
 *  - Intrusion Detection (โหมด AWAY)
 *  - Clap to Toggle (ตบมือ 2 ครั้ง)
 *  - Sound-based Alert
 *  - ส่งสถานะทาง UART ไป NodeMCU
 */

#include "main.h"
#include <stdio.h>
#include <string.h>

/* ---------- Handle ที่มาจาก CubeMX ---------- */
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart2;

/* ---------- GPIO Mapping (แก้ให้ตรงกับ CubeMX) ---------- */
#define LDR_ADC_CHANNEL       ADC_CHANNEL_0   // สมมติ LDR ที่ PA0
#define MIC_GPIO_Port         GPIOB
#define MIC_Pin               GPIO_PIN_0      // Mic D0

#define US_TRIG_GPIO_Port     GPIOB
#define US_TRIG_Pin           GPIO_PIN_1
#define US_ECHO_GPIO_Port     GPIOB
#define US_ECHO_Pin           GPIO_PIN_2

#define LIGHT_GPIO_Port       GPIOB
#define LIGHT_Pin             GPIO_PIN_10    // MOSFET/Relay คุมไฟ

#define BUZZ_GPIO_Port        GPIOB
#define BUZZ_Pin              GPIO_PIN_4

#define LED_GPIO_Port         GPIOC
#define LED_Pin               GPIO_PIN_13    // LED บนบอร์ด (Active Low)

/* ---------- ค่าปรับ (Tuning Parameters) ---------- */
#define LDR_TH_DARK           1800   // ADC < ค่านี้ = มืด (ต้องเทสต์จริง)
#define LDR_TH_BRIGHT         2300   // ADC > ค่านี้ = สว่าง

#define PERSON_DISTANCE_CM    150.0f   // ระยะถือว่ามีคน (เช่น < 1.5 m)

#define BASE_TIMEOUT_MS       (60 * 1000)    // ปิดไฟอัตโนมัติเมื่อไม่มีคน 60 วินาที
#define EXT_TIMEOUT_MS        (180 * 1000)   // ถ้ามีเสียงบ่อย → ขยาย timeout

#define SOUND_ACTIVE_WINDOW_MS  30000        // มีเสียงภายใน 30 วินาที → ถือว่ายัง active

#define CLAP_GAP_MAX_MS       500            // ระยะห่าง clap 2 ครั้งมากสุด
#define CLAP_RESET_MS         800            // ถ้าห่างนานกว่านี้ถือว่าเริ่มนับใหม่

#define NOISE_WINDOW_MS       1000           // วินโดว์ตรวจเสียงดังผิดปกติ
#define NOISE_COUNT_HIGH      5              // ภายใน 1s ถ้า trigger >= 5 = เสียงดังผิดปกติ

#define STATUS_TX_INTERVAL_MS 500            // ส่งสถานะออก UART ทุก 500ms
#define BUZZER_ALERT_MS       300            // buzzer ดังสั้น ๆ เวลามี alert เล็ก

#define INTRUSION_HOLD_MS     5000           // โหมด Intrusion alarm นาน 5 วินาที

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

/* ---------- ตัวแปรสถานะระบบ ---------- */
static SystemMode_t g_mode = MODE_AUTO;

static uint8_t  g_lightOn = 0;
static uint8_t  g_intrusion = 0;

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

/* ---------- ฟังก์ชันช่วยประกาศล่วงหน้า ---------- */
static uint16_t LDR_ReadRaw(void);
static void     Sensors_Update(uint32_t now);
static float    Ultrasonic_ReadDistanceCm(void);
static void     Logic_Update(uint32_t now);
static void     Light_UpdateGPIO(void);
static void     Buzzer_UpdateGPIO(uint32_t now);
static void     Mode_LED_Update(void);

static void     UART_SendStatus(uint32_t now);
static void     UART_ProcessRx(void); // TODO: parser คำสั่งจาก NodeMCU (จะใส่โครงให้)

/* ---------- Retarget printf ไป UART2 ---------- */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* ======================= main ======================= */

int main(void)
{
    HAL_Init();
    SystemClock_Config();   // มาจาก CubeMX
    MX_GPIO_Init();         // มาจาก CubeMX
    MX_ADC1_Init();         // มาจาก CubeMX
    MX_USART2_UART_Init();  // มาจาก CubeMX

    // ถ้าใช้ DWT หรือ TIMER สำหรับ delay microsecond ให้ init ตรงนี้

    printf("\r\n[SYS] Smart Light Controller start\r\n");

    g_lastNoiseWindowStart = HAL_GetTick();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        Sensors_Update(now);
        Logic_Update(now);
        Light_UpdateGPIO();
        Buzzer_UpdateGPIO(now);
        Mode_LED_Update();

        UART_SendStatus(now);
        UART_ProcessRx();  // TODO: เขียน parser ตามโปรโตคอลที่คุณจะใช้

        HAL_Delay(10); // loop ทุก ~10ms
    }
}

/* ======================= SENSOR ======================= */

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

/* อ่าน Ultrasonic – ตรงนี้เป็นโค้ดตัวอย่างง่าย ๆ ต้องไปแก้ tuning เอง */
static float Ultrasonic_ReadDistanceCm(void)
{
    // 1) ส่ง Trigger 10us
    HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_SET);
    // delay ~10 µs (ในที่นี้ใช้วิธีหยาบ ๆ ถ้าคุณมีฟังก์ชัน delay_us ให้ใช้แทน)
    for (volatile int i = 0; i < 300; i++);
    HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);

    // 2) รอ Echo ขึ้น HIGH
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(US_ECHO_GPIO_Port, US_ECHO_Pin) == GPIO_PIN_RESET)
    {
        if ((HAL_GetTick() - start) > 50) {
            return 999.0f; // timeout
        }
    }

    // 3) วัด pulse width แบบหยาบ (ใช้ HAL_GetTick ไม่ค่อยละเอียดมาก แต่พอ demo ได้)
    uint32_t tStart = HAL_GetTick();
    while (HAL_GPIO_ReadPin(US_ECHO_GPIO_Port, US_ECHO_Pin) == GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - tStart) > 50) {
            return 999.0f; // timeout
        }
    }
    uint32_t tEnd = HAL_GetTick();

    uint32_t dt_ms = tEnd - tStart;
    // สูตรจริง: distance(cm) ≈ pulse_us / 58
    // ตรงนี้ใช้ ms แบบคร่าว ๆ = ไม่แม่น แต่พอแยก "มีอะไรใกล้/ไกล" ได้
    float distance = dt_ms * 1000.0f / 58.0f;

    return distance;
}

/* อัปเดตค่าจาก sensor ทั้งหมด และทำ logic clap / noise window */
static void Sensors_Update(uint32_t now)
{
    /* ---------- LDR ---------- */
    g_ldrValue = LDR_ReadRaw();
    g_isDark   = (g_ldrValue < LDR_TH_DARK);
    g_isBright = (g_ldrValue > LDR_TH_BRIGHT);

    /* ---------- Mic Digital ---------- */
    static uint8_t prevMic = 0;
    g_micDigital = (HAL_GPIO_ReadPin(MIC_GPIO_Port, MIC_Pin) == GPIO_PIN_SET) ? 1 : 0;

    // นับ noise trigger ใน 1 วินาที
    if (now - g_lastNoiseWindowStart > NOISE_WINDOW_MS) {
        // สรุป noise level จาก count
        if (g_noiseTriggerCount >= NOISE_COUNT_HIGH) {
            g_noiseLevel = NOISE_HIGH;
        } else if (g_noiseTriggerCount >= (NOISE_COUNT_HIGH / 2)) {
            g_noiseLevel = NOISE_MED;
        } else {
            g_noiseLevel = NOISE_LOW;
        }

        g_noiseTriggerCount = 0;
        g_lastNoiseWindowStart = now;
    }

    if (g_micDigital) {
        g_noiseTriggerCount++;
        g_lastSoundTime = now;
    }

    /* ---------- Clap detection (ใช้ edge 0->1) ---------- */
    if (!prevMic && g_micDigital) {
        uint32_t dt = now - g_lastClapTime;
        if (dt < CLAP_GAP_MAX_MS) {
            g_clapCount++;
        } else {
            g_clapCount = 1;
        }
        g_lastClapTime = now;

        if (g_clapCount >= 2) {
            // ตรวจว่าอยู่ในโหมดที่อนุญาตให้ clap toggle
            if (g_mode == MODE_AUTO || g_mode == MODE_MANUAL) {
                g_lightOn = !g_lightOn;   // toggle ไฟ
            }
            g_clapCount = 0;
        }
    }
    // reset clap ถ้านานเกิน
    if (now - g_lastClapTime > CLAP_RESET_MS) {
        g_clapCount = 0;
    }

    prevMic = g_micDigital;

    /* ---------- Ultrasonic ---------- */
    g_distanceCm = Ultrasonic_ReadDistanceCm();
    if (g_distanceCm > 0 && g_distanceCm < PERSON_DISTANCE_CM) {
        g_hasPerson = 1;
        g_lastPersonTime = now;
    } else {
        // จะปล่อยให้ g_hasPerson ถูกใช้ใน logic อีกที (เช็คเวลา)
    }
}

/* ======================= LOGIC หลัก ======================= */

static void Logic_Update(uint32_t now)
{
    uint32_t timeSincePerson = now - g_lastPersonTime;
    uint32_t timeSinceSound  = now - g_lastSoundTime;

    /* ---------- Adaptive Timeout ---------- */
    uint32_t activeTimeout = BASE_TIMEOUT_MS;
    if (timeSinceSound < SOUND_ACTIVE_WINDOW_MS) {
        activeTimeout = EXT_TIMEOUT_MS; // ถ้ามีเสียงในช่วง 30 วิ เพิ่ม timeout
    }

    /* ---------- Sound-based Alert ---------- */
    static uint32_t lastAlertTime = 0;
    static uint8_t  alertActive = 0;

    if (g_noiseLevel == NOISE_HIGH && (now - lastAlertTime > 2000)) {
        // ทำ alert เบา ๆ ในโหมด AUTO / MANUAL
        if (g_mode == MODE_AUTO || g_mode == MODE_MANUAL) {
            alertActive = 1;
            lastAlertTime = now;
        }
    }

    /* ---------- การตัดสินใจตาม mode ---------- */
    switch (g_mode)
    {
        case MODE_AUTO:
        {
            // hasPerson จะถูกเซ็ตทุกครั้งที่ ultrasonic เจอ
            uint8_t consideredHasPerson = (timeSincePerson < 3000); // ถ้า 3 วิ ไม่เจอ = ไม่มีคน
            // ตัดสินใจเปิดไฟ
            if (consideredHasPerson && g_isDark) {
                g_lightOn = 1;
            } else if (!consideredHasPerson && timeSincePerson > activeTimeout) {
                g_lightOn = 0;
            }

            // Sound-based alert เบา ๆ → ใช้ buzzer สั้น ๆ (ไปทำใน Buzzer_Update)
            if (alertActive) {
                // set flag สำหรับ buzzer ให้ไปจัดการ
                // (ใช้ lastAlertTime + BUZZER_ALERT_MS ใน Buzzer_Update)
            }
        }
        break;

        case MODE_AWAY:
        {
            // โหมดกันขโมย / Intrusion Detection
            uint8_t intrusionCondition = 0;

            // 1) มี movement
            if (g_distanceCm > 0 && g_distanceCm < PERSON_DISTANCE_CM) {
                intrusionCondition = 1;
            }

            // 2) เสียงดังมากผิดปกติ
            if (g_noiseLevel == NOISE_HIGH) {
                intrusionCondition = 1;
            }

            if (intrusionCondition && (now - g_lastIntrusionTime > INTRUSION_HOLD_MS)) {
                g_intrusion = 1;
                g_lastIntrusionTime = now;
                g_lightOn = 1;  // เปิดไฟขู่
                // buzzer จะดังใน Buzzer_Update
            }

            // ปิด intrusion flag หลังครบเวลา
            if (g_intrusion && (now - g_lastIntrusionTime > INTRUSION_HOLD_MS)) {
                g_intrusion = 0;
            }
        }
        break;

        case MODE_MANUAL:
        default:
            // โหมดนี้ g_lightOn จะมาจากปุ่ม / คำสั่ง UART / clap เป็นหลัก
            break;
    }
}

/* ======================= OUTPUT: LIGHT / BUZZER / LED ======================= */

static void Light_UpdateGPIO(void)
{
    if (g_lightOn) {
        HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET);
    }
}

static void Buzzer_UpdateGPIO(uint32_t now)
{
    static uint8_t buzzerOn = 0;
    static uint32_t buzzerStart = 0;

    // 1) ถ้ามี intrusion → เปิด buzzer ค้างช่วงหนึ่ง
    if (g_intrusion) {
        HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
        return;
    }

    // 2) alarm เล็ก ๆ จาก sound-based alert (ในโหมดอื่น)
    // โค้ดนี้สามารถเชื่อมกับ flag หรือ lastAlertTime ได้
    // ตัวอย่าง: beep สั้น ๆ เมื่อ noiseLevel == HIGH
    // (เพื่อให้ง่าย ใช้ noiseLevel ตรวจตรง ๆ)
    if (g_noiseLevel == NOISE_HIGH && !buzzerOn) {
        buzzerOn = 1;
        buzzerStart = now;
        HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
    }

    if (buzzerOn && (now - buzzerStart > BUZZER_ALERT_MS)) {
        buzzerOn = 0;
        HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
    }
}

static void Mode_LED_Update(void)
{
    // ตัวอย่าง: ใช้ LED บนบอร์ดแสดง mode แบบง่าย ๆ
    // AUTO  = LED กระพริบช้า
    // AWAY  = LED ติดค้าง
    // MANUAL= LED ดับ

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

    // LED บน Nucleo F4 (PC13) เป็น active low
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, ledOn ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* ======================= UART Protocol ======================= */

/* ส่งสถานะปัจจุบันออก UART2 ไป NodeMCU (ไว้ต่อ Cloud/Dashboard) */
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
             "STATUS;MODE=%s;LIGHT=%d;LDR=%u;DIST=%.1f;NOISE=%s;INTR=%d\r\n",
             modeStr,
             g_lightOn,
             g_ldrValue,
             g_distanceCm,
             noiseStr,
             g_intrusion);

    HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), 50);
}

/* โครงรับคำสั่งจาก NodeMCU (ยังไม่ parse จริง ให้เป็น template) */
static void UART_ProcessRx(void)
{
    // แนวคิด:
    //  - อ่าน UART ใส่ buffer (ใช้ interrupt + ring buffer จะดีกว่า)
    //  - แกะข้อความรูปแบบ:
    //      CMD;MODE=AUTO
    //      CMD;MODE=AWAY
    //      CMD;LIGHT=ON
    //      CMD;LIGHT=OFF
    //      CMD;SET_TH_DARK=xxxx
    //      CMD;SET_TIMEOUT=xxxx
    //
    //  - จากนั้นเอามา set ตัวแปร g_mode, g_lightOn, LDR_TH_DARK, BASE_TIMEOUT_MS ฯลฯ
    //
    // ในตัวอย่างนี้ขอเว้นไว้ให้คุณไปต่อยอดเอง
}

/* ======================= END main.c ======================= */
