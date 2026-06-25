/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#include "ws2812.h"
#include "bsp_fdcan.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct {
    uint8_t node_id;
    float pos_deg;
    float vel_rpm;
    uint32_t rx_count;
    uint32_t last_rx_ms;
} JC_MotorState;

#define NODE_LEADER_GRIPPER           1   // CAN1
#define NODE_LEADER_WRIST             3   // CAN1
#define NODE_FOLLOWER_GRIPPER         2   // CAN2
#define NODE_FOLLOWER_WRIST           4   // CAN2

static JC_MotorState leader_gripper   = {NODE_LEADER_GRIPPER,   0};
static JC_MotorState leader_wrist     = {NODE_LEADER_WRIST,     0};
static JC_MotorState follower_gripper = {NODE_FOLLOWER_GRIPPER, 0};
static JC_MotorState follower_wrist   = {NODE_FOLLOWER_WRIST,   0};

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 1kHz control flag from TIM2
volatile uint8_t loop_1khz_flag = 0;

// timing/debug
static uint32_t loop_count = 0;
static uint32_t can_tx_drop = 0;
static uint32_t can_rx_other = 0;

// virtual coupling gains
#define CTRL_DIV 1 // 1=1khz,2=500hz, 4=250hz, 10=100hz, 100=10hz,

static float K_COUPLE = 0.16f;
static float D_COUPLE = 0.012f;
static float TAU_MAX  = 0.25f;

static const float DEG2RAD = 0.01745329252f;
static const float RPM2RAD = 0.10471975512f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ===== WS2812 LED status helpers =====
static inline void led_yellow(void){ WS2812_Ctrl(60, 60, 0); }
static inline void led_red(void)   { WS2812_Ctrl(80, 0, 0);  }
static inline void led_green(void) { WS2812_Ctrl(0, 80, 0);  }

// ===== UART1 logging =====
static char logbuf[192];

static void log_uart1(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)s, (uint16_t)strlen(s), 100);
}

static void logf_uart1(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(logbuf, sizeof(logbuf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    size_t len = strnlen(logbuf, sizeof(logbuf));
    HAL_UART_Transmit(&huart1, (uint8_t*)logbuf, (uint16_t)len, 100);
}

// ===== Power 1 enable: PC14 =====
#define MOTOR_EN_PORT GPIOC
#define MOTOR_EN_PIN_1  GPIO_PIN_14
#define MOTOR_EN_PIN_2  GPIO_PIN_13

static inline void arm_enable(uint8_t en)
{
    HAL_GPIO_WritePin(MOTOR_EN_PORT, MOTOR_EN_PIN_1, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_EN_PORT, MOTOR_EN_PIN_2, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// ===== JC protocol helpers =====
static inline uint16_t jc_tx_id(uint8_t node_id){ return (uint16_t)(0x600 + node_id); }

static inline float clampf(float x, float lo, float hi)
{
    if (x > hi) return hi;
    if (x < lo) return lo;
    return x;
}

static inline int16_t be_s16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] << 8 | p[1]);
}

static inline int32_t be_s24(const uint8_t *p)
{
    int32_t v = ((int32_t)p[0] << 16) | ((int32_t)p[1] << 8) | p[2];
    if (v & 0x00800000) v |= 0xFF000000;
    return v;
}

static int jc_send8(FDCAN_HandleTypeDef *hcan, uint16_t std_id, const uint8_t d[8])
{
    int r = fdcanx_send_data(hcan, std_id, (uint8_t*)d, 8);
    if (r != 0) can_tx_drop++;
    return r;
}

static int jc_set_mode(FDCAN_HandleTypeDef *hcan, uint8_t node_id, uint16_t mode)
{
    uint8_t d[8] = {0x2B, 0x00, 0x60, 0x00, 0, 0, 0, 0};
    d[4] = (uint8_t)(mode >> 8);
    d[5] = (uint8_t)(mode & 0xFF);
    return jc_send8(hcan, jc_tx_id(node_id), d);
}

static int jc_enter_closed_loop(FDCAN_HandleTypeDef *hcan, uint8_t node_id)
{
    uint8_t d[8] = {0x2B, 0x00, 0xA2, 0x00, 0x00, 0x01, 0x00, 0x00};
    return jc_send8(hcan, jc_tx_id(node_id), d);
}

static int jc_idle(FDCAN_HandleTypeDef *hcan, uint8_t node_id)
{
    uint8_t d[8] = {0x2B, 0x00, 0xA0, 0x00, 0x00, 0x01, 0x00, 0x00};
    return jc_send8(hcan, jc_tx_id(node_id), d);
}

static int jc_set_torque_nm(FDCAN_HandleTypeDef *hcan, uint8_t node_id, float torque_nm)
{
    torque_nm = clampf(torque_nm, -TAU_MAX, TAU_MAX);
    int16_t v = (int16_t)(torque_nm * 100.0f); // JC: Nm * 100

    uint8_t d[8] = {0x2B, 0x00, 0x20, 0x00, 0, 0, 0, 0};
    d[4] = (uint8_t)((v >> 8) & 0xFF);
    d[5] = (uint8_t)(v & 0xFF);
    return jc_send8(hcan, jc_tx_id(node_id), d);
}

void can_parse_feedback(FDCAN_HandleTypeDef *hcan, uint16_t rx_id, uint8_t *d, uint8_t len)
{
    if (len != 8) return;

    JC_MotorState *m = NULL;

    if (hcan == &hfdcan1 && rx_id == 0x580 + NODE_LEADER_GRIPPER) {
        m = &leader_gripper;
    }
    else if (hcan == &hfdcan1 && rx_id == 0x580 + NODE_LEADER_WRIST) {
        m = &leader_wrist;
    }
    else if (hcan == &hfdcan2 && rx_id == 0x580 + NODE_FOLLOWER_GRIPPER) {
        m = &follower_gripper;
    }
    else if (hcan == &hfdcan2 && rx_id == 0x580 + NODE_FOLLOWER_WRIST) {
        m = &follower_wrist;
    }
    else {
        can_rx_other++;
        return;
    }

    if (d[0] == 0x2A) {
        int32_t pos_raw = be_s24(&d[1]);
        int16_t vel_raw = be_s16(&d[4]);

        m->pos_deg = (float)pos_raw / 100.0f;
        m->vel_rpm = (float)vel_raw / 100.0f;
        m->rx_count++;
        m->last_rx_ms = HAL_GetTick();
    }
}

static void fatal(const char *msg)
{
    led_red();
    logf_uart1("[FATAL] %s\r\n", msg);
    arm_enable(0);
    __disable_irq();
    while (1) { HAL_Delay(1000); }
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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_USART10_UART_Init();
  MX_SPI6_Init();
  /* USER CODE BEGIN 2 */
  led_yellow();
  log_uart1("\r\n[BOOT] H725/H723 JC gripper 1kHz force-feedback demo\r\n");

  arm_enable(0);
  HAL_Delay(50);

  log_uart1("[INIT] CAN...\r\n");
  bsp_can_init();
  HAL_Delay(50);

  // If bsp_can_init() does not already enable RX interrupt, this makes it explicit.
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  HAL_TIM_Base_Start_IT(&htim2);

  log_uart1("[INIT] motor power enable\r\n");
  arm_enable(1);
  HAL_Delay(500);

  log_uart1("[INIT] idle motors\r\n");
  jc_idle(&hfdcan1, NODE_LEADER_GRIPPER);
  HAL_Delay(50);
  jc_idle(&hfdcan1, NODE_LEADER_WRIST);
  HAL_Delay(50);
  jc_idle(&hfdcan2, NODE_FOLLOWER_GRIPPER);
  HAL_Delay(50);
  jc_idle(&hfdcan2, NODE_FOLLOWER_WRIST);
  HAL_Delay(50);

  log_uart1("[INIT] closed loop\r\n");
  if (jc_enter_closed_loop(&hfdcan1, NODE_LEADER_GRIPPER) != 0) fatal("closed leader gripper");
  HAL_Delay(50);
  if (jc_enter_closed_loop(&hfdcan1, NODE_LEADER_WRIST) != 0) fatal("closed leader wrist");
  HAL_Delay(50);
  if (jc_enter_closed_loop(&hfdcan2, NODE_FOLLOWER_GRIPPER) != 0) fatal("closed follower gripper");
  HAL_Delay(50);
  if (jc_enter_closed_loop(&hfdcan2, NODE_FOLLOWER_WRIST) != 0) fatal("closed follower wrist");
  HAL_Delay(50);

  log_uart1("[INIT] torque mode\r\n");
  jc_set_mode(&hfdcan1, NODE_LEADER_GRIPPER, 0);   HAL_Delay(50);
  jc_set_mode(&hfdcan1, NODE_LEADER_WRIST, 0);     HAL_Delay(50);
  jc_set_mode(&hfdcan2, NODE_FOLLOWER_GRIPPER, 0); HAL_Delay(50);
  jc_set_mode(&hfdcan2, NODE_FOLLOWER_WRIST, 0);   HAL_Delay(50);

  jc_set_torque_nm(&hfdcan1, NODE_LEADER_GRIPPER, 0.0f);    HAL_Delay(20);
  jc_set_torque_nm(&hfdcan1, NODE_LEADER_WRIST, 0.0f);      HAL_Delay(20);
  jc_set_torque_nm(&hfdcan2, NODE_FOLLOWER_GRIPPER, 0.0f);  HAL_Delay(20);
  jc_set_torque_nm(&hfdcan2, NODE_FOLLOWER_WRIST, 0.0f);    HAL_Delay(20);


  led_green();
  logf_uart1("[OK] 1kHz demo running. leader=%d follower=%d\r\n", NODE_LEADER_GRIPPER, NODE_FOLLOWER_GRIPPER);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_log_ms = HAL_GetTick();
  while (1)
  {
    if (!loop_1khz_flag) continue;
    loop_1khz_flag = 0;
    loop_count++;
    if ((loop_count % CTRL_DIV) != 0)
    {
        continue;
    }

    float err_g_deg = leader_gripper.pos_deg - follower_gripper.pos_deg;
    float dq_g = leader_gripper.vel_rpm * RPM2RAD - follower_gripper.vel_rpm * RPM2RAD;
    float tau_g = -K_COUPLE * (err_g_deg * DEG2RAD) - D_COUPLE * dq_g;
    tau_g = clampf(tau_g, -TAU_MAX, TAU_MAX);

    float err_w_deg = leader_wrist.pos_deg - follower_wrist.pos_deg;
    float dq_w = leader_wrist.vel_rpm * RPM2RAD - follower_wrist.vel_rpm * RPM2RAD;
    float tau_w = -K_COUPLE * (err_w_deg * DEG2RAD) - D_COUPLE * dq_w;
    tau_w = clampf(tau_w, -TAU_MAX, TAU_MAX);

    int r1 = jc_set_torque_nm(&hfdcan1, NODE_LEADER_GRIPPER, tau_g);
    int r2 = jc_set_torque_nm(&hfdcan2, NODE_FOLLOWER_GRIPPER, -tau_g);

    int r3 = jc_set_torque_nm(&hfdcan1, NODE_LEADER_WRIST, tau_w);
    int r4 = jc_set_torque_nm(&hfdcan2, NODE_FOLLOWER_WRIST, -tau_w);

    // 10Hz debug only. Do not print at 1kHz.
    uint32_t now = HAL_GetTick();
    if ((now - last_log_ms) >= 100)
    {
        last_log_ms = now;

        logf_uart1(
            "G %.1f %.1f tau %.3f | W %.1f %.1f tau %.3f | rx %lu %lu %lu %lu | r %d %d %d %d\r\n",
            leader_gripper.pos_deg,
            follower_gripper.pos_deg,
            tau_g,
            leader_wrist.pos_deg,
            follower_wrist.pos_deg,
            tau_w,
            leader_gripper.rx_count,
            follower_gripper.rx_count,
            leader_wrist.rx_count,
            follower_wrist.rx_count,
            r1, r2, r3, r4
        );
    }

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        loop_1khz_flag = 1;
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
