#include <memory.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_rcc.h>
#include <stm32h7xx_hal_tim.h>
#include <stm32h7xx_hal_flash_ex.h>
#include <stm32h7xx_ll_rng.h>
#include <stm32h7xx_it.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include <cmsis_os2.h>

#include "cliutils/cli.h"
#include "cmds.h"
#include "ethernet/ethernet_lwip.h"
#include "flexptp/event.h"
#include "flexptp/logging.h"
#include "flexptp/profiles.h"
#include "flexptp/ptp_profile_presets.h"
#include "flexptp/settings_interface.h"
#include "standard_output/serial_io.h"
#include "standard_output/standard_output.h"
#include "capture_handler.h"
#include "ptp_tim_sync.h"

#define FLEXPTP_INITIAL_PROFILE ("gPTP")
#define TARGET_SYSCLK_MHZ (configCPU_CLOCK_HZ / 1000000)

// ---------------------------------------------------------------------------
// Periféria Handle-ök
// ---------------------------------------------------------------------------
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

osTimerId_t myLedTimerHandle;
const osTimerAttr_t myLedTimer_attributes = {
  .name = "myLedTimer"
};

// Prototípusok
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
//static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void vLedTimerCallback(void *argument);
void TIM3_SetFrequency(uint32_t frequency);
uint32_t Parse_Frequency(char *str);

// ---------------------------------------------------------------------------
// Órajel és Rendszerbeállítások
// ---------------------------------------------------------------------------
void init_pll() {
    RCC_OscInitTypeDef osc;
    RCC_ClkInitTypeDef clk;

    memset(&osc, 0, sizeof(RCC_OscInitTypeDef));
    memset(&clk, 0, sizeof(RCC_ClkInitTypeDef));

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON; 
    osc.HSIState = RCC_HSI_OFF;
    osc.CSIState = RCC_CSI_OFF;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;

    osc.PLL.PLLM = HSE_VALUE / 2000000;
    osc.PLL.PLLN = TARGET_SYSCLK_MHZ;
    osc.PLL.PLLFRACN = 0;
    osc.PLL.PLLP = 2;
    osc.PLL.PLLR = 2;
    osc.PLL.PLLQ = 4;

    osc.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    osc.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;

    HAL_RCC_OscConfig(&osc);

    clk.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                     RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1);
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.SYSCLKDivider = RCC_SYSCLK_DIV1;
    clk.AHBCLKDivider = RCC_HCLK_DIV2;
    clk.APB1CLKDivider = RCC_APB1_DIV2;
    clk.APB2CLKDivider = RCC_APB2_DIV2;
    clk.APB3CLKDivider = RCC_APB3_DIV2;
    clk.APB4CLKDivider = RCC_APB4_DIV2;

    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4);
}

void init_osc_and_clk() {
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    // VOS1-et használunk a stabil 200MHz körüli működéshez
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    init_pll();

    __HAL_RCC_HSI48_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_CSI_ENABLE();
    HAL_EnableCompensationCell();

    SystemCoreClockUpdate();
    HAL_SetTickFreq(HAL_TICK_FREQ_1KHZ);
}

void print_welcome_message() {
    MSGraw("\033[2J\033[H");
    MSG(ANSI_COLOR_BGREEN "Hi!" ANSI_COLOR_BYELLOW " This is a flexPTP demo merged with Custom Capture/PWM components.\n\n" ANSI_COLOR_RESET);
}

// ---------------------------------------------------------------------------
// Fő indító szál
// ---------------------------------------------------------------------------
void task_startup(void *arg) {
    // FlexPTP alrendszerek indítása
    cli_init();
    print_welcome_message();
    init_ethernet();
    cmd_init();

    // Az Ön egyedi periféria-logikájának inicializálása
    PtpTimSync_Init(&htim2);   // PPS (ETH_PPS_OUT) befogása TIM2_CH4-en -> TIM2 <-> PTP szinkron
    Capture_Init(&htim2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    uint8_t rx_data;
    char cmd_buffer[16];
    int idx = 0;
    char *msg = "Hello from FreeRTOS (flexPTP + Capture Active)!\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    for (;;) {
        // UART karakter fogadása (non-blocking) az Ön kódjából
        if (HAL_UART_Receive(&huart3, &rx_data, 1, 10) == HAL_OK) {
            HAL_UART_Transmit(&huart3, &rx_data, 1, 10); // Echo
            
            if (rx_data == '\r' || rx_data == '\n') {
                cmd_buffer[idx] = '\0';
                
                if (idx > 0) {
                    if (cmd_buffer[0] == 'C') {
                        uint32_t ref_hz = Parse_Frequency(&cmd_buffer[1]);
                        if (ref_hz > 0) {
                            Capture_Calibrate_By_Freq(ref_hz);
                        } else {
                            printf("\r\n>>> HIBA: Ervenytelen frekvencia formatum!\r\n");
                        }
                    }
                    else {
                        uint32_t new_freq = atoi(cmd_buffer);
                        if (new_freq > 0) {
                            TIM3_SetFrequency(new_freq);
                            printf("\r\n>>> Uj frekvencia: %lu Hz\r\n", new_freq);
                        }
                    }
                }
                idx = 0;
            } else if (idx < 15) {
                cmd_buffer[idx++] = rx_data;
            }
        }

        // Capture feldolgozó háttérfüggvénye
        Capture_Process();

        // 50ms késleltetés (hogy a flexPTP szálak is bőségesen kapjanak CPU időt)
        osDelay(50);
    }
}

// ---------------------------------------------------------------------------
// Inicializálások és Main
// ---------------------------------------------------------------------------
void init_randomizer() {
    __HAL_RCC_RNG_CLK_ENABLE();
    LL_RNG_Enable(RNG);
    while (!LL_RNG_IsActiveFlag_DRDY(RNG)) {}
    srand(LL_RNG_ReadRandData32(RNG));
}

int main(void) {
    SystemInit();
    HAL_Init();
    
    init_osc_and_clk();     // Kombinált órajel beállítás
    init_randomizer();
    HAL_MPU_Disable();      // flexPTP kompatibilis MPU kikapcsolás
    serial_io_init();

    // MX Perifériák inicializálása (Az Ön kódjából)
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM2_Init();
    //MX_USART3_UART_Init();
    MX_TIM3_Init();

    osKernelInitialize();

    // Szoftveres LED Timer létrehozása
    myLedTimerHandle = osTimerNew(vLedTimerCallback, osTimerPeriodic, NULL, &myLedTimer_attributes);
    if (myLedTimerHandle != NULL) {
        osTimerStart(myLedTimerHandle, 500); // 500 ms-os periódus a villogáshoz
    }

    // Egyetlen közös Init/Startup szál indítása (2048 szóméret a flexPTP miatt kötelező)
    osThreadAttr_t attr;
    memset(&attr, 0, sizeof(attr));
    attr.stack_size = 2048;
    attr.name = "init";
    attr.priority = (osPriority_t) osPriorityNormal;
    osThreadNew(task_startup, NULL, &attr);

    osKernelStart();

    for (;;) {}
}

// ---------------------------------------------------------------------------
// Callback-ek és egyedi segédfüggvények
// ---------------------------------------------------------------------------
void flexptp_user_event_cb(PtpUserEventCode uev) {
    switch (uev) {
    case PTP_UEV_INIT_DONE:
        ptp_load_profile(ptp_profile_preset_get(FLEXPTP_INITIAL_PROFILE));
        ptp_print_profile();
        ptp_log_enable(PTP_LOG_DEF, true);
        ptp_log_enable(PTP_LOG_BMCA, true);
        break;
    default:
        break;
    }
}

void vLedTimerCallback(void *argument) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
}

void TIM3_SetFrequency(uint32_t frequency) {
    if (frequency == 0) return;
    uint32_t timer_clk = PtpTimSync_GetNominalTimerClock(); // TIM2/TIM3 is az APB1 timer órajelről megy
    uint32_t arr_value = (timer_clk / (frequency)) - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim3, arr_value);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (arr_value + 1) / 2);
}

uint32_t Parse_Frequency(char *str) {
    char *endptr;
    float value = strtof(str, &endptr);
    while (*endptr == ' ') endptr++;
    if (*endptr == 'k' || *endptr == 'K') value *= 1000.0f;
    else if (*endptr == 'M' || *endptr == 'm') value *= 1000000.0f;
    return (uint32_t)value;
}

// FreeRTOS Specifikus Heap tömb szekció elhelyezéssel
uint8_t ucHeap[configTOTAL_HEAP_SIZE] __attribute__((section(".FreeRTOSHeapSection")));

void vApplicationTickHook(void) {
    HAL_IncTick();
}

void vApplicationIdleHook(void) {
    return;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        HAL_IncTick();
    }
}

// ---------------------------------------------------------------------------
// MX Generált Periféria Inicializáló Függvények
// ---------------------------------------------------------------------------
static void MX_TIM2_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4294967295;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim2);
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);
    HAL_TIM_IC_Init(&htim2);
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);
}

static void MX_TIM3_Init(void) {
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 399;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim3);
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 199;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_MspPostInit(&htim3);
}

/*static void MX_USART3_UART_Init(void) {
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&huart3);
    HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8);
    HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8);
    HAL_UARTEx_DisableFifoMode(&huart3);
}*/

static void MX_DMA_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}