#include "capture_handler.h"
#include <stdint.h>
#include <stdio.h>
#include "float.h"
#include "stdlib.h"
#include "stm32h7xx_hal_tim.h"

#define DEBOUNCE_THRESHOLD_TICKS 100000000  // 500 ms @ 200MHz (500ms / 5ns = 100,000,000)
#define DMA_BUF_SIZE 1024

static TIM_HandleTypeDef *p_htim;
static volatile uint32_t last_avg_ticks = 0;
static volatile uint32_t last_avg_ns = 0;
static volatile uint32_t final_avg_ticks = 0;
static volatile uint32_t shared_avg_ticks = 0;
static volatile uint8_t event_ready = 0;
static float calibration_factor = 1.0f;
static uint32_t raw_delta_ns = 0;
static volatile uint32_t dma_buffer[DMA_BUF_SIZE] __attribute__((section(".axi_sram")));;
static volatile Capture_Data_t last_capture_result;

void Capture_Init(TIM_HandleTypeDef *htim) {
    p_htim = htim;
    for(int i=0; i<DMA_BUF_SIZE; i++) dma_buffer[i] = 0;
    HAL_TIM_IC_Start_DMA(p_htim, TIM_CHANNEL_1, (uint32_t*)dma_buffer, DMA_BUF_SIZE);
}

// Segédfüggvény a biztonságos átlagszámításhoz
static uint32_t Calculate_Average_Safe(uint32_t start_idx, uint32_t end_idx) {
    uint32_t diff;
    if (dma_buffer[end_idx] >= dma_buffer[start_idx]) {
        diff = dma_buffer[end_idx] - dma_buffer[start_idx];
    } else {
        diff = (0xFFFFFFFF - dma_buffer[start_idx]) + dma_buffer[end_idx] + 1;
    }
    return diff / (end_idx - start_idx);
}

//Half-transfer: dma_buffer[0..511], Transfer-complete: dma_buffer[512..1023]
void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        shared_avg_ticks = Calculate_Average_Safe(0, (DMA_BUF_SIZE / 2) - 1);
        event_ready = 1;
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        shared_avg_ticks = Calculate_Average_Safe(DMA_BUF_SIZE / 2, DMA_BUF_SIZE - 1);
        event_ready = 1;
    }
}

void Capture_Calibrate_By_Freq(uint32_t ref_hz) {
    if (ref_hz == 0) return;

    // Kiszámoljuk, hogy ez hány nanoszekundum lenne ideális esetben
    uint32_t ref_ns = 1000000000UL / ref_hz;
    uint32_t raw_val = Capture_GetLastRawNS();

    if (raw_val > 0) {
        calibration_factor = (float)ref_ns / (float)raw_val;
        printf("\r\n>>> FREKVENCIARA KALIBRALVA! (Ref: %lu Hz)\r\n", ref_hz);
        printf(">>> Uj faktor: %.6f\r\n", Capture_GetFactor());
    } else {
        printf("\r\n>>> HIBA: Nincs jel a kalibráláshoz!\r\n");
    }
}

void Capture_Process(void) {
    if (event_ready) {
        uint32_t ticks = shared_avg_ticks;
        if (ticks > 0) {
            // Frekvencia számítása (200 MHz / ticks)
            float freq = 200000000.0f / (float)ticks;
            raw_delta_ns = (uint32_t)(ticks * 5);
            
            last_capture_result.period_ns = (float)ticks * 5.0f * calibration_factor;
            last_capture_result.frequency_hz = (uint32_t)(freq / calibration_factor);

            if (last_capture_result.frequency_hz >= 1000000) {
                printf("[MEAS] Freq: %.3f MHz | ", (float)last_capture_result.frequency_hz / 1000000.0f);
            } else if (last_capture_result.frequency_hz >= 1000) {
                printf("[MEAS] Freq: %.2f kHz | ", (float)last_capture_result.frequency_hz / 1000.0f);
            } else {
                printf("[MEAS] Freq: %lu Hz | ", last_capture_result.frequency_hz);
            }

            printf("Period: %.1f ns | Ticks: %lu\r\n",  
                    last_capture_result.period_ns, ticks);
        }
        event_ready = 0;
    }
    /* Reakcióidő mérése
    if (event_ready) {
        // A diff_val mostmár pergésmentesített értékeket tartalmaz
        float reaction_ms = (float)diff_val * 5.0f / 1000000.0f; // Ticks -> ns -> ms

        printf("--- REAKCIOIDO: %.2f ms ---\r\n", reaction_ms);
        
        event_ready = 0;
    }*/
}

uint32_t Capture_GetLastRawNS(void) {
    return raw_delta_ns;
}

float Capture_GetFactor(void) {
    return calibration_factor;
}