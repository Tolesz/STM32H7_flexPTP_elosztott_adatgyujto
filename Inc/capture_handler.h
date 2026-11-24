#ifndef INC_CAPTURE_HANDLER_H_
#define INC_CAPTURE_HANDLER_H_

#include <stdbool.h>

#include "stm32h7xx_hal.h"

// Mért csatornák száma a TIM2-n (1..3). A TIM2_CH4 a PPS-szinkroné (ptp_tim_sync).
//   CH1: PA0 (DMA1_Stream1), CH2: PB3 (DMA1_Stream2), CH3: PB10 (DMA1_Stream3) - mind AF1
#define CAPTURE_CHANNEL_COUNT 3

// Struktúra az esemény adatainak
typedef struct {
    uint32_t capture_value;
    uint32_t timestamp;
} CaptureEvent_t;

typedef struct {
    uint32_t raw_ticks;
    uint32_t frequency_hz;
    float period_ns;
    float duty_cycle;
} Capture_Data_t;


void Capture_Init(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void Capture_Calibrate_By_Freq(uint32_t ref_hz); // Frekvencia alapján történő kalibráció  
void Capture_Process(void); // Adatok kiírása
void Capture_SetGateMs(uint32_t ms); // Kapuidő beállítása [ms] (ennyi időnyi periódust átlagol)
uint32_t Capture_GetGateMs(void);
void Capture_SetChannelEnabled(uint32_t ch, bool en); // Csatorna mérésének ki/bekapcsolása (ch: 1..CAPTURE_CHANNEL_COUNT)
bool Capture_IsChannelEnabled(uint32_t ch);
uint32_t Capture_GetLastRawNS(void); // Utolsó mért nyers idő lekérése
float Capture_GetFactor(void); // Kalibrációs faktor lekérése
void Capture_GetData(Capture_Data_t *data);

#endif /* INC_CAPTURE_HANDLER_H_ */