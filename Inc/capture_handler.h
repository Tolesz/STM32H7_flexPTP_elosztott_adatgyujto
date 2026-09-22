#ifndef INC_CAPTURE_HANDLER_H_
#define INC_CAPTURE_HANDLER_H_

#include "stm32h7xx_hal.h"

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
uint32_t Capture_GetLastRawNS(void); // Utolsó mért nyers idő lekérése
float Capture_GetFactor(void); // Kalibrációs faktor lekérése
void Capture_GetData(Capture_Data_t *data);

#endif /* INC_CAPTURE_HANDLER_H_ */