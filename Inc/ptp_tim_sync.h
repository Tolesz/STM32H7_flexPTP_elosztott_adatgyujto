#ifndef INC_PTP_TIM_SYNC_H_
#define INC_PTP_TIM_SYNC_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"
#include "flexptp/timeutils.h"

/*
 * TIM2 <-> PTP óra szinkronizáció
 *
 * Az ETH MAC PTP órájából képzett 1PPS jelet (ETH_PPS_OUT, PB5) egy átkötéssel
 * a TIM2 egy szabad input capture csatornájára vezetjük (alapértelmezés: CH4, PA3 = A0).
 * A PPS felfutó éle minden PTP másodpercben ugyanott, x.536870912 s-nál (2^29 ns) érkezik
 * (ETH digitális átfordulás, 1 Hz-es fix PPS mód), így a befogott TIM2 számlálóértékekből:
 *   - megmérjük, hány TIM2 tick esik egy PTP másodpercre (a TIM2 valódi frekvenciája
 *     a hálózati időhöz képest),
 *   - és bármely TIM2 capture értéket PTP időbélyeggé alakíthatunk
 *     (legutóbbi PPS él + eltelt tickek / frekvencia).
 */

// PPS bemenet csatornája (TIM2_CH4)
#define PTS_TIM_CHANNEL         TIM_CHANNEL_4
#define PTS_TIM_ACTIVE_CHANNEL  HAL_TIM_ACTIVE_CHANNEL_4

// 1: a PPS-t belső trigger bemenetről (ITRx -> TRC -> IC4) fogjuk be, átkötés nem kell
// 0: külső átkötés PB5 (ETH_PPS_OUT) -> PA3 (TIM2_CH4)
#define PTS_USE_INTERNAL_ITR    1
#define PTS_INTERNAL_ITR        TIM_TS_ITR4

// Külső PPS láb (csak PTS_USE_INTERNAL_ITR == 0 esetén): TIM2_CH4 @ PA3 (A0), AF1
#define PTS_GPIO_PORT           GPIOA
#define PTS_GPIO_PIN            GPIO_PIN_3
#define PTS_GPIO_AF             GPIO_AF1_TIM2
#define PTS_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()

void PtpTimSync_Init(TIM_HandleTypeDef *htim);      // PPS capture csatorna indítása
void PtpTimSync_OnPpsCapture(uint32_t capture);    // TIM2 capture callbackből hívandó (ISR)
void PtpTimSync_RegisterCli(void);                  // "tsync" CLI parancs regisztrálása

uint32_t PtpTimSync_GetNominalTimerClock(void);     // TIM2 névleges órajele az RCC alapján [Hz]
double PtpTimSync_GetTimerFreq(void);               // TIM2 tick / PTP másodperc (nem szinkron: névleges)
bool PtpTimSync_IsLocked(void);                     // van-e érvényes PPS referencia
bool PtpTimSync_TicksToPtp(uint32_t ticks, TimestampU *pTs); // TIM2 érték -> PTP idő

#endif /* INC_PTP_TIM_SYNC_H_ */
