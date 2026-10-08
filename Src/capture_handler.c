#include "capture_handler.h"
#include <stdint.h>
#include "float.h"
#include "stdlib.h"
#include "stm32h7xx_hal_tim.h"

#include "ptp_tim_sync.h"
#include "standard_output/standard_output.h"

#define DMA_BUF_SIZE 1024
#define DEFAULT_GATE_MS 1000

/*
 * Kapuidős frekvenciamérés
 *
 * A DMA körpufferbe folyamatosan írja a TIM2_CH1 capture értékeket. A Capture_Process()
 * (50 ms-onként) a DMA aktuális írási pozíciójából megkeresi a legutolsó élt, és visszafelé
 * annyi periódust átlagol, amennyi a kapuidőbe belefér (legalább egyet). Így:
 *   - gyors jelnél sok periódus átlagolódik (pontos eredmény, kapuidőnként egy kiírás),
 *   - lassú jelnél (pl. 1 PPS) minden új él után azonnal van eredmény (1 periódus).
 * Korlát: egy periódus nem lehet hosszabb a TIM2 32 bites átfordulásánál (240 MHz-en ~17.9 s).
 */

static TIM_HandleTypeDef *p_htim;
static float calibration_factor = 1.0f;
static uint32_t raw_delta_ns = 0;
static volatile uint32_t dma_buffer[DMA_BUF_SIZE] __attribute__((section(".axi_sram")));;
static volatile Capture_Data_t last_capture_result;

static uint32_t gate_ms = DEFAULT_GATE_MS;
static uint32_t last_wr_idx = 0;       // DMA írási pozíció az előző feldolgozáskor
static uint32_t last_seen_edge = 0;    // az előző feldolgozáskor látott legutolsó él
static uint32_t last_print_edge = 0;   // a legutóbbi kiíráshoz tartozó él
static uint32_t valid_samples = 0;     // a pufferben lévő érvényes befogások száma (max. DMA_BUF_SIZE)
static bool printed_once = false;

void Capture_Init(TIM_HandleTypeDef *htim) {
    p_htim = htim;
    for(int i=0; i<DMA_BUF_SIZE; i++) dma_buffer[i] = 0;
    HAL_TIM_IC_Start_DMA(p_htim, TIM_CHANNEL_1, (uint32_t*)dma_buffer, DMA_BUF_SIZE);
}

// Ezt hívja a DMA transfer-complete (CH1) ÉS a PPS capture megszakítás (CH4) is.
// A CH1 adatait nem itt dolgozzuk fel, hanem a Capture_Process() olvassa a DMA pufferből.
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM2) return;

    if (htim->Channel == PTS_TIM_ACTIVE_CHANNEL) {
        PtpTimSync_OnPpsCapture(HAL_TIM_ReadCapturedValue(htim, PTS_TIM_CHANNEL));
    }
}

void Capture_SetGateMs(uint32_t ms) {
    if (ms < 10) ms = 10;
    gate_ms = ms;
}

uint32_t Capture_GetGateMs(void) {
    return gate_ms;
}

void Capture_Calibrate_By_Freq(uint32_t ref_hz) {
    if (ref_hz == 0) return;

    // Kiszámoljuk, hogy ez hány nanoszekundum lenne ideális esetben
    uint32_t ref_ns = 1000000000UL / ref_hz;
    uint32_t raw_val = Capture_GetLastRawNS();

    if (raw_val > 0) {
        calibration_factor = (float)ref_ns / (float)raw_val;
        MSG("\n>>> FREKVENCIARA KALIBRALVA! (Ref: %u Hz)\n", ref_hz);
        MSG(">>> Uj faktor: %.6f\n", Capture_GetFactor());
    } else {
        MSG("\n>>> HIBA: Nincs jel a kalibralashoz!\n");
    }
}

static inline uint32_t buf_at(uint32_t newest_idx, uint32_t back) {
    return dma_buffer[(newest_idx + DMA_BUF_SIZE - back) % DMA_BUF_SIZE];
}

void Capture_Process(void) {
    // DMA aktuális írási pozíciója (a következő írandó elem indexe)
    uint32_t ndtr = __HAL_DMA_GET_COUNTER(p_htim->hdma[TIM_DMA_ID_CC1]);
    uint32_t wr_idx = (DMA_BUF_SIZE - ndtr) % DMA_BUF_SIZE;
    uint32_t newest = (wr_idx + DMA_BUF_SIZE - 1) % DMA_BUF_SIZE;
    uint32_t last_edge = dma_buffer[newest];

    // Új élek érkeztek-e? (az index-különbség körbefordulhat gyors jelnél, ezért az értéket is nézzük)
    uint32_t new_samples = (wr_idx + DMA_BUF_SIZE - last_wr_idx) % DMA_BUF_SIZE;
    bool new_edge = (new_samples > 0) || (last_edge != last_seen_edge);
    if (new_samples == 0 && new_edge) new_samples = DMA_BUF_SIZE;
    last_wr_idx = wr_idx;
    last_seen_edge = last_edge;

    if (!new_edge) return;

    valid_samples += new_samples;
    if (valid_samples > DMA_BUF_SIZE) valid_samples = DMA_BUF_SIZE;
    if (valid_samples < 2) return; // legalább egy teljes periódus kell

    double f_tim = PtpTimSync_GetTimerFreq();
    uint32_t gate_ticks = (uint32_t)(f_tim * gate_ms / 1000.0);

    // Kiírás legfeljebb kapuidőnként egyszer (10% tűréssel, hogy 1 Hz-es jel 1 s-os kapunál minden élnél kiíródjon)
    if (printed_once && (last_edge - last_print_edge) < gate_ticks - gate_ticks / 10) return;

    // Visszafelé annyi periódus, amennyi a kapuidőbe belefér (legalább 1)
    uint32_t max_periods = valid_samples - 1;
    uint32_t periods = 1;
    while (periods < max_periods && (last_edge - buf_at(newest, periods + 1)) <= gate_ticks) {
        periods++;
    }

    uint32_t total = last_edge - buf_at(newest, periods); // 32 bites átfordulás kezelve
    if (total == 0) return;

    double ticks = (double)total / (double)periods;
    double period_ns = ticks * 1e9 / f_tim;
    double freq = 1e9 / period_ns / calibration_factor;

    raw_delta_ns = (uint32_t)period_ns;
    last_capture_result.raw_ticks = (uint32_t)ticks;
    last_capture_result.period_ns = (float)(period_ns * calibration_factor);
    last_capture_result.frequency_hz = (uint32_t)freq;

    last_print_edge = last_edge;
    printed_once = true;

    if (freq >= 1000000.0) {
        MSG("[MEAS] Freq: %.3f MHz | ", freq / 1000000.0);
    } else if (freq >= 1000.0) {
        MSG("[MEAS] Freq: %.3f kHz | ", freq / 1000.0);
    } else {
        MSG("[MEAS] Freq: %.3f Hz | ", freq);
    }

    MSG("Period: %.1f ns | Ticks: %.2f", period_ns * calibration_factor, ticks);

    // Az utolsó él időbélyege a közös (PTP) időskálán
    TimestampU ts;
    if (PtpTimSync_TicksToPtp(last_edge, &ts)) {
        MSG(" | PTP: %u.%09u\n", (uint32_t)ts.sec, ts.nanosec);
    } else {
        MSG(" | PTP: nincs szinkron\n");
    }
}

uint32_t Capture_GetLastRawNS(void) {
    return raw_delta_ns;
}

float Capture_GetFactor(void) {
    return calibration_factor;
}
