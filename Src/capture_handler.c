#include "capture_handler.h"
#include <stdint.h>
#include <math.h>
#include "float.h"
#include "stdlib.h"
#include "stm32h7xx_hal_tim.h"

#include "ptp_tim_sync.h"
#include "standard_output/standard_output.h"

#define DMA_BUF_SIZE 1024
#define DEFAULT_GATE_MS 1000

#if CAPTURE_CHANNEL_COUNT < 1 || CAPTURE_CHANNEL_COUNT > 3
#error "CAPTURE_CHANNEL_COUNT must be between 1 and 3 (TIM2_CH4 is reserved for PPS synchronization)"
#endif

/*
 * Kapuidős frekvenciamérés, csatornánként (TIM2_CH1..CH3)
 *
 * Minden csatorna capture értékeit saját DMA körpufferbe írja a DMA. A Capture_Process()
 * (50 ms-onként) csatornánként a DMA aktuális írási pozíciójából megkeresi a legutolsó élt, és
 * visszafelé annyi periódust átlagol, amennyi a kapuidőbe belefér (legalább egyet). Így:
 *   - gyors jelnél sok periódus átlagolódik (pontos eredmény, kapuidőnként egy kiírás),
 *   - lassú jelnél (pl. 1 PPS) minden új él után azonnal van eredmény (1 periódus).
 * Mivel minden csatorna ugyanazt a TIM2 számlálót használja, egyetlen PPS-szinkron mindet kiszolgálja,
 * és az élek PTP időbélyegei csatornák között közvetlenül összevethetők.
 * Korlát: egy periódus nem lehet hosszabb a TIM2 32 bites átfordulásánál (240 MHz-en ~17.9 s).
 * TIM2 csatornák: CH1: PA0, CH2: PB3, CH3: PB10
 */

typedef struct {
    uint32_t tim_channel;     // TIM_CHANNEL_x
    uint16_t dma_id;          // TIM_DMA_ID_CCx
    bool enabled;             // kiírás engedélyezve
    bool resync;              // újraindítás kérése (bekapcsoláskor)
    uint32_t last_wr_idx;     // DMA írási pozíció az előző feldolgozáskor
    uint32_t last_seen_edge;  // az előző feldolgozáskor látott legutolsó él
    uint32_t last_print_edge; // a legutóbbi kiíráshoz tartozó él
    uint32_t valid_samples;   // a pufferben lévő érvényes befogások száma (max. DMA_BUF_SIZE)
    bool printed_once;
} CaptureChannel;

static const uint32_t sTimChannels[3] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};
static const uint16_t sDmaIds[3] = {TIM_DMA_ID_CC1, TIM_DMA_ID_CC2, TIM_DMA_ID_CC3};

static TIM_HandleTypeDef *p_htim;
static float calibration_factor = 1.0f;
static uint32_t raw_delta_ns = 0; // CH1 utolsó nyers periódusa (kalibrációhoz)
static volatile uint32_t dma_buffer[CAPTURE_CHANNEL_COUNT][DMA_BUF_SIZE] __attribute__((section(".axi_sram")));
static volatile Capture_Data_t last_capture_result;

static uint32_t gate_ms = DEFAULT_GATE_MS;
static CaptureChannel channels[CAPTURE_CHANNEL_COUNT];

void Capture_Init(TIM_HandleTypeDef *htim) {
    p_htim = htim;

    for (int c = 0; c < CAPTURE_CHANNEL_COUNT; c++) {
        CaptureChannel *ch = &channels[c];
        ch->tim_channel = sTimChannels[c];
        ch->dma_id = sDmaIds[c];
        ch->enabled = true;
        ch->resync = true;

        for (int i = 0; i < DMA_BUF_SIZE; i++) dma_buffer[c][i] = 0;
        if (HAL_TIM_IC_Start_DMA(p_htim, ch->tim_channel, (uint32_t *)dma_buffer[c], DMA_BUF_SIZE) != HAL_OK) {
            MSG("[MEAS] HIBA: a CH%u capture DMA nem indult el!\n", c + 1);
        }
    }
}

// Ezt hívja a DMA half/transfer-complete (CH1..CH3) ÉS a PPS capture megszakítás (CH4) is.
// A mérőcsatornák adatait nem itt dolgozzuk fel, hanem a Capture_Process() olvassa a DMA pufferekből.
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

void Capture_SetChannelEnabled(uint32_t ch, bool en) {
    if (ch < 1 || ch > CAPTURE_CHANNEL_COUNT) return;
    CaptureChannel *c = &channels[ch - 1];
    if (en && !c->enabled) {
        c->resync = true; // a kikapcsolt idő alatti régi élekből ne számoljunk
    }
    c->enabled = en;
}

bool Capture_IsChannelEnabled(uint32_t ch) {
    if (ch < 1 || ch > CAPTURE_CHANNEL_COUNT) return false;
    return channels[ch - 1].enabled;
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

// Fixpontos szám formázása szöveggé (az embfmt %.Nf kerekítési hibáját elkerülve: N >= 2 esetén
// pl. 0.9999 -> "1.990" lesz belőle). dec: tizedesjegyek száma (0..9).
static const char *fmt_fixed(char *buf, size_t len, double v, int dec) {
    static const uint32_t pow10[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};
    if (dec < 0) dec = 0;
    if (dec > 9) dec = 9;

    bool neg = v < 0;
    if (neg) v = -v;

    uint64_t scaled = (uint64_t)llround(v * (double)pow10[dec]);
    uint64_t ip = scaled / pow10[dec];
    uint32_t fp = (uint32_t)(scaled % pow10[dec]);

    // számjegyek visszafelé
    char tmp[32];
    int n = 0;
    for (int i = 0; i < dec; i++) {
        tmp[n++] = (char)('0' + fp % 10);
        fp /= 10;
    }
    if (dec > 0) tmp[n++] = '.';
    do {
        tmp[n++] = (char)('0' + ip % 10);
        ip /= 10;
    } while (ip > 0 && n < (int)sizeof(tmp) - 1);
    if (neg && scaled != 0) tmp[n++] = '-';

    size_t k = 0;
    while (n > 0 && k < len - 1) buf[k++] = tmp[--n];
    buf[k] = '\0';
    return buf;
}

static inline uint32_t buf_at(uint32_t c, uint32_t newest_idx, uint32_t back) {
    return dma_buffer[c][(newest_idx + DMA_BUF_SIZE - back) % DMA_BUF_SIZE];
}

static void Process_Channel(uint32_t c) {
    CaptureChannel *ch = &channels[c];

    // DMA aktuális írási pozíciója (a következő írandó elem indexe)
    uint32_t ndtr = __HAL_DMA_GET_COUNTER(p_htim->hdma[ch->dma_id]);
    uint32_t wr_idx = (DMA_BUF_SIZE - ndtr) % DMA_BUF_SIZE;
    uint32_t newest = (wr_idx + DMA_BUF_SIZE - 1) % DMA_BUF_SIZE;
    uint32_t last_edge = dma_buffer[c][newest];

    if (ch->resync) {
        // (Újra)indulás: csak az innentől érkező élekből számolunk
        ch->resync = false;
        ch->last_wr_idx = wr_idx;
        ch->last_seen_edge = last_edge;
        ch->valid_samples = 0;
        ch->printed_once = false;
        return;
    }

    // Új élek érkeztek-e? (az index-különbség körbefordulhat gyors jelnél, ezért az értéket is nézzük)
    uint32_t new_samples = (wr_idx + DMA_BUF_SIZE - ch->last_wr_idx) % DMA_BUF_SIZE;
    bool new_edge = (new_samples > 0) || (last_edge != ch->last_seen_edge);
    if (new_samples == 0 && new_edge) new_samples = DMA_BUF_SIZE;
    ch->last_wr_idx = wr_idx;
    ch->last_seen_edge = last_edge;

    if (!new_edge || !ch->enabled) return;

    ch->valid_samples += new_samples;
    if (ch->valid_samples > DMA_BUF_SIZE) ch->valid_samples = DMA_BUF_SIZE;
    if (ch->valid_samples < 2) return; // legalább egy teljes periódus kell

    double f_tim = PtpTimSync_GetTimerFreq();
    uint32_t gate_ticks = (uint32_t)(f_tim * gate_ms / 1000.0);

    // Kiírás legfeljebb kapuidőnként egyszer (10% tűréssel, hogy 1 Hz-es jel 1 s-os kapunál minden élnél kiíródjon)
    if (ch->printed_once && (last_edge - ch->last_print_edge) < gate_ticks - gate_ticks / 10) return;

    // Visszafelé annyi periódus, amennyi a kapuidőbe belefér (legalább 1)
    uint32_t max_periods = ch->valid_samples - 1;
    uint32_t periods = 1;
    while (periods < max_periods && (last_edge - buf_at(c, newest, periods + 1)) <= gate_ticks) {
        periods++;
    }

    uint32_t total = last_edge - buf_at(c, newest, periods); // 32 bites átfordulás kezelve
    if (total == 0) return;

    double ticks = (double)total / (double)periods;
    double period_ns = ticks * 1e9 / f_tim;
    double freq = 1e9 / period_ns / calibration_factor;

    if (c == 0) {
        raw_delta_ns = (uint32_t)period_ns;
        last_capture_result.raw_ticks = (uint32_t)ticks;
        last_capture_result.period_ns = (float)(period_ns * calibration_factor);
        last_capture_result.frequency_hz = (uint32_t)freq;
    }

    ch->last_print_edge = last_edge;
    ch->printed_once = true;

    double f_unit = freq;
    const char *unit = "Hz";
    if (freq >= 1000000.0) {
        f_unit = freq / 1000000.0;
        unit = "MHz";
    } else if (freq >= 1000.0) {
        f_unit = freq / 1000.0;
        unit = "kHz";
    }

    // Frekvencia: egész rész + 9 tizedesjegy (a PTP időbélyeghez hasonlóan egészként kiírva,
    // így a tizedes rész felfelé kerekítése helyesen átvisz az egész részbe)
    uint64_t f_scaled = (uint64_t)llround(f_unit * 1e9);
    uint32_t f_int = (uint32_t)(f_scaled / 1000000000ULL);
    uint32_t f_frac = (uint32_t)(f_scaled % 1000000000ULL);

    // Periódus: 10 jegyű egész rész (vezető nullákkal) + 1 tizedesjegy [ns].
    // Az egész rész két darabban (felső jegyek + alsó 9 jegy), mert 4.29 s felett nem fér el 32 biten.
    uint64_t p_scaled = (uint64_t)llround(period_ns * calibration_factor * 10.0);
    uint64_t p_int = p_scaled / 10;
    uint32_t p_hi = (uint32_t)(p_int / 1000000000ULL);
    uint32_t p_lo = (uint32_t)(p_int % 1000000000ULL);
    uint32_t p_frac = (uint32_t)(p_scaled % 10);

    char s_ticks[24];
    MSG("[MEAS CH%u] Freq: %u.%09u %s | Period: %u%09u.%u ns | Ticks: %s",
        c + 1, f_int, f_frac, unit, p_hi, p_lo, p_frac,
        fmt_fixed(s_ticks, sizeof(s_ticks), ticks, 2));

    // A helyi óra (TIM2) eltérése a PTP időtől (mastertől): + = az STM32 órája gyorsabb
    if (PtpTimSync_IsLocked()) {
        double nominal = (double)PtpTimSync_GetNominalTimerClock();
        char s_ppm[24];
        MSG(" | Clk: %s ppm", fmt_fixed(s_ppm, sizeof(s_ppm), (f_tim - nominal) / nominal * 1e6, 3));
    }

    // Az utolsó él időbélyege a közös (PTP) időskálán
    TimestampU ts;
    if (PtpTimSync_TicksToPtp(last_edge, &ts)) {
        MSG(" | PTP: %u.%09u\n", (uint32_t)ts.sec, ts.nanosec);
    } else {
        MSG(" | PTP: nincs szinkron\n");
    }
}

void Capture_Process(void) {
    for (uint32_t c = 0; c < CAPTURE_CHANNEL_COUNT; c++) {
        Process_Channel(c);
    }
}

uint32_t Capture_GetLastRawNS(void) {
    return raw_delta_ns;
}

float Capture_GetFactor(void) {
    return calibration_factor;
}
