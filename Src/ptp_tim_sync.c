#include "ptp_tim_sync.h"

#include <math.h>

#include "cliutils/cli.h"
#include "standard_output/standard_output.h"

#define NSEC_PER_SEC            (1000000000LL)
#define PTS_MAX_DEVIATION_PPM   (1000.0)   // ennél nagyobb eltérésű PPS periódust eldobunk (pl. PTP óraugrás)
#define PTS_LOCK_COUNT          (3)        // ennyi egymást követő érvényes PPS után tekintjük szinkronnak
#define PTS_FREQ_FILTER_ALPHA   (0.25)     // frekvenciabecslő IIR szűrő együtthatója
#define PTS_MAX_EXTRAPOLATION_S (4)        // a legutóbbi PPS-től max. ennyi másodpercre extrapolálunk

// A PPS felfutó élének helye a PTP másodpercen belül.
// Digitális átfordulás (TSCTRLSSR=1) és PPSCTRL=0 (1 Hz) esetén a PPS a nanoszekundum-számláló
// 29. bitje: felfut x.536870912 s-nál (2^29 ns), lefut a másodperc-átforduláskor (x.000000000).
#define PTS_PPS_EDGE_PHASE_NS   (536870912LL)
#define PTS_MAX_PHASE_ERR_NS    (20000)    // a szoftveresen mért élidő ennyire térhet el a várt fázistól

// A PPS élhez tartozó referencia (ISR írja, feldolgozó kód olvassa - kritikus szakaszban)
typedef struct {
    uint32_t pps_tick;      // TIM2 számláló értéke a PPS élnél
    uint32_t pps_sec;       // az élhez tartozó PTP másodperc (az él PTP ideje: pps_sec + PTS_PPS_EDGE_PHASE_NS)
    double freq;            // TIM2 tick / PTP másodperc (szűrt)
    double last_freq;       // legutóbbi nyers periódus [tick]
    uint32_t valid_count;   // egymást követő érvényes PPS periódusok száma
    uint32_t pps_total;     // összes befogott PPS él
    uint32_t rejected;      // eldobott PPS élek / periódusok
    uint32_t isr_latency_ns;// PPS él és a PTP óra kiolvasása közti idő (diagnosztika)
    int32_t phase_err_ns;   // szoftveresen mért élidő - várt élidő (diagnosztika)
    bool has_prev;
} PtsState;

static TIM_HandleTypeDef *sHtim;
static volatile PtsState sState;
static double sNominalFreq;

// --------------------------------------------------------------------------

uint32_t PtpTimSync_GetNominalTimerClock(void) {
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t hclk = HAL_RCC_GetHCLKFreq();
    uint32_t ppre1 = RCC->D2CFGR & RCC_D2CFGR_D2PPRE1;

    if (ppre1 < RCC_D2CFGR_D2PPRE1_DIV2) { // 0xx: APB1 osztás nélkül
        return (RCC->CFGR & RCC_CFGR_TIMPRE) ? hclk : pclk1;
    }

    if (RCC->CFGR & RCC_CFGR_TIMPRE) {
        // TIMPRE=1: APB /2, /4 esetén HCLK, egyébként 4 x PCLK
        return (ppre1 <= RCC_D2CFGR_D2PPRE1_DIV4) ? hclk : 4 * pclk1;
    } else {
        // TIMPRE=0: APB osztó != 1 esetén 2 x PCLK
        return 2 * pclk1;
    }
}

static inline uint32_t irq_lock(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void irq_unlock(uint32_t primask) {
    __set_PRIMASK(primask);
}

// PTP óra konzisztens kiolvasása (másodperc-átfordulás kezelésével)
static void read_ptp_time(uint32_t *pSec, uint32_t *pNs) {
    uint32_t s1 = ETH->MACSTSR;
    uint32_t ns = ETH->MACSTNR & ETH_MACSTNR_TSSS;
    uint32_t s2 = ETH->MACSTSR;
    if (s1 != s2) {
        ns = ETH->MACSTNR & ETH_MACSTNR_TSSS;
    }
    *pSec = s2;
    *pNs = ns;
}

// --------------------------------------------------------------------------

void PtpTimSync_Init(TIM_HandleTypeDef *htim) {
    sHtim = htim;
    sNominalFreq = (double)PtpTimSync_GetNominalTimerClock();
    sState.freq = sNominalFreq;
    sState.has_prev = false;
    sState.valid_count = 0;

    TIM_IC_InitTypeDef ic = {0};

#if PTS_USE_INTERNAL_ITR
    // Belső PPS: a trigger bemenet (TS = ITRx) kiválasztása slave mód nélkül,
    // a capture csatorna forrása a TRC lesz. A trigger mindig felfutó élre érzékeny.
    TIM_SlaveConfigTypeDef slave = {0};
    slave.SlaveMode = TIM_SLAVEMODE_DISABLE;
    slave.InputTrigger = PTS_INTERNAL_ITR;
    if (HAL_TIM_SlaveConfigSynchro(sHtim, &slave) != HAL_OK) {
        MSG("[TSYNC] HIBA: a belso trigger bemenet nem allithato be!\n");
        return;
    }
    ic.ICSelection = TIM_ICSELECTION_TRC;
#else
    // Külső PPS bemeneti láb
    GPIO_InitTypeDef gpio = {0};
    PTS_GPIO_CLK_ENABLE();
    gpio.Pin = PTS_GPIO_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLDOWN; // átkötés nélkül se lebegjen
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = PTS_GPIO_AF;
    HAL_GPIO_Init(PTS_GPIO_PORT, &gpio);
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
#endif

    // Input capture: felfutó él, előosztó és szűrő nélkül (a szűrő fix késleltetést adna)
    ic.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    ic.ICPrescaler = TIM_ICPSC_DIV1;
    ic.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(sHtim, &ic, PTS_TIM_CHANNEL) != HAL_OK ||
        HAL_TIM_IC_Start_IT(sHtim, PTS_TIM_CHANNEL) != HAL_OK) {
        MSG("[TSYNC] HIBA: a PPS capture csatorna nem indult el!\n");
        return;
    }

    MSG("[TSYNC] PPS capture aktiv (%s), TIM2 nevleges orajel: %u Hz\n",
        PTS_USE_INTERNAL_ITR ? "belso ITR" : "kulso PA3", (uint32_t)sNominalFreq);
}

// ISR kontextusban fut (TIM2_IRQHandler -> HAL_TIM_IC_CaptureCallback)
void PtpTimSync_OnPpsCapture(uint32_t capture) {
    // TIM2 számláló és PTP óra kiolvasása közvetlenül egymás után
    uint32_t primask = irq_lock();
    uint32_t cnt = __HAL_TIM_GET_COUNTER(sHtim);
    uint32_t sec, ns;
    read_ptp_time(&sec, &ns);
    irq_unlock(primask);

    // Az él PTP idejének durva (szoftveres) becslése: kiolvasott idő - befogás óta eltelt idő
    int64_t elapsed_ns = llround((double)(uint32_t)(cnt - capture) * 1e9 / sNominalFreq);
    int64_t edge_est_ns = (int64_t)sec * NSEC_PER_SEC + ns - elapsed_ns;

    // Hozzárendelés a legközelebbi várt élidőhöz (egész másodperc + fix fázis): ez már pontos,
    // a szoftveres becslés csak a másodperc kiválasztására és ellenőrzésre szolgál
    int64_t rel = edge_est_ns - PTS_PPS_EDGE_PHASE_NS + NSEC_PER_SEC / 2;
    int64_t edge_sec = (rel >= 0) ? rel / NSEC_PER_SEC : -((-rel + NSEC_PER_SEC - 1) / NSEC_PER_SEC);
    int32_t phase_err = (int32_t)(edge_est_ns - (edge_sec * NSEC_PER_SEC + PTS_PPS_EDGE_PHASE_NS));

    sState.pps_total++;
    sState.isr_latency_ns = (uint32_t)elapsed_ns;
    sState.phase_err_ns = phase_err;

    if (edge_sec < 0 || phase_err > PTS_MAX_PHASE_ERR_NS || phase_err < -PTS_MAX_PHASE_ERR_NS) {
        // Az él nem a várt fázisban jött (pl. éppen ugrott a PTP óra): referencia eldobása
        sState.valid_count = 0;
        sState.rejected++;
        sState.has_prev = false;
        return;
    }

    if (sState.has_prev) {
        uint32_t dticks = capture - sState.pps_tick; // 32 bites túlcsordulás automatikusan kezelve
        int32_t dsec = (int32_t)(edge_sec - sState.pps_sec);
        double dev_ppm = ((double)dticks - sNominalFreq) / sNominalFreq * 1e6;

        if (dsec == 1 && fabs(dev_ppm) < PTS_MAX_DEVIATION_PPM) {
            sState.last_freq = (double)dticks;
            if (sState.valid_count == 0) {
                sState.freq = (double)dticks;
            } else {
                sState.freq += PTS_FREQ_FILTER_ALPHA * ((double)dticks - sState.freq);
            }
            sState.valid_count++;
        } else {
            // PTP óraugrás (pl. durva korrekció) vagy kimaradt PPS: újraszinkronizálás
            sState.valid_count = 0;
            sState.rejected++;
        }
    }

    sState.pps_tick = capture;
    sState.pps_sec = (uint32_t)edge_sec;
    sState.has_prev = true;
}

bool PtpTimSync_IsLocked(void) {
    return sState.valid_count >= PTS_LOCK_COUNT;
}

double PtpTimSync_GetTimerFreq(void) {
    uint32_t primask = irq_lock();
    bool locked = PtpTimSync_IsLocked();
    double freq = sState.freq;
    irq_unlock(primask);

    return locked ? freq : sNominalFreq;
}

bool PtpTimSync_TicksToPtp(uint32_t ticks, TimestampU *pTs) {
    uint32_t primask = irq_lock();
    bool locked = PtpTimSync_IsLocked();
    uint32_t ref_tick = sState.pps_tick;
    uint32_t ref_sec = sState.pps_sec;
    double freq = sState.freq;
    irq_unlock(primask);

    if (!locked) {
        return false;
    }

    int32_t dt = (int32_t)(ticks - ref_tick); // negatív is lehet (PPS előtti él)
    if (fabs((double)dt) > (double)PTS_MAX_EXTRAPOLATION_S * freq) {
        return false;
    }

    int64_t total_ns = (int64_t)ref_sec * NSEC_PER_SEC + PTS_PPS_EDGE_PHASE_NS + llround((double)dt * 1e9 / freq);
    if (total_ns < 0) {
        return false;
    }

    pTs->sec = (uint64_t)(total_ns / NSEC_PER_SEC);
    pTs->nanosec = (uint32_t)(total_ns % NSEC_PER_SEC);
    return true;
}

// --------------------------------------------------------------------------

static CMD_FUNCTION(cmd_tsync) {
    uint32_t primask = irq_lock();
    PtsState st = sState;
    irq_unlock(primask);

    double nominal = (double)PtpTimSync_GetNominalTimerClock();

    MSG("TIM2 <-> PTP szinkron: %s\n", PtpTimSync_IsLocked() ? "LOCKED" : "NINCS SZINKRON");
    MSG("  PPS elek: %u, egymast koveto ervenyes: %u, eldobott: %u\n", st.pps_total, st.valid_count, st.rejected);
    MSG("  Nevleges TIM2 orajel: %u Hz\n", (uint32_t)nominal);

    if (st.valid_count > 0) {
        MSG("  Mert TIM2 frekvencia (PTP idoben): %.3f Hz (utolso periodus: %.0f tick)\n", st.freq, st.last_freq);
        MSG("  Elteres a nevlegestol: %.4f ppm\n", (st.freq - nominal) / nominal * 1e6);
        MSG("  Utolso PPS: PTP %u.%09u s @ TIM2 %u, ISR kesleltetes: %u ns\n",
            st.pps_sec, (uint32_t)PTS_PPS_EDGE_PHASE_NS, st.pps_tick, st.isr_latency_ns);
    }
    if (st.pps_total > 0) {
        MSG("  PPS el fazishibaja (szoftveres becsles - vart): %d ns\n", st.phase_err_ns);
    }

    // Ellenőrzés: TIM2 CNT és PTP óra egyidejű kiolvasása, a CNT átszámítása PTP időre
    if (PtpTimSync_IsLocked()) {
        uint32_t sec, ns;
        primask = irq_lock();
        uint32_t cnt = __HAL_TIM_GET_COUNTER(sHtim);
        read_ptp_time(&sec, &ns);
        irq_unlock(primask);

        TimestampU ts;
        if (PtpTimSync_TicksToPtp(cnt, &ts)) {
            int64_t diff = ((int64_t)sec * NSEC_PER_SEC + ns) - ((int64_t)ts.sec * NSEC_PER_SEC + ts.nanosec);
            MSG("  Ellenorzes: TIM2 CNT -> PTP %u.%09u, PTP ora %u.%09u, kulonbseg %d ns\n",
                (uint32_t)ts.sec, ts.nanosec, sec, ns, (int32_t)diff);
        }
    }

    return 0;
}

void PtpTimSync_RegisterCli(void) {
    cli_register_command("tsync \t\t\tTIM2 <-> PTP clock synchronization status", 1, 0, cmd_tsync);
}
