#ifndef BARCODE_H
#define BARCODE_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

// ---- Shared element type for prebuf[] and burst[] ----
typedef struct {
    uint32_t t_us;   // timestamp (us)
    int      val;    // raw ADC value
} BcSample;

typedef struct {
    // --- monitor / thresholding state ---
    ThresholdMode th_mode;
    Polarity      polarity;
    int           white_mean;
    int           white_std;
    int           threshold;        // adaptive current threshold (BLACK enter)
    const char*   monitor_prev;     // "WHITE" | "BLACK" | "TRIGGERED"

    // --- pretrigger ring buffer ---
    #define PREBUF_MAX_SAMPLES  (PRETRIGGER_MS * BARCODE_MONITOR_HZ / 1000 + 8)
    BcSample     prebuf[PREBUF_MAX_SAMPLES];
    int          pre_head;
    int          pre_count;

    // --- burst capture buffer (dynamic) ---
    BcSample*    burst;
    int          burst_cap;
    int          burst_len;
    bool         burst_active;

    // --- decode output ---
    char         symbols[MAX_SYMBOLS_PER_SCAN];
    int          sym_len;
} BarcodeCtx;

// API
void barcode_init(BarcodeCtx* bc);
bool barcode_monitor_tick(BarcodeCtx* bc, int adc_pin, int* out_last_raw);
void barcode_start_burst(BarcodeCtx* bc);
bool barcode_burst_complete(BarcodeCtx* bc);
bool barcode_decode(BarcodeCtx* bc);

typedef enum { CMD_NONE=0, CMD_LEFT, CMD_RIGHT, CMD_STOP, CMD_UTURN } DecodedCmd;
DecodedCmd barcode_first_symbol_to_cmd(BarcodeCtx* bc);

// ISR access
extern BarcodeCtx* g_barcode_singleton;
void barcode_burst_isr(void);

#endif // BARCODE_H
