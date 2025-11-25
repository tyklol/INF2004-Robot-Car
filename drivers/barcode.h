#ifndef BARCODE_H
#define BARCODE_H

#include <stdbool.h>
#include <stdint.h>

// Hardware selection (ROBO-PICO / Pico W)
#define BARCODE_ADC_GPIO 27 // GP27
#define BARCODE_ADC_CH 1 // ADC1
#define BARCODE_BTN_BLACK 21 // GP21 (hold 1s to capture BLACK)

// Sampling
#define BARCODE_SAMPLE_HZ 600 // 300 Hz ~3.33 ms period

// Basic black/white state
typedef enum { BW_WHITE = 0, BW_BLACK = 1 } bw_t;

// Run & bit types
typedef struct {
    bw_t tone; // BLACK = bar, WHITE = space
    uint16_t len; // length in samples (3.33 ms each at 300 Hz)
} run_t;

// Lifecycle
void barcode_init(void);
void barcode_calibrate(void);
void barcode_clear_result(void);
// Classifier accessors
uint16_t barcode_read_adc_raw(void);
uint16_t barcode_last_adc(void);
bw_t barcode_last_bw(void);

// 300 Hz stepper (call from main loop)
bool     barcode_step_300hz(void);

// Thresholds info (for debugging/telemetry)
void barcode_get_thresholds(uint16_t *thr_black, uint16_t *thr_white,
                                uint16_t *mid, uint16_t *hys,
                                uint16_t *white_avg, uint16_t *black_avg,
                                uint16_t *white_std, uint16_t *black_std,
                                bool *inverted);

// N/W info (used by main.c)
void barcode_get_nw_info(bool *seeded, uint16_t *narrow_base, uint16_t *wide_threshold,
                            uint16_t *factor_num, uint16_t *factor_den);

// Run & bit buffers (non-blocking pop)
bool barcode_get_next_run(run_t *out);
bool barcode_get_next_bit(uint8_t *bit, bool *is_bar);

// outputs
const char* barcode_last_symbols(void); // e.g., "*Z*"
const char* barcode_last_payload(void); // e.g., "Z"
const char* barcode_last_command(void); // "LEFT" / "RIGHT" / "NONE"

#endif // BARCODE_H
