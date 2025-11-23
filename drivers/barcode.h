#ifndef BARCODE_H
#define BARCODE_H

#include <stdbool.h>
#include <stdint.h>

// Hardware selection
#define BARCODE_ADC_GPIO 27 // GP27
#define BARCODE_ADC_CH 1 // ADC1
#define BARCODE_BTN_BLACK 21 // GP21

// Sampling
#define BARCODE_SAMPLE_HZ 600

// Basic black/white state
typedef enum { BW_WHITE = 0, BW_BLACK = 1 } bw_t;

// Run & bit types
typedef struct {
    bw_t tone;
    uint16_t len; 
} run_t;

// Lifecycle
void barcode_init(void);
void barcode_calibrate(void); // Learns polarity (High=Black or Low=Black)

// Accessors
bool barcode_step_300hz(void); // Call this in your loop

// Debugging (Modified to return simplified info)
void barcode_get_thresholds(uint16_t *thr_black, uint16_t *thr_white,
                            uint16_t *mid, uint16_t *hys,
                            uint16_t *white_avg, uint16_t *black_avg,
                            uint16_t *white_std, uint16_t *black_std,
                            bool *inverted);

void barcode_get_nw_info(bool *seeded, uint16_t *narrow_base, uint16_t *wide_threshold,
                            uint16_t *factor_num, uint16_t *factor_den);

bool barcode_get_next_run(run_t *out);
bool barcode_get_next_bit(uint8_t *bit, bool *is_bar);

const char* barcode_last_symbols(void);
const char* barcode_last_payload(void);
const char* barcode_last_command(void);

#endif // BARCODE_H