#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// ------------------------- PIN MAP (EDIT THESE) -------------------------
#define PIN_IR_LEFT_ADC_GPIO     26   // GP26 = ADC0
#define PIN_IR_LEFT_ADC_CH        0

#define PIN_IR_RIGHT_ADC_GPIO    27   // GP27 = ADC1
#define PIN_IR_RIGHT_ADC_CH       1

#define PIN_BARCODE_DO_GPIO      16   // digital DO of dedicated barcode sensor

#define PIN_MOTOR_LEFT_PWM       18   // PWM for left motor
#define PIN_MOTOR_RIGHT_PWM      19   // PWM for right motor

// If your motor driver needs DIR pins, add them here and set in set_motors().
#define HAVE_DIR_PINS            0
#define PIN_MOTOR_LEFT_DIR       20
#define PIN_MOTOR_RIGHT_DIR      21
// ------------------------------------------------------------------------

// ---------------------- LINE FOLLOW TUNABLES ----------------------------
#define BASE_SPEED_DUTY        0.40f   // nominal duty (0..1)
#define KP_TURN                0.30f   // proportional gain for steering
#define ADC_SMOOTH_SAMPLES        12
// ------------------------------------------------------------------------

// ---------------------- BARCODE CAPTURE TUNABLES ------------------------
#define MAX_ELEMS               256    // max bars/spaces captured in one scan
#define MIN_PULSE_US            120    // deglitch: ignore < this (µs)
#define GAP_TIMEOUT_MS          160    // stop capture if no edges for this long
#define EDGE_BURST_COUNT         10    // edges within WINDOW to enter barcode mode
#define EDGE_BURST_WINDOW_MS    140    // window to count burst edges
// ------------------------------------------------------------------------

// ==================== MOTOR PWM UTILITIES ===============================
static uint slice_left, slice_right;

static float clampf(float x, float lo, float hi){ return x<lo?lo:(x>hi?hi:x); }

static void pwm_init_pin(uint pin, float freq_hz) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    // Configure for ~freq_hz; default sys clk 125MHz
    float f_sys = 125000000.0f;
    uint32_t top = 9999; // 10k counts
    uint32_t div = (uint32_t)roundf((f_sys / (freq_hz * (top + 1))));
    if (div < 1) div = 1;
    pwm_set_clkdiv(slice, (float)div);
    pwm_set_wrap(slice, top);
    pwm_set_enabled(slice, true);
}

static void set_motor_pwm(uint pin, float duty) {
    duty = clampf(duty, 0.f, 1.f);
    uint slice = pwm_gpio_to_slice_num(pin);
    uint32_t top = pwm_get_wrap(slice);
    uint32_t level = (uint32_t)roundf(duty * (float)top);
    pwm_set_gpio_level(pin, level);
}

static void motors_init(void) {
    pwm_init_pin(PIN_MOTOR_LEFT_PWM, 1000.0f);
    pwm_init_pin(PIN_MOTOR_RIGHT_PWM,1000.0f);
#if HAVE_DIR_PINS
    gpio_init(PIN_MOTOR_LEFT_DIR);  gpio_set_dir(PIN_MOTOR_LEFT_DIR, GPIO_OUT);
    gpio_init(PIN_MOTOR_RIGHT_DIR); gpio_set_dir(PIN_MOTOR_RIGHT_DIR, GPIO_OUT);
#endif
}

static void set_motors(float left_duty, float right_duty) {
#if HAVE_DIR_PINS
    // Example mapping if you have direction pins:
    gpio_put(PIN_MOTOR_LEFT_DIR,  left_duty  >= 0);
    gpio_put(PIN_MOTOR_RIGHT_DIR, right_duty >= 0);
    set_motor_pwm(PIN_MOTOR_LEFT_PWM,  fabsf(left_duty));
    set_motor_pwm(PIN_MOTOR_RIGHT_PWM, fabsf(right_duty));
#else
    // If your driver is single-direction PWM only, use positive duty.
    set_motor_pwm(PIN_MOTOR_LEFT_PWM,  clampf(left_duty,  0.f,1.f));
    set_motor_pwm(PIN_MOTOR_RIGHT_PWM, clampf(right_duty, 0.f,1.f));
#endif
}
// =======================================================================

// ====================== LINE SENSOR UTILITIES ==========================
static uint16_t adc_read_avg(uint ch_gpio, uint ch, int samples) {
    (void)ch_gpio; // already prepared via adc_gpio_init once
    adc_select_input(ch);
    uint32_t acc=0;
    for (int i=0;i<samples;i++) acc += adc_read();
    return (uint16_t)(acc / samples);
}

static void line_follow_step(void) {
    // Read two sensors
    uint16_t L = adc_read_avg(PIN_IR_LEFT_ADC_GPIO,  PIN_IR_LEFT_ADC_CH,  ADC_SMOOTH_SAMPLES);
    uint16_t R = adc_read_avg(PIN_IR_RIGHT_ADC_GPIO, PIN_IR_RIGHT_ADC_CH, ADC_SMOOTH_SAMPLES);

    // Higher value ~ white (more reflect), lower ~ black
    // Compute error: positive => bias to right (turn left), negative => turn right
    float err = (float)L - (float)R;    // simple difference
    // Normalize by 4095 to keep small
    float steer = KP_TURN * (err / 4095.0f);

    float left  = clampf(BASE_SPEED_DUTY - steer, 0.f, 1.f);
    float right = clampf(BASE_SPEED_DUTY + steer, 0.f, 1.f);

    set_motors(left, right);
}
// =======================================================================

// ======================== BARCODE SUPPORT ==============================

typedef struct {
    bool is_black;  // true=black bar, false=white space
    float ms;       // duration in ms
} elem_t;

static volatile uint64_t bc_last_edge_us = 0;
static volatile bool     bc_last_level   = false; // level AFTER last edge
static volatile int      bc_count        = 0;
static volatile elem_t   bc_elems[MAX_ELEMS];
static volatile uint32_t bc_edge_counter = 0;     // for burst detection

// ISR: record element durations with colors
static void bc_irq(uint gpio, uint32_t events) {
    (void)gpio; (void)events;
    uint64_t now = time_us_64();
    bool level_now = gpio_get(PIN_BARCODE_DO_GPIO); // 1=white, 0=black (typical)

    if (bc_last_edge_us != 0) {
        uint32_t dt = (uint32_t)(now - bc_last_edge_us);
        if (dt >= MIN_PULSE_US && bc_count < MAX_ELEMS) {
            elem_t e; e.is_black = !level_now; e.ms = dt / 1000.0f;
            bc_elems[bc_count++] = e;
        }
    }
    bc_last_edge_us = now;
    bc_last_level   = level_now;
    bc_edge_counter++;
}

// estimate "narrow" duration
static float narrow_ms_estimate(const elem_t *e, int n) {
    float m=1e9f;
    for (int i=0;i<n;i++) if (e[i].ms<m) m=e[i].ms;
    return m;
}

static void build_nw_pattern(char *out9, const elem_t *e9, float narrow_ms) {
    float split = 2.0f * narrow_ms; // between 1x and 3x
    for (int i=0;i<9;i++) out9[i] = (e9[i].ms < split)? 'n':'W';
    out9[9]='\0';
}

typedef struct { const char *pat; char ch; } c39_entry;

// FULL Code 39 table (bars/spaces starting with BLACK; 9 elements of n/W).
// Source: standard Code 39 reference patterns (3:1 wide:narrow).
static const c39_entry C39[] = {
    // 0–9
    {"n n n W W n W n n", '0'},
    {"W n n n n W n W n", '1'},
    {"n W n n n W n W n", '2'},
    {"W W n n n n n W n", '3'},
    {"n n W n n W n W n", '4'},
    {"W n W n n n n W n", '5'},
    {"n W W n n n n W n", '6'},
    {"n n n W n W n W n", '7'},
    {"W n n W n n n W n", '8'},
    {"n W n W n n n W n", '9'},
    // A–Z
    {"W n n n n n W n W", 'A'},
    {"n W n n n n W n W", 'B'},
    {"W W n n n n n n W", 'C'},
    {"n n W n n n W n W", 'D'},
    {"W n W n n n n n W", 'E'},
    {"n W W n n n n n W", 'F'},
    {"n n n n W n W n W", 'G'},
    {"W n n n W n n n W", 'H'},
    {"n W n n W n n n W", 'I'},
    {"n n W n W n n n W", 'J'},
    {"W n n n n W n n W", 'K'},
    {"n W n n n W n n W", 'L'},
    {"W W n n n W n n n", 'M'},
    {"n n W n n W n n W", 'N'},
    {"W n W n n W n n n", 'O'},
    {"n W W n n W n n n", 'P'},
    {"n n n n W W n n W", 'Q'},
    {"W n n n W W n n n", 'R'},
    {"n W n n W W n n n", 'S'},
    {"n n W n W W n n n", 'T'},
    {"W n n n n n W W n", 'U'},
    {"n W n n n n W W n", 'V'},
    {"W W n n n n W n n", 'W'},
    {"n n W n n n W W n", 'X'},
    {"W n W n n n W n n", 'Y'},
    {"n W W n n n W n n", 'Z'},
    // symbols: - . SPACE $ / + %
    {"n n n W n n W W n", '-'},
    {"W n n W n n n n W", '.'},
    {"n W n W n W n n n", ' '},
    {"n n n n n W n W W", '$'},
    {"n n n W n W n n W", '/'},
    {"n n W n n n n W W", '+'},
    {"n W n n n n n W W", '%'},
    // start/stop *
    {"n W n n W n W n n", '*'}
};
static const int C39_LEN = sizeof(C39)/sizeof(C39[0]);

static int match_table(const char *spaced) {
    for (int i=0;i<C39_LEN;i++)
        if (strcmp(C39[i].pat, spaced)==0) return i;
    return -1;
}

static char decode_char(const elem_t *e9, float narrow_ms, char *spaced_out /*opt*/) {
    char pat9[10]; build_nw_pattern(pat9, e9, narrow_ms);
    char spaced[3*9]; int k=0;
    for (int i=0;i<9;i++){ spaced[k++]=pat9[i]; if(i<8) spaced[k++]=' '; }
    spaced[k]='\0';
    if (spaced_out) strcpy(spaced_out, spaced);
    int idx = match_table(spaced);
    return (idx>=0)? C39[idx].ch : '?';
}

static int skip_interchar(const elem_t *e, int i, int N, float narrow_ms) {
    if (i>=N) return -1;
    if (e[i].is_black) return -1;
    if (e[i].ms >= 2.0f*narrow_ms) return -1; // must be narrow white
    return i+1;
}

static int find_star(const elem_t *e, int N, float narrow_ms) {
    for (int i=0;i+8<N;i++) {
        if (!e[i].is_black) continue;
        bool ok=true;
        for (int j=1;j<9;j++){
            bool exp_black = (j%2==0);
            if (e[i+j].is_black != exp_black){ ok=false; break; }
        }
        if(!ok) continue;
        char ch = decode_char(&e[i], narrow_ms, NULL);
        if (ch=='*') return i;
    }
    return -1;
}

static void barcode_gpio_init(void){
    gpio_init(PIN_BARCODE_DO_GPIO);
    gpio_set_dir(PIN_BARCODE_DO_GPIO, GPIO_IN);
    gpio_disable_pulls(PIN_BARCODE_DO_GPIO);
    bc_last_level   = gpio_get(PIN_BARCODE_DO_GPIO);
    bc_last_edge_us = time_us_64();
}

static void barcode_irq_enable(bool en){
    gpio_set_irq_enabled_with_callback(
        PIN_BARCODE_DO_GPIO,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        en, &bc_irq);
}

// returns true if decoded payload is valid; prints progress
static bool barcode_decode(char *out, size_t out_sz) {
    int N = bc_count;
    if (N < 9) { printf("[BC] too few elements (%d)\n", N); return false; }

    float narrow_ms = narrow_ms_estimate((const elem_t*)bc_elems, N);
    printf("[BC] narrow ≈ %.2f ms, elems=%d\n", narrow_ms, N);

    int i = find_star((const elem_t*)bc_elems, N, narrow_ms);
    if (i < 0) { printf("[BC] start * not found\n"); return false; }

    char buf[96]; int bi=0;
    while (i+8 < N && bi < (int)sizeof(buf)-1) {
        char spaced[32];
        char ch = decode_char(&bc_elems[i], narrow_ms, spaced);
        printf("[BC] @%d: %s => %c\n", i, spaced, ch);
        buf[bi++] = ch;

        if (ch=='*') break;         // reached stop
        i += 9;
        int nxt = skip_interchar((const elem_t*)bc_elems, i, N, narrow_ms);
        if (nxt < 0) { printf("[BC] bad inter-char space\n"); break; }
        i = nxt;
    }
    buf[bi]='\0';

    if (bi>=2 && buf[0]=='*' && buf[bi-1]=='*') {
        // strip stars
        buf[bi-1]='\0';
        strncpy(out, buf+1, out_sz-1); out[out_sz-1]='\0';
        return true;
    } else {
        printf("[BC] invalid *...* envelope (%s)\n", buf);
        return false;
    }
}
// =======================================================================

// ====================== MODE MANAGEMENT =================================
typedef enum { MODE_LINE=0, MODE_BARCODE } mode_t;
static mode_t mode = MODE_LINE;

static void enter_barcode_mode(void){
    mode = MODE_BARCODE;
    // slow down for stable decoding
    set_motors(0.20f, 0.20f);
    // reset buffers
    bc_count = 0; bc_edge_counter = 0;
    bc_last_edge_us = time_us_64();
    barcode_irq_enable(true);
    printf("\n--- ENTER BARCODE MODE ---\n");
}

static void exit_barcode_mode(void){
    barcode_irq_enable(false);
    printf("--- EXIT BARCODE MODE ---\n\n");
    mode = MODE_LINE;
}

// detect burst of edges to auto-enter barcode mode
static bool barcode_burst_seen(void){
    static absolute_time_t window_end = {0};
    static uint32_t last_edge_count = 0;

    if (bc_edge_counter != last_edge_count) {
        last_edge_count = bc_edge_counter;
        window_end = delayed_by_ms(get_absolute_time(), EDGE_BURST_WINDOW_MS);
    }
    // If we gathered enough edges within the active window, trigger
    if (last_edge_count >= EDGE_BURST_COUNT &&
        absolute_time_diff_us(get_absolute_time(), window_end) > 0) {
        // window active and enough edges
        bc_edge_counter = 0;
        return true;
    }
    return false;
}
// =======================================================================

int main() {
    stdio_init_all();
    sleep_ms(300);
    printf("\nRobocar: line follow + Code39 barcode (auto)\n");

    // motors
    motors_init();
    set_motors(0,0);

    // analog ADC init (left/right line sensors)
    adc_init();
    adc_gpio_init(PIN_IR_LEFT_ADC_GPIO);
    adc_gpio_init(PIN_IR_RIGHT_ADC_GPIO);

    // barcode DO init
    barcode_gpio_init();

    absolute_time_t bc_timeout = make_timeout_time_ms(0);

    while (true) {
        if (mode == MODE_LINE) {
            // normal line following
            line_follow_step();

            // watch for barcode edge burst on the DO pin
            if (barcode_burst_seen()) {
                enter_barcode_mode();
                bc_timeout = delayed_by_ms(get_absolute_time(), GAP_TIMEOUT_MS);
            }
        } else { // MODE_BARCODE
            // keep creeping slowly while scanning
            set_motors(0.18f, 0.18f);

            // extend timeout every time a new element is recorded
            static int prev_count = -1;
            if (bc_count != prev_count) {
                prev_count = bc_count;
                bc_timeout = delayed_by_ms(get_absolute_time(), GAP_TIMEOUT_MS);
            }

            // done when we see a gap
            if (absolute_time_diff_us(get_absolute_time(), bc_timeout) < 0) {
                // decode
                char payload[64];
                bool ok = barcode_decode(payload, sizeof payload);
                if (ok) {
                    printf("[BC] DECODED: \"%s\"\n", payload);
                    // OPTIONAL: map commands, e.g., "L","R","U","STOP" etc.
                    // For now, just beep/print and resume line mode.
                } else {
                    printf("[BC] decode failed\n");
                }
                exit_barcode_mode();
            }
        }

        sleep_ms(10); // loop pace
    }
}
