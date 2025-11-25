#include "barcode.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

// INTERNAL CONFIG
#define RUN_Q_CAP 512
#define BIT_Q_CAP 1024
#define SEED_CAP 32
#define SEED_MIN 12
#define NW_FACTOR_NUM 18
#define NW_FACTOR_DEN 10
#define TRANSITION_HOLD_SAMPLES 3

// Adaptive N/W tracking
#define RECENT_CAP 20
#define ADAPT_ALPHA_NUM 2 // alpha = 0.2
#define ADAPT_ALPHA_DEN 10

// min_len ≈ 0.25 * narrow_base
#define MIN_RUN_FACTOR_NUM 25
#define MIN_RUN_FACTOR_DEN 100

#define WHITE_TIMEOUT_SEC 0.5
#define WHITE_TIMEOUT_SAMPLES (WHITE_TIMEOUT_SEC * BARCODE_SAMPLE_HZ)

#define PRESEED_CAP 64

// CODE 39 TABLE (bars/spaces, 9 elements, wide=1, narrow=0)
typedef struct
{
    const char *bits;
    char ch;
} c39_t;

static const c39_t C39[] = {
    {"000110100", '0'},
    {"100100001", '1'},
    {"001100001", '2'},
    {"101100000", '3'},
    {"000110001", '4'},
    {"100110000", '5'},
    {"001110000", '6'},
    {"000100101", '7'},
    {"100100100", '8'},
    {"001100100", '9'},
    {"100001001", 'A'},
    {"001001001", 'B'},
    {"101001000", 'C'},
    {"000011001", 'D'},
    {"100011000", 'E'},
    {"001011000", 'F'},
    {"000001101", 'G'},
    {"100001100", 'H'},
    {"001001100", 'I'},
    {"000011100", 'J'},
    {"100000011", 'K'},
    {"001000011", 'L'},
    {"101000010", 'M'},
    {"000010011", 'N'},
    {"100010010", 'O'},
    {"001010010", 'P'},
    {"000000111", 'Q'},
    {"100000110", 'R'},
    {"001000110", 'S'},
    {"000010110", 'T'},
    {"110000001", 'U'},
    {"011000001", 'V'},
    {"111000000", 'W'},
    {"010010001", 'X'},
    {"110010000", 'Y'},
    {"011010000", 'Z'},
    {"010000101", '-'},
    {"110000100", '.'},
    {"011000100", ' '},
    {"010101000", '$'},
    {"010100010", '/'},
    {"010001010", '+'},
    {"000101010", "%"[0]},
    {"010010100", '*'}, // start/stop
};
static const size_t C39_N = sizeof(C39) / sizeof(C39[0]);

// INTERNAL STATE
static struct
{
    uint16_t white_avg, black_avg;
    uint16_t white_std, black_std;
    bool inverted;

    uint16_t mid, hys, thr_black, thr_white;

    uint16_t last_adc;
    bw_t last_bw;
    bool has_bw;
    absolute_time_t next_tick;

    bw_t cur_tone;
    uint16_t cur_len;
    bool have_run;

    bw_t pend_tone;
    uint16_t pend_len;

    run_t run_q[RUN_Q_CAP];
    uint16_t run_q_head, run_q_tail;

    uint8_t bit_q[BIT_Q_CAP], bit_is_bar[BIT_Q_CAP];
    uint16_t bit_q_head, bit_q_tail;

    uint16_t seed_len[SEED_CAP];
    uint16_t seed_cnt;
    bool seeded;
    uint16_t narrow_base;
    uint16_t wide_threshold;

    uint16_t recent_runs[RECENT_CAP];
    uint8_t recent_head;
    uint8_t recent_count;
    uint16_t narrow_base_adapt;
    bool adaptive_started;

    run_t preseed_runs[PRESEED_CAP];
    uint16_t preseed_cnt;
    bool preseed_started;

    uint16_t white_consec;
    bool dumped_this_scan;

    char bit_line[600];
    uint16_t bit_len;

    char last_syms[64];
    char last_payload[60];
    char last_cmd[8];
} S;


static inline uint16_t u16_clip_i32(int32_t v)
{
    if (v < 0) return 0;
    if (v > 4095) return 4095;
    return (uint16_t)v;
}
uint16_t barcode_read_adc_raw(void) { return adc_read(); }
uint16_t barcode_last_adc(void) { return S.last_adc; }
bw_t barcode_last_bw(void) { return S.last_bw; }

// INIT & CALIBRATION
static void reset_queues_and_run(void)
{
    S.run_q_head = S.run_q_tail = 0;
    S.bit_q_head = S.bit_q_tail = 0;
    S.have_run = false;
    S.cur_len = 0;
    S.seed_cnt = 0;
    S.seeded = false;
    S.narrow_base = 0;
    S.wide_threshold = 0;
    S.recent_head = 0;
    S.recent_count = 0;
    S.narrow_base_adapt = 0;
    S.adaptive_started = false;
    S.pend_len = 0;
    S.preseed_cnt = 0;
    S.preseed_started = false;
    S.white_consec = 0;
    S.dumped_this_scan = false;
    S.bit_len = 0;
    S.bit_line[0] = '\0';
    S.last_syms[0] = '\0';
    S.last_payload[0] = '\0';
    S.last_cmd[0] = '\0';
}
void barcode_init(void)
{
    adc_init();
    adc_gpio_init(BARCODE_ADC_GPIO);
    adc_select_input(BARCODE_ADC_CH);
    gpio_init(BARCODE_BTN_BLACK);
    gpio_pull_up(BARCODE_BTN_BLACK);
    gpio_set_dir(BARCODE_BTN_BLACK, GPIO_IN);

    S.white_avg = 3900;
    S.black_avg = 3600;
    S.white_std = 20;
    S.black_std = 20;
    S.inverted = (S.black_avg > S.white_avg);

    uint16_t std_max = (S.white_std > S.black_std) ? S.white_std : S.black_std;
    uint16_t w_avg = S.inverted ? (uint16_t)(4095 - S.white_avg) : S.white_avg;
    uint16_t b_avg = S.inverted ? (uint16_t)(4095 - S.black_avg) : S.black_avg;
    S.mid = (uint16_t)((w_avg + b_avg) / 2);
    S.hys = std_max;
    S.thr_black = u16_clip_i32((int32_t)S.mid - (int32_t)S.hys);
    S.thr_white = u16_clip_i32((int32_t)S.mid + (int32_t)S.hys);

    S.has_bw = false;
    S.last_adc = 0;
    reset_queues_and_run();
    S.next_tick = make_timeout_time_ms(0);
}

static void capture_stats_ms(uint32_t dur_ms, uint16_t *avg, uint16_t *std)
{
    const uint32_t N = dur_ms * BARCODE_SAMPLE_HZ / 1000u;
    if (!N) { *avg = 0; *std = 0; return; }

    uint32_t sum = 0;
    for (uint32_t i = 0; i < N; ++i)
    {
        sum += barcode_read_adc_raw();
        sleep_us(1000000u / BARCODE_SAMPLE_HZ);
    }
    uint16_t mean = (uint16_t)(sum / N);

    uint64_t acc2 = 0;
    for (uint32_t i = 0; i < N; ++i)
    {
        int32_t d = (int32_t)barcode_read_adc_raw() - (int32_t)mean;
        acc2 += (uint64_t)(d * d);
        sleep_us(1000000u / BARCODE_SAMPLE_HZ);
    }
    uint32_t var = (uint32_t)(acc2 / N);
    uint16_t sdev = (uint16_t)(sqrt((double)var) + 0.5);
    *avg = mean;
    *std = sdev;
}
static void wait_for_button_press_and_release_ms(uint32_t hold_ms)
{
    while (gpio_get(BARCODE_BTN_BLACK)) { tight_loop_contents(); }
    absolute_time_t t_end = make_timeout_time_ms(hold_ms);
    while (!time_reached(t_end))
    {
        if (gpio_get(BARCODE_BTN_BLACK))
            return wait_for_button_press_and_release_ms(hold_ms);
        tight_loop_contents();
    }
    while (!gpio_get(BARCODE_BTN_BLACK)) { tight_loop_contents(); }
}
void barcode_calibrate(void)
{
    printf("Now place sensor over WHITE background.\n");
    printf("Press GP%d and hold for 1s, then release to capture WHITE.\n", BARCODE_BTN_BLACK);
    printf("Waiting for press...\n");
    wait_for_button_press_and_release_ms(1000);
    printf("Capturing WHITE for 1000 ms...\n");
    capture_stats_ms(1000, &S.white_avg, &S.white_std);
    printf("\nNow place sensor over BLACK line.\n");
    printf("Press GP%d and hold for 1s, then release.\n", BARCODE_BTN_BLACK);
    printf("Waiting for press...\n");
    wait_for_button_press_and_release_ms(1000);
    capture_stats_ms(500, &S.black_avg, &S.black_std);

    S.inverted = (S.black_avg > S.white_avg);
    uint16_t w_avg = S.inverted ? (uint16_t)(4095 - S.white_avg) : S.white_avg;
    uint16_t b_avg = S.inverted ? (uint16_t)(4095 - S.black_avg) : S.black_avg;
    uint16_t std_max = (S.white_std > S.black_std) ? S.white_std : S.black_std;
    S.mid = (uint16_t)((w_avg + b_avg) / 2);
    S.hys = std_max;
    S.thr_black = u16_clip_i32((int32_t)S.mid - (int32_t)S.hys);
    S.thr_white = u16_clip_i32((int32_t)S.mid + (int32_t)S.hys);

    printf("\nCalibration done.\n");
    printf("WHITE avg=%u std=%u\n", S.white_avg, S.white_std);
    printf("BLACK avg=%u std=%u\n", S.black_avg, S.black_std);
    printf("Polarity: %s\n", S.inverted ? "INVERTED (using 4095 - ADC)" : "NORMAL");
    printf("Mid=%u  Hys=%u  -> thr_black<=%u  thr_white>=%u\n", S.mid, S.hys, S.thr_black, S.thr_white);

    if ((S.mid - S.hys) >= (S.mid + S.hys))
        printf("WARN: Weak separation (BLK+HYS >= WHT-HYS). Consider re-calibrating or lowering sensor height.\n");

    S.has_bw = false;
    reset_queues_and_run();
}

// HYSTERESIS + QUEUES
static bw_t classify_with_hysteresis(uint16_t v)
{
    if (!S.has_bw)
    {
        bw_t s0 = (v <= S.mid) ? BW_BLACK : BW_WHITE;
        S.has_bw = true;
        return s0;
    }
    if (S.last_bw == BW_WHITE)
        return (v <= S.thr_black) ? BW_BLACK : BW_WHITE;
    else
        return (v >= S.thr_white) ? BW_WHITE : BW_BLACK;
}
static inline bool run_q_push(run_t r)
{
    uint16_t nhead = (S.run_q_head + 1) % RUN_Q_CAP;
    if (nhead == S.run_q_tail) return false;
    S.run_q[S.run_q_head] = r;
    S.run_q_head = nhead;
    return true;
}
bool barcode_get_next_run(run_t *out)
{
    if (S.run_q_head == S.run_q_tail) return false;
    if (out) *out = S.run_q[S.run_q_tail];
    S.run_q_tail = (S.run_q_tail + 1) % RUN_Q_CAP;
    return true;
}
static inline bool bit_q_push(uint8_t bit, bool is_bar)
{
    uint16_t nhead = (S.bit_q_head + 1) % BIT_Q_CAP;
    if (nhead == S.bit_q_tail) return false;
    S.bit_q[S.bit_q_head] = bit;
    S.bit_is_bar[S.bit_q_head] = is_bar ? 1 : 0;
    S.bit_q_head = nhead;
    return true;
}
bool barcode_get_next_bit(uint8_t *bit, bool *is_bar)
{
    if (S.bit_q_head == S.bit_q_tail) return false;
    if (bit) *bit = S.bit_q[S.bit_q_tail];
    if (is_bar) *is_bar = S.bit_is_bar[S.bit_q_tail] ? true : false;
    S.bit_q_tail = (S.bit_q_tail + 1) % BIT_Q_CAP;
    return true;
}

// SEEDING HELPERS
static uint16_t median_u16(uint16_t *arr, uint16_t n)
{
    for (uint16_t i = 1; i < n; ++i)
    {
        uint16_t key = arr[i], j = i;
        while (j && arr[j - 1] > key) { arr[j] = arr[j - 1]; --j; }
        arr[j] = key;
    }
    return arr[n / 2];
}
static void nw_add_seed(uint16_t len)
{
    if (S.seed_cnt == 0 && S.cur_tone == BW_WHITE && len > 1000) return;
    if (S.seed_cnt < SEED_CAP) S.seed_len[S.seed_cnt++] = len;

    if (!S.seeded && S.seed_cnt >= SEED_MIN)
    {
        uint16_t tmp[SEED_CAP];
        memcpy(tmp, S.seed_len, sizeof(uint16_t) * S.seed_cnt);
        uint16_t med = median_u16(tmp, S.seed_cnt);
        S.narrow_base = med;
        uint32_t thr = ((uint32_t)med * NW_FACTOR_NUM + (NW_FACTOR_DEN - 1)) / NW_FACTOR_DEN;
        S.wide_threshold = (uint16_t)thr;
        S.seeded = true;
        printf("[NW] Seeded with %u runs: narrow_base=%u samples, wide>=~%u (x%u/%u)\n",
               S.seed_cnt, S.narrow_base, S.wide_threshold, NW_FACTOR_NUM, NW_FACTOR_DEN);
    }
}
static uint8_t nw_classify(uint16_t len)
{
    if (!S.seeded) return 0;
    uint32_t lhs = (uint32_t)len * NW_FACTOR_DEN;
    uint32_t rhs = (uint32_t)S.narrow_base * NW_FACTOR_NUM;
    return (lhs >= rhs) ? 1 : 0;
}

// ADAPTIVE NARROW/WIDE TRACKING
static inline void recent_push(uint16_t len, bw_t tone)
{
    if (tone == BW_WHITE && len > 1000) return;
    S.recent_runs[S.recent_head] = len;
    S.recent_head = (uint8_t)((S.recent_head + 1) % RECENT_CAP);
    if (S.recent_count < RECENT_CAP) S.recent_count++;
}

static uint16_t recent_robust_narrow(void)
{
    if (S.recent_count < 4 || !S.adaptive_started) return S.narrow_base;

    uint16_t tmp[RECENT_CAP];
    for (uint8_t i = 0; i < S.recent_count; ++i) tmp[i] = S.recent_runs[i];

    for (uint8_t i = 1; i < S.recent_count; ++i)
    {
        uint16_t key = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > key) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = key;
    }

    uint8_t half = (uint8_t)(S.recent_count / 2);
    uint8_t mid  = (uint8_t)(half / 2);
    return tmp[mid];
}

static void adaptive_update_after_run(run_t r)
{
    if (!S.adaptive_started) return;

    recent_push(r.len, r.tone);

    uint16_t est = recent_robust_narrow();
    if (est == 0) return;

    uint32_t num = (uint32_t)(ADAPT_ALPHA_DEN - ADAPT_ALPHA_NUM) * S.narrow_base_adapt
                 + (uint32_t)ADAPT_ALPHA_NUM * est
                 + (ADAPT_ALPHA_DEN / 2);
    S.narrow_base_adapt = (uint16_t)(num / ADAPT_ALPHA_DEN);

    S.narrow_base = S.narrow_base_adapt;
    uint32_t thr = ((uint32_t)S.narrow_base * NW_FACTOR_NUM + (NW_FACTOR_DEN - 1)) / NW_FACTOR_DEN;
    S.wide_threshold = (uint16_t)thr;
}

// Min-run filter
static inline bool run_is_too_short(uint16_t len)
{
    if (!S.seeded || S.narrow_base == 0) return false;
    uint32_t min_len = ((uint32_t)S.narrow_base * MIN_RUN_FACTOR_NUM + (MIN_RUN_FACTOR_DEN - 1)) / MIN_RUN_FACTOR_DEN;
    if (min_len < TRANSITION_HOLD_SAMPLES + 1) min_len = TRANSITION_HOLD_SAMPLES + 1;
    return (len < (uint16_t)min_len);
}

static inline void append_bit_to_line(uint8_t bit)
{
    if (S.bit_len + 1 < sizeof(S.bit_line))
    {
        S.bit_line[S.bit_len++] = bit ? '1' : '0';
        S.bit_line[S.bit_len] = '\0';
    }
}

static void backfill_preseed_bits(void)
{
    if (!S.seeded || S.preseed_cnt == 0) return;
    for (uint16_t i = 0; i < S.preseed_cnt; ++i)
    {
        run_t r = S.preseed_runs[i];
        uint8_t bit = (uint8_t)(((uint32_t)r.len * NW_FACTOR_DEN) >= ((uint32_t)S.narrow_base * NW_FACTOR_NUM));
        (void)bit_q_push(bit, (r.tone == BW_BLACK));
        append_bit_to_line(bit);
    }
    S.preseed_cnt = 0;
}

static void start_new_scan(void)
{
    S.preseed_started = false;
    S.preseed_cnt = 0;
    S.seed_cnt = 0;
    S.seeded = false;
    S.narrow_base = 0;
    S.wide_threshold = 0;
    S.recent_head = 0;
    S.recent_count = 0;
    S.narrow_base_adapt = 0;
    S.adaptive_started = false;
    S.run_q_head = S.run_q_tail = 0;
    S.bit_q_head = S.bit_q_tail = 0;
    S.bit_len = 0;
    S.bit_line[0] = '\0';
    S.white_consec = 0;
    S.dumped_this_scan = false;
    S.have_run = false;
    S.cur_len = 0;
    S.pend_len = 0;
    S.last_syms[0] = '\0';
    S.last_payload[0] = '\0';
    S.last_cmd[0] = '\0';
}

// Decoding helpers
static int c39_lookup(const char *bits9)
{
    for (size_t i = 0; i < C39_N; ++i)
        if (strcmp(C39[i].bits, bits9) == 0) return (int)i;
    return -1;
}
static void reverse_str(char *s)
{
    size_t n = strlen(s);
    for (size_t i = 0; i < n / 2; ++i)
    {
        char t = s[i];
        s[i] = s[n - 1 - i];
        s[n - 1 - i] = t;
    }
}
static int extract_symbol_9(const char *bits, const uint8_t *is_bar, size_t n, size_t *idx, char bits9[10])
{
    if (*idx + 9 > n) return 0;
    if (!is_bar[*idx]) return 0;
    for (int k = 0; k < 9; ++k)
    {
        if (((k % 2) == 0) != (is_bar[*idx + k] ? 1 : 0)) return 0;
        bits9[k] = bits[*idx + k];
    }
    bits9[9] = '\0';
    *idx += 9;
    if (*idx < n && !is_bar[*idx] && bits[*idx] == '0') (*idx)++;
    return 1;
}

static void classify_command_from_first_letter(char first, char *out)
{
    const char *R = "ACEGIKMOQSUWY";
    const char *L = "BDFHJLNPRTVXZ";
    if (strchr(R, first)) { strcpy(out, "RIGHT"); return; }
    if (strchr(L, first)) { strcpy(out, "LEFT"); return; }
    strcpy(out, "NONE");
}

static void decode_and_print(void)
{
    uint16_t count = (S.bit_q_head >= S.bit_q_tail)
                         ? (S.bit_q_head - S.bit_q_tail)
                         : (BIT_Q_CAP - S.bit_q_tail + S.bit_q_head);
    if (count == 0) { S.last_syms[0]=S.last_payload[0]=S.last_cmd[0]='\0'; return; }

    static char bits_buf[BIT_Q_CAP + 1];
    static uint8_t bar_buf[BIT_Q_CAP];
    for (uint16_t k = 0; k < count; ++k)
    {
        uint16_t pos = (S.bit_q_tail + k) % BIT_Q_CAP;
        bits_buf[k] = S.bit_q[pos] ? '1' : '0';
        bar_buf[k]  = S.bit_is_bar[pos];
    }
    bits_buf[count] = '\0';

    char syms[64]; syms[0]='\0';
    char payload[60]; payload[0]='\0';

    for (int pass = 0; pass < 2; ++pass)
    {
        size_t i = 0;
        char curr_syms[64]; curr_syms[0]='\0';
        while (i < count)
        {
            char b9[10];
            size_t saved = i;
            if (!extract_symbol_9(bits_buf, bar_buf, count, &i, b9)) { ++i; continue; }
            int idx = c39_lookup(b9);
            if (idx < 0) { i = saved + 1; continue; }
            size_t Ls = strlen(curr_syms);
            if (Ls + 1 < sizeof(curr_syms))
            {
                curr_syms[Ls] = C39[idx].ch;
                curr_syms[Ls + 1] = '\0';
            }
        }
        size_t Ls = strlen(curr_syms);
        if (Ls >= 2 && curr_syms[0]=='*' && curr_syms[Ls-1]=='*')
        {
            strncpy(syms, curr_syms, sizeof(syms)-1); syms[sizeof(syms)-1]='\0';
            size_t pl = Ls - 2; if (pl >= sizeof(payload)) pl = sizeof(payload)-1;
            memcpy(payload, curr_syms + 1, pl); payload[pl]='\0';
            break;
        }
        if (pass == 0)
        {
            reverse_str(bits_buf);
            for (uint16_t a=0,b=count-1; a<b; ++a,--b)
            { uint8_t t=bar_buf[a]; bar_buf[a]=bar_buf[b]; bar_buf[b]=t; }
        }
    }

    strncpy(S.last_syms, syms, sizeof(S.last_syms)-1); S.last_syms[sizeof(S.last_syms)-1]='\0';
    strncpy(S.last_payload, payload, sizeof(S.last_payload)-1); S.last_payload[sizeof(S.last_payload)-1]='\0';

    if (S.last_payload[0]) classify_command_from_first_letter(S.last_payload[0], S.last_cmd);
    else S.last_cmd[0]='\0';

    if (S.last_syms[0]) printf("SYMS: %s\n", S.last_syms);
    if (S.last_payload[0]) printf("Decoded: %s\n", S.last_payload);
    if (S.last_cmd[0]) printf("Command: %s\n", S.last_cmd);
    else printf("Command: NONE\n");
}

// Not 300hz, Despite the name, this function steps at BARCODE_SAMPLE_HZ
bool barcode_step_300hz(void)
{
    if (!time_reached(S.next_tick)) sleep_until(S.next_tick);
    S.next_tick = delayed_by_us(get_absolute_time(), 1000000u / BARCODE_SAMPLE_HZ);

    uint16_t raw = barcode_read_adc_raw();
    uint16_t v = S.inverted ? (uint16_t)(4095 - raw) : raw;
    bw_t bw = classify_with_hysteresis(v);
    S.last_adc = v;
    S.last_bw = bw;

    // WHITE timeout, end of scan
    if (bw == BW_WHITE)
    {
        if (S.white_consec < 0xFFFF) S.white_consec++;
        if (!S.dumped_this_scan && S.white_consec >= WHITE_TIMEOUT_SAMPLES)
        {
            if (S.preseed_started && S.bit_len > 0) printf("BIT: %s\n", S.bit_line);
            decode_and_print();
            S.dumped_this_scan = true;
            start_new_scan();
            return true;
        }
    }
    else
    {
        S.white_consec = 0;
        S.dumped_this_scan = false;
    }

    // Debounced run accumulation
    if (!S.have_run)
    {
        S.cur_tone = bw;
        S.cur_len = 1;
        S.pend_len = 0;
        S.have_run = true;
    }
    else
    {
        if (bw == S.cur_tone)
        {
            S.pend_len = 0;
            if (S.cur_len < 0xFFFF) S.cur_len++;
        }
        else
        {
            if (S.pend_len == 0)
            {
                S.pend_tone = bw;
                S.pend_len = 1;
            }
            else
            {
                if (bw == S.pend_tone)
                {
                    if (S.pend_len < 0xFFFF) S.pend_len++;
                }
                else
                {
                    S.pend_len = 0;
                }
            }

            if (S.pend_len >= TRANSITION_HOLD_SAMPLES)
            {
                run_t r = {.tone = S.cur_tone, .len = S.cur_len};
                bool discard_short = run_is_too_short(r.len);

                if (discard_short)
                {
                    // treat as noise, undo the transition
                    uint32_t merged = (uint32_t)S.cur_len + S.pend_len; // add held samples back
                    S.cur_len = (merged > 0xFFFF) ? 0xFFFF : (uint16_t)merged;
                    S.pend_len = 0;
                }
                else
                {
                    (void)run_q_push(r);

                    // preseed buffering
                    if (!S.seeded)
                    {
                        if (!S.preseed_started)
                        {
                            if (r.tone == BW_BLACK)
                            {
                                S.preseed_started = true;
                                S.bit_len = 0;
                                S.bit_line[0] = '\0';
                                if (S.preseed_cnt < PRESEED_CAP)
                                    S.preseed_runs[S.preseed_cnt++] = r;
                            }
                        }
                        else
                        {
                            if (S.preseed_cnt < PRESEED_CAP)
                                S.preseed_runs[S.preseed_cnt++] = r;
                        }
                    }

                    bool was_seeded = S.seeded;
                    nw_add_seed(r.len);
                    bool became_seeded = (!was_seeded && S.seeded);

                    if (became_seeded)
                    {
                        S.narrow_base_adapt = S.narrow_base;
                        S.adaptive_started = true;
                        S.recent_head = 0;
                        S.recent_count = 0;
                        backfill_preseed_bits();
                    }
                    else if (S.seeded)
                    {
                        adaptive_update_after_run(r);
                        uint8_t bit = nw_classify(r.len);
                        (void)bit_q_push(bit, (r.tone == BW_BLACK));
                        append_bit_to_line(bit);
                    }

                    // switch to new tone normally
                    S.cur_tone = S.pend_tone;
                    S.cur_len = S.pend_len;
                    S.pend_len = 0;
                }
            }
        }
    }
    return true;
}

// INTROSPECTION + Accessors
void barcode_get_thresholds(uint16_t *thr_black, uint16_t *thr_white,
                            uint16_t *mid, uint16_t *hys,
                            uint16_t *white_avg, uint16_t *black_avg,
                            uint16_t *white_std, uint16_t *black_std,
                            bool *inverted)
{
    if (thr_black) *thr_black = S.thr_black;
    if (thr_white) *thr_white = S.thr_white;
    if (mid) *mid = S.mid;
    if (hys) *hys = S.hys;
    if (white_avg) *white_avg = S.white_avg;
    if (black_avg) *black_avg = S.black_avg;
    if (white_std) *white_std = S.white_std;
    if (black_std) *black_std = S.black_std;
    if (inverted) *inverted = S.inverted;
}
void barcode_get_nw_info(bool *seeded, uint16_t *narrow_base, uint16_t *wide_threshold,
                         uint16_t *factor_num, uint16_t *factor_den)
{
    if (seeded) *seeded = S.seeded;
    if (narrow_base) *narrow_base = S.narrow_base;
    if (wide_threshold) *wide_threshold = S.wide_threshold;
    if (factor_num) *factor_num = NW_FACTOR_NUM;
    if (factor_den) *factor_den = NW_FACTOR_DEN;
}

const char *barcode_last_symbols(void) { return S.last_syms; }
const char *barcode_last_payload(void) { return S.last_payload; }
const char *barcode_last_command(void) { return S.last_cmd; }