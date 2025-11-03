#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "barcode.h"

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/timer.h"

// ---------- helpers ----------
static inline uint32_t now_ms(void){ return to_ms_since_boot(get_absolute_time()); }
static inline uint32_t now_us(void){ return (uint32_t)time_us_64(); }

static inline int adc_read_raw_pin(int pin){
    int ch = (pin==26)?0:(pin==27)?1:(pin==28)?2:-1;
    if(ch<0) return 0;
    adc_select_input(ch);
    return adc_read();
}

// Singleton for ISR
BarcodeCtx* g_barcode_singleton = NULL;

// ---------- Code 39 map ----------
typedef struct { const char* bits; char ch; } c39_t;
static const c39_t C39[] = {
    {"000110100",'0'}, {"100010001",'1'}, {"001010001",'2'}, {"101010000",'3'},
    {"000110001",'4'}, {"100110000",'5'}, {"001110000",'6'}, {"000101010",'7'},
    {"100101001",'8'}, {"001101001",'9'},
    {"100001001",'A'}, {"001001001",'B'}, {"101001000",'C'}, {"000011001",'D'},
    {"100011000",'E'}, {"001011000",'F'}, {"000001101",'G'}, {"100001100",'H'},
    {"001001100",'I'}, {"000011100",'J'}, {"100000001",'K'}, {"001000001",'L'},
    {"101000000",'M'}, {"000010001",'N'}, {"100010000",'O'}, {"001010000",'P'},
    {"000000011",'Q'}, {"100000010",'R'}, {"001000010",'S'}, {"000010010",'T'},
    {"110000001",'U'}, {"010000001",'V'}, {"111000000",'W'}, {"010010001",'X'},
    {"110010000",'Y'}, {"011010000",'Z'},
    {"010000010",'-'}, {"110000100",'.'}, {"011001001",' '},
    {"010101010",'$'}, {"010110010",'/'}, {"010011010",'+'}, {"001010110",'%'},
    {"010010100",'*'}  // start/stop
};
static int c39_count(void){ return (int)(sizeof(C39)/sizeof(C39[0])); }
static int c39_lookup_bits9(const char* bits){
    for(int i=0;i<c39_count();++i){
        if(C39[i].bits[9]==0 && strncmp(C39[i].bits,bits,9)==0) return i;
    }
    return -1;
}

// ---------- thresholding ----------
static const char* level_from_adc(BarcodeCtx* bc, int raw){
    if (BARCODE_THRESHOLD_MODE == TH_FIXED){
        if(raw <= BLACK_TRIP_ABS) return "BLACK";
        if(raw >= BLACK_EXIT_HYST) return "WHITE";
        return bc->monitor_prev ? bc->monitor_prev : "WHITE";
    } else {
        if(bc->monitor_prev && strcmp(bc->monitor_prev,"WHITE")==0){
            bc->white_mean = (int)(0.98f * bc->white_mean + 0.02f * raw);
            int dev = abs(raw - bc->white_mean);
            bc->white_std  = (int)(0.99f * bc->white_std  + 0.01f * dev);
            int delta = ADAPTIVE_MARGIN_MIN;
            int ksig  = (int)(ADAPTIVE_MARGIN_KSIG * bc->white_std);
            if(ksig > delta) delta = ksig;
            int target = bc->white_mean - delta;
            int max_step = (int)(ADAPTIVE_RATE_LIMIT / BARCODE_MONITOR_HZ);
            if(target > bc->threshold) bc->threshold += max_step;
            else if(target < bc->threshold) bc->threshold -= max_step;
        }
        int enter = bc->threshold;
        int exit  = bc->threshold + (BLACK_EXIT_HYST - BLACK_TRIP_ABS);
        if(raw <= enter) return "BLACK";
        if(raw >= exit)  return "WHITE";
        return bc->monitor_prev ? bc->monitor_prev : "WHITE";
    }
}

static void prebuf_push(BarcodeCtx* bc, uint32_t t_us, int val){
    if(bc->pre_count < PREBUF_MAX_SAMPLES) bc->pre_count++;
    bc->pre_head = (bc->pre_head + 1) % PREBUF_MAX_SAMPLES;
    int idx = (bc->pre_head - 1 + PREBUF_MAX_SAMPLES) % PREBUF_MAX_SAMPLES;
    bc->prebuf[idx].t_us = t_us;
    bc->prebuf[idx].val  = val;
}

static void prebuf_dump_into_burst(BarcodeCtx* bc){
    int n = bc->pre_count;
    int start = (bc->pre_head - n + PREBUF_MAX_SAMPLES) % PREBUF_MAX_SAMPLES;
    for(int i=0;i<n;i++){
        int idx = (start + i) % PREBUF_MAX_SAMPLES;
        if(bc->burst_len < bc->burst_cap){
            bc->burst[bc->burst_len++] = bc->prebuf[idx]; // same BcSample type
        }
    }
}

// ---------- lifecycle ----------
void barcode_init(BarcodeCtx* bc){
    memset(bc, 0, sizeof(*bc));
    bc->th_mode     = BARCODE_THRESHOLD_MODE;
    bc->polarity    = BARCODE_POLARITY;
    bc->white_mean  = (WHITE_WINDOW_MIN + WHITE_WINDOW_MAX)/2;
    bc->white_std   = 10;
    bc->threshold   = BLACK_TRIP_ABS;
    bc->monitor_prev= "WHITE";

    // burst reserve (~pretrigger window + 300ms + margin)
    bc->burst_cap   = (int)((PRETRIGGER_MS/1000.0f + 0.30f) * BARCODE_BURST_HZ) + 2048;
    bc->burst       = malloc(sizeof *bc->burst * bc->burst_cap);
    bc->burst_len   = 0;
    bc->burst_active= false;
    bc->sym_len     = 0;

    g_barcode_singleton = bc;
}

bool barcode_monitor_tick(BarcodeCtx* bc, int adc_pin, int* out_last_raw){
    int raw = adc_read_raw_pin(adc_pin);
    if(bc->polarity == POL_INVERT) raw = ADC_MAX_12B - raw;
    *out_last_raw = raw;

    prebuf_push(bc, now_us(), raw);

    const char* level = level_from_adc(bc, raw);
    bool trig = false;
    if(strcmp(bc->monitor_prev ? bc->monitor_prev : "WHITE", "WHITE")==0 &&
       strcmp(level,"BLACK")==0){
        bc->monitor_prev = "TRIGGERED";
        trig = true;
    }else{
        bc->monitor_prev = level;
    }
    return trig;
}

void barcode_start_burst(BarcodeCtx* bc){
    bc->burst_len = 0;
    prebuf_dump_into_burst(bc);
    bc->burst_active = true;
}

// Called at BARCODE_BURST_HZ in your timer ISR
void barcode_burst_isr(void){
    if(!g_barcode_singleton) return;
    BarcodeCtx* b = g_barcode_singleton;
    if(b->burst_active && b->burst_len < b->burst_cap){
        int raw = adc_read_raw_pin(BARCODE_ADC_PIN);
        if(b->polarity == POL_INVERT) raw = ADC_MAX_12B - raw;
        b->burst[b->burst_len].t_us = now_us();
        b->burst[b->burst_len].val  = raw;
        b->burst_len++;
    }
}

static int has_stable_white_tail(BarcodeCtx* bc, int dwell_ms){
    if(bc->burst_len < (BARCODE_BURST_HZ * dwell_ms / 1000)) return 0;
    int need = (BARCODE_BURST_HZ * dwell_ms / 1000);
    int white_cnt = 0;
    for(int i=bc->burst_len-need;i<bc->burst_len;i++){
        const char* lev = level_from_adc(bc, bc->burst[i].val);
        if(strcmp(lev,"WHITE")==0) white_cnt++;
    }
    return white_cnt >= need*0.9f;
}

bool barcode_burst_complete(BarcodeCtx* bc){
    static uint32_t start_us = 0;
    if(bc->burst_active && bc->burst_len==0){ start_us = now_us(); }
    if(!bc->burst_active) return true;

    if(bc->burst_len > (BARCODE_BURST_HZ * 0.05f)){     // >50ms of data
        if(has_stable_white_tail(bc, 30)){              // 30ms white tail
            bc->burst_active = false; 
            return true;
        }
    }
    if(now_us() - start_us > 600000){                   // 600ms safety cap
        bc->burst_active = false;
        return true;
    }
    return false;
}

// ---------- decoding ----------
typedef struct { const char* level; uint32_t start_us, end_us; } Run;

static int detect_runs(BarcodeCtx* bc, Run* runs, int max_runs){
    if(bc->burst_len <= 0) return 0;
    int count = 0;
    const char* cur = "WHITE";
    uint32_t cur_start = bc->burst[0].t_us;
    for(int i=0;i<bc->burst_len;i++){
        const char* lev = level_from_adc(bc, bc->burst[i].val);
        if(strcmp(lev, cur)!=0){
            if(count < max_runs){
                runs[count++] = (Run){ cur, cur_start, bc->burst[i].t_us };
            }
            cur = lev; cur_start = bc->burst[i].t_us;
        }
    }
    if(count < max_runs){
        runs[count++] = (Run){ cur, cur_start, bc->burst[bc->burst_len-1].t_us };
    }

    // drop too short runs
    uint32_t min_dur = (uint32_t)((MIN_BAR_SAMPLES * 1000000.0f) / BARCODE_BURST_HZ);
    int w=0;
    for(int i=0;i<count;i++){
        uint32_t dur = runs[i].end_us - runs[i].start_us;
        if(dur >= min_dur) runs[w++] = runs[i];
    }
    return w;
}

static float estimate_narrow_unit_from_runs(Run* runs, int n){
    if(n<=0) return 0;
    uint32_t* dur = (uint32_t*)malloc(sizeof(uint32_t)*n);
    for(int i=0;i<n;i++) dur[i] = runs[i].end_us - runs[i].start_us;
    // crude lower-quartile estimate
    for(int i=0;i<n/4;i++){
        int m=i; for(int j=i+1;j<n;j++) if(dur[j]<dur[m]) m=j;
        uint32_t t=dur[i]; dur[i]=dur[m]; dur[m]=t;
    }
    float narrow = (n/4>0)? (float)dur[(n/4)/2] : (float)dur[0];
    free(dur);
    return narrow;
}

static int classify_to_bits(Run* runs, int n, float narrow, char* out_bits, int max_bits){
    int bits=0;
    for(int i=0;i<n;i++){
        float dur = (float)(runs[i].end_us - runs[i].start_us);
        float rW = NARROW_WIDE_RATIO * narrow;
        float loN = narrow*(1.0f-WIDTH_TOLERANCE), hiN = narrow*(1.0f+WIDTH_TOLERANCE);
        float loW = rW*(1.0f-WIDTH_TOLERANCE), hiW = rW*(1.0f+WIDTH_TOLERANCE);
        char bw;
        if(dur>=loN && dur<=hiN) bw='0';
        else if(dur>=loW && dur<=hiW) bw='1';
        else return -1; // reject
        if(bits < max_bits) out_bits[bits++] = bw; else return -1;
    }
    return bits;
}

static int find_star_index(const char* bits, int len){
    const char* star = "010010100";
    int L = 9;
    for(int i=0;i+L<=len;i++){
        if(strncmp(bits+i, star, L)==0) return i;
    }
    return -1;
}

static void reverse_bits(char* bits, int len){
    for(int i=0;i<len/2;i++){ char t=bits[i]; bits[i]=bits[len-1-i]; bits[len-1-i]=t; }
}

bool barcode_decode(BarcodeCtx* bc){
    bc->sym_len = 0;
    if(bc->burst_len < 16) return false;

    const int MAX_RUNS = 4096;
    Run* runs = (Run*)malloc(sizeof(Run)*MAX_RUNS);
    int nr = detect_runs(bc, runs, MAX_RUNS);
    if(nr < 12){ free(runs); return false; }

    char* bits = (char*)malloc(nr+1);
    float narrow = estimate_narrow_unit_from_runs(runs,nr);
    int nbits = classify_to_bits(runs, nr, narrow, bits, nr);
    if(nbits < 0){ free(runs); free(bits); return false; }
    bits[nbits] = 0;

    int idx_fwd = find_star_index(bits, nbits);
    if(idx_fwd < 0){
        reverse_bits(bits, nbits);
        idx_fwd = find_star_index(bits, nbits);
        if(idx_fwd < 0){ free(runs); free(bits); return false; }
    }

    int i = idx_fwd + 9;
    while(i + 9 <= nbits && bc->sym_len < MAX_SYMBOLS_PER_SCAN){
        char tmp[10]; memcpy(tmp, bits+i, 9); tmp[9]=0;
        int k = c39_lookup_bits9(tmp);
        if(k < 0){ free(runs); free(bits); return false; }
        char ch = C39[k].ch;
        if(ch=='*') break;
        bc->symbols[bc->sym_len++] = ch;
        i += 9; // gap consumed by run segmentation
    }

    free(runs); free(bits);
    return (bc->sym_len > 0);
}

DecodedCmd barcode_first_symbol_to_cmd(BarcodeCtx* bc){
    if(bc->sym_len <= 0) return CMD_NONE;
    switch(bc->symbols[0]){
        case 'S': return CMD_STOP;
        case 'U': return CMD_UTURN;
        case 'A': case 'C': return CMD_LEFT;
        case 'B': case 'D': case 'F': return CMD_RIGHT;
        default:  return CMD_NONE;
    }
}
