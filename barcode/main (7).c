#include <stdio.h>
#include "pico/stdlib.h"
#include "barcode.h"

static void print_thresholds(void) {
    uint16_t thr_b, thr_w, mid, hys, wavg, bavg, wstd, bstd;
    bool inv;
    barcode_get_thresholds(&thr_b, &thr_w, &mid, &hys, &wavg, &bavg, &wstd, &bstd, &inv);

    printf("\n[CAL] WHITE avg=%u std=%u | BLACK avg=%u std=%u | %s\n",
           wavg, wstd, bavg, bstd, inv ? "INVERTED" : "NORMAL");
    printf("[CAL] Mid=%u  Hys=%u  thr_black<=%u  thr_white>=%u\n",
           mid, hys, thr_b, thr_w);
}

static void print_nw_info_once(void) {
    static bool shown = false;
    if (shown) return;
    bool seeded; uint16_t base, thr; uint16_t fn, fd;
    barcode_get_nw_info(&seeded, &base, &thr, &fn, &fd);
    if (seeded) {
        printf("[NW] narrow_base=%u samples | wide if len >= ~%u (factor=%u/%u)\n",
               base, thr, fn, fd);
        shown = true;
    }
}

int main() {
    stdio_init_all();
    sleep_ms(8000);

    printf("\n=== Barcode Capture and Decode ===\n");
    printf("ADC on GP%d (ADC%d). Button GP%d for BLACK capture.\n",
           BARCODE_ADC_GPIO, BARCODE_ADC_CH, BARCODE_BTN_BLACK);

    barcode_init();
    sleep_ms(100);

    // Guided calibration
    barcode_calibrate();
    print_thresholds();
    sleep_ms(5000);

    printf("\nRunning at %d Hz.\n", BARCODE_SAMPLE_HZ);
    printf("- Runs:  RUN <BLACK/WHITE> len=<samples>\n");
    printf("- Bits:  BIT <bar/space>=<0|1> (0=narrow,1=wide)\n");
    printf("- End:   BIT: <full bitstream>\n");
    printf("- Decode: SYMS: *...* | Decoded: <payload> | Command: <LEFT/RIGHT/NONE>\n\n");

    while (true) {
        barcode_step_300hz(); // paces internally

        // Drain any completed runs
        run_t r;
        while (barcode_get_next_run(&r)) {
            printf("RUN %-5s len=%u\n", (r.tone==BW_BLACK)?"BLACK":"WHITE", r.len);
        }

        // Show NW info once
        print_nw_info_once();

        // Optional: live element bits (commented to reduce spam)
        // uint8_t bit; bool is_bar;
        // while (barcode_get_next_bit(&bit, &is_bar)) {
        //     printf("BIT %s=%u\n", is_bar ? "bar  " : "space", bit);
        // }
    }
    return 0;
}