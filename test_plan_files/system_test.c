#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ----------------------
// CONFIG / THRESHOLDS
// ----------------------
#define PAUSE_BETWEEN_TESTS_MS 2000
#define TELEMETRY_MIN_HZ       2.0f      // >= 2 Hz
#define CMD_MAX_LATENCY_MS     250       // <= 250 ms
#define STOP_DISTANCE_MAX_CM   30        // STOP within 10 cm after decode (system margin up to 30 cm)
#define TURN_ANGLE_TOL_DEG     5.0f      // IMU-assisted 90° turn within ±5°
#define AVOID_SCAN_DEG_MIN     120.0f    // Servo scan span >= 120°
#define AVOID_WIDTH_TOL_MM     20.0f     // Width estimate within ±20 mm
#define AVOID_COMPLETE_MS_MAX  5000      // Rejoin line within 5 s after scan
#define RUN_TIME_MINUTES_MIN   30        // 30 min continuous run
#define MCU_TEMP_MAX_C         60.0f     // < 60 °C (example ceiling)
#define WIFI_RECOVER_MS_MAX    1000      // Safe stop < 1 s on Wi-Fi drop
#define LINE_DRIFT_MAX_CM      2.0f      // Track drift < 2 cm in straight

// ----------------------
// UTILITIES
// ----------------------
static void pause_between_tests(void) {
    printf("\n... (pausing for %d ms) ...\n\n", PAUSE_BETWEEN_TESTS_MS);
    sleep_ms(PAUSE_BETWEEN_TESTS_MS);
}

static void wait_for_enter(const char *msg) {
    if (msg && *msg) printf("%s", msg);
    int ch;
    do { ch = getchar_timeout_us(0); } while (ch != PICO_ERROR_TIMEOUT); // flush
    while (true) {
        ch = getchar_timeout_us(100000);
        if (ch == '\r' || ch == '\n') break;
    }
}

// Get float from console with a prompt
static float ask_float(const char *prompt) {
    char buf[64] = {0};
    printf("%s", prompt);
    int idx = 0;
    while (idx < (int)sizeof(buf) - 1) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) continue;
        if (c == '\r' || c == '\n') break;
        buf[idx++] = (char)c;
    }
    buf[idx] = 0;
    float v = 0.0f;
    if (sscanf(buf, "%f", &v) != 1) v = 0.0f;
    return v;
}

static int ask_int(const char *prompt) {
    char buf[64] = {0};
    printf("%s", prompt);
    int idx = 0;
    while (idx < (int)sizeof(buf) - 1) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) continue;
        if (c == '\r' || c == '\n') break;
        buf[idx++] = (char)c;
    }
    buf[idx] = 0;
    int v = 0;
    if (sscanf(buf, "%d", &v) != 1) v = 0;
    return v;
}

int main() {
    stdio_init_all();
    sleep_ms(3000); // let serial attach

    printf("--- System Testing (End-to-End) ---\n");
    printf("NOTE: These tests validate the full stack: Line, Barcode, PID, Encoders, Ultrasonic+Servo, IMU, MQTT.\n");
    wait_for_enter("\nPress ENTER to begin...\n");

    bool pass;

// ====================================================================
// SYS-T-01: End-to-end line → barcode → obstacle → resume
// ====================================================================
    printf("=== [Test SYS-T-01: End-to-end course] ===\n");
    printf("Expected: Robot follows line, decodes barcode, avoids obstacle, resumes line. Telemetry >= 2 Hz.\n");
    wait_for_enter("1) Place robot at track start. Ensure obstacle ahead and a valid barcode (e.g., STOP or LEFT). Press ENTER to start.\n");

    // Operator supplies observed numbers
    float drift_cm          = ask_float("Enter observed max line drift on straight (cm): ");
    float telemetry_rate_hz = ask_float("Enter observed telemetry rate (Hz): ");
    int   resumed_ok        = ask_int("Did robot resume line after obstacle? (1=yes,0=no): ");

    printf("\n--- Actual State ---\n");
    printf("  Drift: %.2f cm\n", drift_cm);
    printf("  Telemetry rate: %.2f Hz\n", telemetry_rate_hz);
    printf("  Resume after obstacle: %s\n", resumed_ok ? "Yes" : "No");

    pass = (drift_cm <= LINE_DRIFT_MAX_CM) && (telemetry_rate_hz >= TELEMETRY_MIN_HZ) && (resumed_ok == 1);
    printf("Result: %s\n", pass ? "Pass" : "Fail");
    pause_between_tests();

// ====================================================================
// SYS-T-02: Telemetry + remote STOP command
// ====================================================================
    printf("=== [Test SYS-T-02: Telemetry & Remote Command] ===\n");
    printf("Expected: Telemetry updates <= 500 ms; STOP command latency <= %d ms; robot halts safely.\n", CMD_MAX_LATENCY_MS);
    wait_for_enter("1) Run robot in AUTO.\n2) From dashboard send STOP.\n3) Measure command latency.\nPress ENTER when done...\n");

    float cmd_latency_ms = ask_float("Enter measured command latency (ms): ");
    int   halted_ok      = ask_int("Did robot stop within ~0.5 s? (1=yes,0=no): ");

    printf("\n--- Actual State ---\n");
    printf("  Command latency: %.1f ms\n", cmd_latency_ms);
    printf("  Halted: %s\n", halted_ok ? "Yes" : "No");

    pass = (cmd_latency_ms <= CMD_MAX_LATENCY_MS) && (halted_ok == 1);
    printf("Result: %s\n", pass ? "Pass" : "Fail");
    pause_between_tests();

// ====================================================================
// SYS-T-03: IMU-assisted 90° turns
// ====================================================================
    printf("=== [Test SYS-T-03: IMU-Assisted 90-degree Turns] ===\n");
    printf("Expected: Final heading within ±%.1f°; line reacquired within 3 s.\n", TURN_ANGLE_TOL_DEG);
    wait_for_enter("1) Run through LEFT/RIGHT barcode section.\n2) Measure heading error after turn.\nPress ENTER when done...\n");

    float heading_err_deg = ask_float("Enter |heading error| (deg): ");
    int   reacquired_ms   = ask_int("Enter time to reacquire line (ms): ");

    printf("\n--- Actual State ---\n");
    printf("  Heading error: %.2f deg\n", heading_err_deg);
    printf("  Reacquire time: %d ms\n", reacquired_ms);

    pass = (fabsf(heading_err_deg) <= TURN_ANGLE_TOL_DEG) && (reacquired_ms <= 3000);
    printf("Result: %s\n", pass ? "Pass" : "Fail");
    pause_between_tests();

// ====================================================================
// SYS-T-04: Obstacle detect + scan + avoid
// ====================================================================
    printf("=== [Test SYS-T-04: Obstacle Detect, Scan & Avoid] ===\n");
    printf("Expected: Stop <= 300 mm from obstacle, scan >= %.0f°, width within ±%.0f mm, rejoin <= %d ms.\n",
           AVOID_SCAN_DEG_MIN, AVOID_WIDTH_TOL_MM, AVOID_COMPLETE_MS_MAX);
    wait_for_enter("1) Place obstacle ~200 mm ahead.\n2) Run robot; let it scan and avoid.\nPress ENTER when done...\n");

    int   stop_dist_mm     = ask_int("Enter stop distance before obstacle (mm): ");
    float scan_span_deg    = ask_float("Enter servo scan span (deg): ");
    float width_err_mm     = ask_float("Enter |width estimation error| (mm): ");
    int   rejoin_ms        = ask_int("Enter time to rejoin line (ms): ");

    printf("\n--- Actual State ---\n");
    printf("  Stop distance: %d mm\n", stop_dist_mm);
    printf("  Scan span: %.1f deg\n", scan_span_deg);
    printf("  Width error: %.1f mm\n", width_err_mm);
    printf("  Rejoin time: %d ms\n", rejoin_ms);

    pass = (stop_dist_mm <= 300) &&
           (scan_span_deg >= AVOID_SCAN_DEG_MIN) &&
           (fabsf(width_err_mm) <= AVOID_WIDTH_TOL_MM) &&
           (rejoin_ms <= AVOID_COMPLETE_MS_MAX);
    printf("Result: %s\n", pass ? "Pass" : "Fail");
    pause_between_tests();

// ====================================================================
// SYS-T-05: 30-minute stability run
// ====================================================================
    printf("=== [Test SYS-T-05: Stability 30-minute Run] ===\n");
    printf("Expected: No resets/faults; continuous telemetry; MCU temp < %.0f °C.\n", MCU_TEMP_MAX_C);
    wait_for_enter("1) Run robot for >= 30 min loop.\n2) Observe temp & stability.\nPress ENTER when done...\n");

    int   run_minutes   = ask_int("Enter continuous run time (minutes): ");
    float mcu_temp_c    = ask_float("Enter max MCU temperature (°C): ");
    int   faults_occured= ask_int("Any resets/faults? (1=yes,0=no): ");
    int   telemetry_ok  = ask_int("Telemetry continuous? (1=yes,0=no): ");

    printf("\n--- Actual State ---\n");
    printf("  Run time: %d min\n", run_minutes);
    printf("  Max temp: %.1f °C\n", mcu_temp_c);
    printf("  Resets/Faults: %s\n", faults_occured ? "Yes" : "No");
    printf("  Telemetry continuous: %s\n", telemetry_ok ? "Yes" : "No");

    pass = (run_minutes >= RUN_TIME_MINUTES_MIN) &&
           (mcu_temp_c < MCU_TEMP_MAX_C) &&
           (faults_occured == 0) &&
           (telemetry_ok == 1);
    printf("Result: %s\n", pass ? "Pass" : "Fail");
    pause_between_tests();

// ====================================================================
// SYS-T-06: Safe-state on Wi-Fi drop, auto recovery
// ====================================================================
    printf("=== [Test SYS-T-06: Safe-state on Wi-Fi Drop & Recovery] ===\n");
    printf("Expected: Safe stop < %d ms; resumes after reconnection.\n", WIFI_RECOVER_MS_MAX);
    wait_for_enter("1) Run robot in AUTO.\n2) Cut Wi-Fi (AP off or RF block).\n3) Restore Wi-Fi.\nPress ENTER when done...\n");

    int stop_ms   = ask_int("Enter time from drop to safe stop (ms): ");
    int resumed   = ask_int("Did robot resume after reconnection? (1=yes,0=no): ");

    printf("\n--- Actual State ---\n");
    printf("  Stop delay: %d ms\n", stop_ms);
    printf("  Resume: %s\n", resumed ? "Yes" : "No");

    pass = (stop_ms <= WIFI_RECOVER_MS_MAX) && (resumed == 1);
    printf("Result: %s\n", pass ? "Pass" : "Fail");

    printf("\n--- ALL SYSTEM TESTS COMPLETE ---\n");
    while (true) { sleep_ms(1000); }
    return 0;
}
