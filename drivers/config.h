#ifndef CONFIG_H
#define CONFIG_H

// ===== Pins (ADC channels are tied to GP26=ADC0, GP27=ADC1) =====
#define LINE_ADC_PIN        26   // GP26 / ADC0
#define BARCODE_ADC_PIN     27   // GP27 / ADC1

// ===== Polarity =====
typedef enum { POL_NORMAL = 0, POL_INVERT = 1 } Polarity;
#define LINE_POLARITY       POL_NORMAL
#define BARCODE_POLARITY    POL_NORMAL

// ===== Loop & timing =====
#define LINE_LOOP_PERIOD_MS       25            // ~40 Hz
#define BARCODE_MONITOR_HZ        300
#define BARCODE_BURST_HZ          2500
#define PRETRIGGER_MS             80
#define LINE_LOST_MS              500
#define RECOVER_STOP_MAX_MS       250
#define RECOVER_TIMEOUT_MS        3000

// ===== Speeds (m/s) =====
#define DEFAULT_TARGET_SPEED_MPS  0.45f
#define SCAN_SPEED_MAX_MPS        0.35f
#define TURN_SPEED_MPS            0.20f

// ===== PID defaults =====
typedef struct { float Kp, Ki, Kd; } PIDGains;
#define LINE_KP 0.45f
#define LINE_KI 0.02f
#define LINE_KD 0.10f
#define SPEED_KP 0.50f
#define SPEED_KI 0.05f
#define SPEED_KD 0.00f

// ===== Junction detection =====
#define T_JUNCTION_DARK_MS        150
#define JUNCTION_CONFIRM_MS       80
#define JUNCTION_EDGE_DENSITY_MIN 0.3f

// ===== ADC/thresholding =====
#define ADC_MAX_12B               4095
typedef enum { TH_FIXED = 0, TH_ADAPTIVE = 1 } ThresholdMode;
#define BARCODE_THRESHOLD_MODE    TH_FIXED
#define WHITE_WINDOW_MIN          3800
#define WHITE_WINDOW_MAX          3900
#define BLACK_TRIP_ABS            3800
#define BLACK_EXIT_HYST           3850
#define ADAPTIVE_MARGIN_MIN       80
#define ADAPTIVE_MARGIN_KSIG      2.5f
#define ADAPTIVE_RATE_LIMIT       50.0f   // counts per second (adc units/second)

// ===== Code-39 (no checksum) =====
#define NARROW_WIDE_RATIO         3.0f
#define WIDTH_TOLERANCE           0.45f
#define MIN_BAR_SAMPLES           3
#define INTERCHAR_GAP_MIN_N       1.0f
#define INTERCHAR_GAP_MAX_N       3.0f
#define MAX_SYMBOLS_PER_SCAN      16

// ===== Execution policy =====
typedef enum { EXE_AT_NEXT_JUNCTION = 0, EXE_IMMEDIATE = 1 } ExecMode;
#define EXECUTE_MODE              EXE_AT_NEXT_JUNCTION

// ===== Time-based turn fallbacks (if no IMU wired yet) =====
#define TURN_TIME_MS_90           550   // tweak on floor
#define TURN_TIME_MS_180          1050  // tweak on floor

// ===== MQTT topics (stubs) =====
#define MQTT_TOPIC_STATE   "robot/state"
#define MQTT_TOPIC_EVENT   "robot/event"
#define MQTT_TOPIC_METRIC  "robot/metric"

#endif // CONFIG_H
