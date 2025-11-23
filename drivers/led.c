#include "led.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

// --- Constants ---
#define WS2812_PIN 18
#define NUM_PIXELS 2
#define WS2812_FREQ 800000

// --- PIO Globals ---
static PIO pixel_pio = pio0;
static uint pixel_sm;

// --- Raw PIO assembly instructions for WS2812 timing ---
// This is the raw machine code for the WS2812 protocol
static const uint16_t ws2812_program_instructions[] = { 
    0x6221, //  0: out    x, 1        side 0 [2] 
    0x1123, //  1: jmp    !x, 3       side 1 [1] 
    0x1400, //  2: jmp    0           side 1     
    0xa442  //  3: nop                side 0 [4] 
};

static const struct pio_program ws2812_program = {
    .instructions = ws2812_program_instructions,
    .length = 4,
    .origin = -1
};

// Internal config helper
static inline pio_sm_config ws2812_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset, offset + 3);
    sm_config_set_sideset(&c, 1, false, false);
    return c;
}

// --- Initialization Function ---
void led_init(void) {
    // 1. Load the PIO program
    uint offset = pio_add_program(pixel_pio, &ws2812_program);

    // 2. Claim a state machine
    pixel_sm = pio_claim_unused_sm(pixel_pio, true);

    // 3. Configure the State Machine
    pio_sm_config c = ws2812_program_get_default_config(offset);

    // 4. Setup the specific pin
    pio_gpio_init(pixel_pio, WS2812_PIN);
    pio_sm_set_consecutive_pindirs(pixel_pio, pixel_sm, WS2812_PIN, 1, true);
    sm_config_set_sideset_pins(&c, WS2812_PIN);
    
    // 5. Configure FIFO to auto-push (optional but good for DMA, keeping off for now to match logic)
    sm_config_set_out_shift(&c, false, true, 24); // Shift left, autopull, threshold 24
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    // 6. Set Frequency (Critical for WS2812 color accuracy)
    float div = clock_get_hz(clk_sys) / (WS2812_FREQ * 10.0f); // 10 cycles per bit in this PIO code
    sm_config_set_clkdiv(&c, div);

    // 7. Start the State Machine
    pio_sm_init(pixel_pio, pixel_sm, offset, &c);
    pio_sm_set_enabled(pixel_pio, pixel_sm, true);
}

// --- Internal Helpers ---

static inline void pio_put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pixel_pio, pixel_sm, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b);
}

// --- Public LED Functions ---

// Set two LEDs at once (Left and Right)
void update_leds(uint8_t r0, uint8_t g0, uint8_t b0, uint8_t r1, uint8_t g1, uint8_t b1) {
    pio_put_pixel(urgb_u32(r0, g0, b0));
    pio_put_pixel(urgb_u32(r1, g1, b1));
}

// --- Smart Indicators Logic ---

// Static state variables strictly for the LED logic
static uint32_t led_timer = 0;
static bool led_blink_state = false;

void handle_smart_indicators(float steer) {
    led_timer++;
    
    // Toggle blink state every 10 ticks
    if (led_timer > 10) { 
        led_blink_state = !led_blink_state; 
        led_timer = 0; 
    }

    if (steer < -12.0f) { 
        // Turning Left - Blink Left Orange
        if (led_blink_state) update_leds(120, 60, 0, 0, 0, 0); 
        else update_leds(0, 0, 0, 0, 0, 0);
    } else if (steer > 12.0f) { 
        // Turning Right - Blink Right Orange
        if (led_blink_state) update_leds(0, 0, 0, 120, 60, 0); 
        else update_leds(0, 0, 0, 0, 0, 0);
    } else { 
        // Going Straight - Dim Blue
        update_leds(0, 0, 20, 0, 0, 20);
    }
}