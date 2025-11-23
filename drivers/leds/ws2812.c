#include "ws2812.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

// --- PIO Program ---
#define ws2812_T1 2
#define ws2812_T2 5
#define ws2812_T3 3

static const uint16_t ws2812_program_instructions[] = { 0x6221, 0x1123, 0x1400, 0xa442 };
static const struct pio_program ws2812_program = { .instructions = ws2812_program_instructions, .length = 4, .origin = -1 };

static inline pio_sm_config ws2812_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset, offset + 3);
    sm_config_set_sideset(&c, 1, false, false);
    return c;
}

static PIO pixel_pio = pio0;
static uint pixel_sm;

static inline void pio_put_pixel(uint32_t pixel_grb) { 
    pio_sm_put_blocking(pixel_pio, pixel_sm, pixel_grb << 8u); 
}
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) { 
    return ((uint32_t)(g) << 16) | ((uint32_t)(r) << 8) | (uint32_t)(b); 
}

void leds_init(void) {
    set_sys_clock_khz(125000, true); // Standard requirement for PIO/WS2812 timing
    pixel_sm = pio_claim_unused_sm(pixel_pio, true);
    uint offset = pio_add_program(pixel_pio, &ws2812_program);
    
    pio_gpio_init(pixel_pio, WS2812_PIN); // From sys_config.h
    pio_sm_set_consecutive_pindirs(pixel_pio, pixel_sm, WS2812_PIN, 1, true);
    
    pio_sm_config c = ws2812_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, WS2812_PIN);
    sm_config_set_out_shift(&c, false, true, 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    
    int cycles_per_bit = ws2812_T1 + ws2812_T2 + ws2812_T3;
    float div = (float)clock_get_hz(clk_sys) / (WS2812_FREQ * cycles_per_bit);
    sm_config_set_clkdiv(&c, div);
    
    pio_sm_init(pixel_pio, pixel_sm, offset, &c);
    pio_sm_set_enabled(pixel_pio, pixel_sm, true);
}

void update_leds(uint8_t r0, uint8_t g0, uint8_t b0, uint8_t r1, uint8_t g1, uint8_t b1) {
    pio_put_pixel(urgb_u32(r0, g0, b0)); 
    pio_put_pixel(urgb_u32(r1, g1, b1)); 
}

// Smart Indicator Logic
static uint32_t led_timer = 0;
static bool led_blink_state = false;

void handle_smart_indicators(float steer) {
    led_timer++;
    if (led_timer > 10) { led_blink_state = !led_blink_state; led_timer = 0; }
    
    if (steer < -12.0f) { // Turning Right (Left motor dominant)
        if (led_blink_state) update_leds(120, 60, 0, 0, 0, 0); else update_leds(0, 0, 0, 0, 0, 0);
    } else if (steer > 12.0f) { // Turning Left
        if (led_blink_state) update_leds(0, 0, 0, 120, 60, 0); else update_leds(0, 0, 0, 0, 0, 0);
    } else { 
        update_leds(0, 0, 20, 0, 0, 20); // Blue tail lights
    }
}