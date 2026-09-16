#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"

// Telemetry used to go out over an SPI slave port; it now streams out
// the Pico's native USB (USB-CDC "virtual UART") instead, so no SPI
// peripheral or GPIO pin muxing is needed here anymore.

// rpm
#define RPM_PIN_L 27
#define RPM_PIN_R 28

const double m_pi = 3.14159265358979323846f;

const int steps_per_rev = 1600;
const int margin_of_error = 2;

const int min_step_angle = 600;
const int max_step_angle = 1000;

volatile int current_step_angle;
volatile int pulse_count_L = 0;
volatile int pulse_count_R = 0;

volatile int requested_speed = 0;
volatile int requested_dir = 0; //0 is forward, 1 is backward
volatile int requested_angle = 800;

volatile double speed_L = 0;
volatile double speed_R = 0;

const int magnets_per_wheel = 68;

int time_ms() {
    return to_ms_since_boot(get_absolute_time());
}

// There's no SPI master clocking us anymore, so instead of replying to
// a transfer request this board just pushes a fresh telemetry frame out
// over USB serial on its own schedule (see core1_entry's sleep below):
// [0] = 0xAA (start byte)
// [1..8] = left wheel speed (double, 8 bytes)
// [9..16] = right wheel speed (double, 8 bytes)
// [17] => XOR checksum of bytes [1..16]
void handle_uart_transfer() {
    uint8_t tx_buffer[18];

    tx_buffer[0] = 0xAA;

    memcpy(&tx_buffer[1], &speed_L, sizeof(double));
    memcpy(&tx_buffer[9], &speed_R, sizeof(double));

    uint8_t checksum = 0;
    for(int i = 1; i < 17; i++)
        checksum ^= tx_buffer[i];

    tx_buffer[17] = checksum;

    for (int i = 0; i < 18; i++) {
        putchar_raw(tx_buffer[i]);
    }
}
void gpio_interrupt(uint gpio, uint32_t events) {
    if(gpio == RPM_PIN_L && (events & GPIO_IRQ_EDGE_RISE)) {
        pulse_count_L++;
    }

    if(gpio == RPM_PIN_R && (events & GPIO_IRQ_EDGE_RISE)) {
        pulse_count_R++;
    }
}
void core1_entry() {

    while (true) {
        handle_uart_transfer();
        // Pace telemetry pushes - USB serial has no master clock to
        // throttle us the way the old SPI slave link did.
        sleep_ms(20);
    }
}
int main()
{
    stdio_init_all();

    gpio_init(RPM_PIN_L);
    gpio_set_dir(RPM_PIN_L, GPIO_IN);
    gpio_set_irq_enabled_with_callback(RPM_PIN_L, GPIO_IRQ_EDGE_RISE, true, &gpio_interrupt);

    gpio_init(RPM_PIN_R);
    gpio_set_dir(RPM_PIN_R, GPIO_IN);
    gpio_set_irq_enabled(RPM_PIN_R, GPIO_IRQ_EDGE_RISE, true);

    multicore_launch_core1(core1_entry);
    int last_time = time_ms();
    while (true) {
        if(time_ms() - last_time > 100) {
            speed_L = (pulse_count_L * 2 * 2 * M_PI) / magnets_per_wheel;
            speed_R = (pulse_count_R * 2 * 2 * M_PI) / magnets_per_wheel;

            pulse_count_L = 0;
            pulse_count_R = 0;
        }
    }
}
