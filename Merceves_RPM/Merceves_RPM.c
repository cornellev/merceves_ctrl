#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19
// spi
#define SPI_CLK 1
#define SPI_TX 2
#define SPI_RX 3
#define SPI_CS 4


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

volatile bool spi_transfer_requested = false;
int time_ms() {
    return to_ms_since_boot(get_absolute_time());
}
void handle_spi_transfer() {
     uint8_t tx_buffer[18];
    uint8_t rx_buffer[18];

    tx_buffer[0] = 0xAA;

    memcpy(&tx_buffer[1], &speed_L, sizeof(double));
    memcpy(&tx_buffer[9], &speed_R, sizeof(double));

    uint8_t checksum = 0;
    for(int i = 1; i < 17; i++)
        checksum ^= tx_buffer[i];

    tx_buffer[17] = checksum;

    for (int i = 0; i < 18; i++) {
        uint8_t b = 0;
        //printf("reading %d byte\n", i);

        spi_read_blocking(spi0, tx_buffer[i], &b, 1);
        //printf("read %d byte\n", i);
        rx_buffer[i] = b;
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

        
        handle_spi_transfer();
        // sleep_us(100);
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

    spi_init(spi0, 1000*500);
    spi_set_slave(spi0, true);

    gpio_set_function(22, GPIO_FUNC_SPI); //miso
    gpio_set_function(9, GPIO_FUNC_SPI); //gnd

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
