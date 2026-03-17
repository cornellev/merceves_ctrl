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
#include "stepper.h"

#define ADC_PIN 26
#define STEP_PIN 2
#define DIR_PIN 3
#define ENA_PIN 4

// dc driver
// B is forwards and A is backwards
#define DC_PWM_PIN 0
#define M1INA_PIN 14
#define M1INB_PIN 15
#define M2INA_PIN 12
#define M2INB_PIN 13

// rpm
#define RPM_PIN_L 27
#define RPM_PIN_R 28

// spi
#define SPI_CLK 6
#define SPI_TX 7
#define SPI_RX 8
#define SPI_CS 9

#define LED_PIN 25

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

static inline int pollADC() {
    return floorf((adc_read() * steps_per_rev) / 4095.0f);
}

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

    //printf("reading...\n");

    for (int i = 0; i < 18; i++) {
        uint8_t b = 0;
        //printf("reading %d byte\n", i);

        spi_read_blocking(spi0, tx_buffer[i], &b, 1);
        //printf("read %d byte\n", i);
        rx_buffer[i] = b;
    }

    // spi_read_blocking(spi0, 0x01, &rx_buffer, 18);

    // spi_write_read_blocking(spi0, tx_buffer, rx_buffer, 18);

    //printf("read!\n");

    // if(rx_buffer[0] != 0xAA)
    //     return;
    back:
    checksum = 0;
    for(int i = 1; i < 17; i++) {
        //printf("%x, ", rx_buffer[i]);
        checksum ^= rx_buffer[i];
    }

    //printf("\n");

    if(checksum != rx_buffer[17]) {
        //printf("checksum failed :(");
        uint8_t b = 0;
        while(b != 0xAA) {
            spi_read_blocking(spi0, 0xAA, &b, 1);
        }
        rx_buffer[0] = 0xAA;
        for (int i = 1; i < 18; i++) {
            uint8_t b = 0;
            //printf("reading %d byte\n", i);

            spi_read_blocking(spi0, tx_buffer[i], &b, 1);
            //printf("read %d byte\n", i);
            rx_buffer[i] = b;
        }
        goto back;
        return;
    }

    double speed_cmd;
    double angle_cmd;

    memcpy(&speed_cmd, &rx_buffer[1], sizeof(double));
    memcpy(&angle_cmd, &rx_buffer[9], sizeof(double));

    if(speed_cmd > 0) {
        requested_dir = 0;
    } else {
        requested_dir = 1;
        speed_cmd *= -1;
    }

    requested_speed = (int)round(speed_cmd);

    requested_angle = (int)(400.0f * ((angle_cmd + (m_pi/2.0f)) / (m_pi))) + 600;

    printf("speed_cmd: %lf, angle_cmd: %lf\nrequested speed: %i, requested angle: %i\n", speed_cmd, angle_cmd, requested_speed, requested_angle);
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

int main() {

    stdio_init_all();

    sleep_ms(1000);

    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);

    stepper_init(STEP_PIN, DIR_PIN, ENA_PIN);

    gpio_set_function(DC_PWM_PIN, GPIO_FUNC_PWM);
    uint dc_slice_num = pwm_gpio_to_slice_num(DC_PWM_PIN);

    pwm_set_wrap(dc_slice_num, 2048);
    pwm_set_chan_level(dc_slice_num, PWM_CHAN_A, requested_speed);
    pwm_set_enabled(dc_slice_num, true);

    gpio_init(M1INA_PIN);
    gpio_set_dir(M1INA_PIN, GPIO_OUT);

    gpio_init(M1INB_PIN);
    gpio_set_dir(M1INB_PIN, GPIO_OUT);


    gpio_init(M2INA_PIN);
    gpio_set_dir(M2INA_PIN, GPIO_OUT);

    gpio_init(M2INB_PIN);
    gpio_set_dir(M2INB_PIN, GPIO_OUT);


    current_step_angle = 800;


    gpio_init(RPM_PIN_L);
    gpio_set_dir(RPM_PIN_L, GPIO_IN);
    gpio_set_irq_enabled_with_callback(RPM_PIN_L, GPIO_IRQ_EDGE_RISE, true, &gpio_interrupt);

    gpio_init(RPM_PIN_R);
    gpio_set_dir(RPM_PIN_R, GPIO_IN);
    gpio_set_irq_enabled(RPM_PIN_R, GPIO_IRQ_EDGE_RISE, true);

    spi_init(spi0, 1000*500);
    spi_set_slave(spi0, true);

    gpio_set_function(21, GPIO_FUNC_SPI);
    gpio_set_function(20, GPIO_FUNC_SPI);
    gpio_set_function(19, GPIO_FUNC_SPI);
    gpio_set_function(18, GPIO_FUNC_SPI);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);
    
    multicore_launch_core1(core1_entry);

    int last_time = time_ms();

    while (true) {
        int adc_value = requested_angle;
        //int adc_value = pollADC();
        int error = current_step_angle - adc_value;

        uint dc_slice_num = pwm_gpio_to_slice_num(DC_PWM_PIN);
        pwm_set_chan_level(dc_slice_num, PWM_CHAN_A, requested_speed);

        if(requested_dir == 0) {
            gpio_put(M1INA_PIN, 0);
            gpio_put(M2INA_PIN, 0);
            gpio_put(M1INB_PIN, 1);
            gpio_put(M2INB_PIN, 1);
        } else {
            gpio_put(M1INB_PIN, 0);
            gpio_put(M2INB_PIN, 0);
            gpio_put(M1INA_PIN, 1);
            gpio_put(M2INA_PIN, 1);
        }

        gpio_put(DIR_PIN, error > 0 ? 0 : 1);

        if (abs(error) > margin_of_error) {

            if((error > 0 && current_step_angle > min_step_angle) ||
               (error < 0 && current_step_angle < max_step_angle)) {

                for(int i = 0; i < 25; i++) {

                    gpio_put(STEP_PIN, 1);
                    sleep_us(300);
                    gpio_put(STEP_PIN, 0);
                    sleep_us(300);
                }

                current_step_angle += error > 0 ? -1 : 1;
            }
        }
        
        if(time_ms() - last_time > 100) {
            speed_L = (pulse_count_L * 2 * 2 * M_PI) / magnets_per_wheel;
            speed_R = (pulse_count_R * 2 * 2 * M_PI) / magnets_per_wheel;

            pulse_count_L = 0;
            pulse_count_R = 0;
        }
        //printf("L rad/s: %f  R rad/s: %f\n", speed_L, speed_R);
    }
}
