#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "stepper.h"
#include <math.h>

#define ADC_PIN 26
#define STEP_PIN 2
#define DIR_PIN 3
#define ENA_PIN 4

//dc driver
#define DC_PWM_PIN 0
#define M1INA_PIN 14
#define M1INB_PIN 15
#define M2INA_PIN 12
#define M2INB_PIN 13

//rpm
#define RPM_PIN_L 27
#define RPM_PIN_R 28

//spi
#define SPI_CLK 6
#define SPI_TX 7
#define SPI_RX 8
#define SPI_CS 9

#define M_PI 3.14159265358979323846

const int steps_per_rev = 1600;
const int margin_of_error = 20;

const int min_step_angle = 600;
const int max_step_angle = 1000;

volatile int current_step_angle;
volatile int pulse_count_L = 0;
volatile int pulse_count_R = 0;

volatile int requested_speed = 0;
volatile int requested_angle = 1000;

volatile double speed_L = 0;
volatile double speed_R = 0;

const int magnets_per_wheel = 68;
const double radius = .86;

void gpio_interrupt(uint gpio, uint32_t events) {
    printf("interrupted!!!!");
    switch(gpio) {
        case RPM_PIN_L:
            if(events == GPIO_IRQ_EDGE_RISE) pulse_count_L++;
            break;
        case RPM_PIN_R:
            if(events == GPIO_IRQ_EDGE_RISE) pulse_count_R++;
            break;
        case SPI_CS:
        printf("spi_cs triggered");
            if(events != GPIO_IRQ_EDGE_FALL) break;
            uint8_t buffer[18];
            buffer[0] = 0xBB;
            speed_R = 10.5f;
            for(int i = 1; i < 9; i++) {
                buffer[i] = (uint8_t)((uint64_t)(speed_L)>>(8 * (i-1))) & 0xFF;
                printf("left wheel data recieved");
            }
            for(int i = 9; i < 17; i++) {
                buffer[i] = (uint8_t)((uint64_t)(speed_R)>>(8 * (i-9))) & 0xFF;
                printf("right wheel data recieved");
            }
            uint8_t checksum = 0;
            for(int i = 0; i < 17; i++) {
                checksum ^= buffer[i];
            }
            buffer[17] = checksum;
            for (int i = 0; i < 18; i++) {
                printf("%02X ", buffer[i]);
            }
            printf("\n");
            uint8_t buffer_out[18];
            spi_write_read_blocking(spi0, buffer, buffer_out, 18);
            
            checksum = 0;
            for(int i = 0; i < 17; i++) {
                checksum ^= buffer_out[i];
            }
            if(buffer_out[17] != checksum) {
                printf("checksum failed :(");
                break;
            }
            
            uint64_t speed;
            for(int i = 1; i < 9; i++) {
                speed |= buffer_out[i] << (8 * (i-1));
            }
            double speed_d = (double)speed;
            requested_speed = (int)round(speed_d);
            printf("speed: %lf\n", speed_d);
            printf("requested speed: %lf\n", requested_speed);

            uint64_t angle_int;
            for(int i = 9; i < 17; i++) {
                angle_int |= buffer_out[i] << (8 * (i-9));
            }
            printf("angle %llu\n", angle_int);
            double angle_d = (double)speed;
            printf("angle %lf:", angle_d);
            requested_angle = (int)(1000 * ((angle_d + (M_PI/2))/(M_PI)));
            printf("requested angle %lf:", requested_angle);
    }
}

static inline int pollADC() {
    return floorf((adc_read() * steps_per_rev) / 4095.0f);
}
//sending back hall sensor from the left and hall sensor from the right and the checksum

void core1_entry() {
    while (true) {
        //int adc_value = pollADC();
        int adc_value = requested_angle;
        int error = current_step_angle - adc_value;

        uint dc_slice_num = pwm_gpio_to_slice_num(DC_PWM_PIN);
        pwm_set_chan_level(dc_slice_num, PWM_CHAN_A, requested_speed);
        gpio_put(DIR_PIN, error > 0 ? 0 : 1);

        if (abs(error) > margin_of_error) {
            if(error > 0 && current_step_angle > min_step_angle || error < 0 && current_step_angle < max_step_angle) {
                for(int i = 0; i < 5; i++ ) {
                    gpio_put(STEP_PIN, 1);
                    sleep_us(100);   // Slightly longer than microseconds
                    gpio_put(STEP_PIN, 0);
                    sleep_us(100);   // Slightly longer than microseconds
                }
                current_step_angle += error > 0 ? -1 : 1;
            }
        }

        sleep_us(100);  // Give USB time to flush
    }
}

int main() {
    stdio_init_all();
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);

    stepper_init(STEP_PIN, DIR_PIN, ENA_PIN);

    gpio_set_function(DC_PWM_PIN, GPIO_FUNC_PWM);
    uint dc_slice_num = pwm_gpio_to_slice_num(DC_PWM_PIN);

    pwm_set_enabled(dc_slice_num, true);
    pwm_set_wrap(dc_slice_num, 2048); // 16-bit resolution
    pwm_set_chan_level(dc_slice_num, PWM_CHAN_A, requested_speed); 

    gpio_init(M1INA_PIN);
    gpio_set_dir(M1INA_PIN, GPIO_OUT);
    gpio_put(M1INA_PIN, 1);

    gpio_init(M2INA_PIN);
    gpio_set_dir(M2INA_PIN, GPIO_OUT);
    gpio_put(M2INA_PIN, 1);

    current_step_angle = 800; // Start at the middle position
    multicore_launch_core1(core1_entry);

    //rpm
    gpio_init(RPM_PIN_L);
    gpio_set_dir(RPM_PIN_L, GPIO_IN);
    gpio_set_irq_enabled_with_callback(RPM_PIN_L, GPIO_IRQ_EDGE_RISE, true, &gpio_interrupt);

    gpio_init(RPM_PIN_R);
    gpio_set_dir(RPM_PIN_R, GPIO_IN);
    gpio_set_irq_enabled_with_callback(RPM_PIN_R, GPIO_IRQ_EDGE_RISE, true, &gpio_interrupt);

    gpio_init(SPI_CS);
    gpio_set_dir(SPI_CS, GPIO_IN);
    gpio_set_irq_enabled_with_callback(SPI_CS, GPIO_IRQ_EDGE_FALL, true, &gpio_interrupt);

    spi_init(spi0, 1000 * 100);
    spi_set_slave(spi0, true);
    gpio_set_function(SPI_RX, GPIO_FUNC_SPI);
    gpio_set_function(SPI_CLK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_TX, GPIO_FUNC_SPI);
    gpio_set_function(SPI_CS, GPIO_FUNC_SPI);

    while (true) {
        sleep_ms(100);
        speed_L = (pulse_count_L* 2 * 2 * 3.141592) / magnets_per_wheel; // Convert to RPM
        printf("RPM (rad/s): %f\n", speed_L);
        pulse_count_L = 0; // Reset pulse count for the next interval

        speed_R = (pulse_count_R* 2 * 2 * 3.141592) / magnets_per_wheel; // Convert to RPM
        printf("RPM (rad/s): %f\n", speed_R);
        pulse_count_R = 0; // Reset pulse count for the next interval
    }

    
}
