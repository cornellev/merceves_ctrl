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
#define M1INA_PIN 16
#define M1INB_PIN 17
#define M2INA_PIN 12
#define M2INB_PIN 13

//rpm pins
#define LEFT_WHEEL_PIN  10
#define RIGHT_WHEEL_PIN 11

// spi
#define SPI_PORT spi0
#define SPI_CLK 18
#define SPI_TX 19
#define SPI_RX 20
#define SPI_CS 21

#define LED_PIN 25

const double m_pi = 3.14159265358979323846f;

const int steps_per_rev = 1600;
const int margin_of_error = 2;

const int min_step_angle = 0;
const int max_step_angle = 2000;

volatile int current_step_angle;

volatile int requested_speed = 0;
volatile int requested_dir = 0; //0 is forward, 1 is backward
volatile int requested_angle = 1000;

const int magnets_per_rev = 68;

volatile int left_pulse = 0;
volatile int right_pulse = 0;

volatile double left_rpm = 0;
volatile double right_rpm = 0;

volatile bool step_state = false;
volatile int steps_remaining = 0;
struct repeating_timer step_timer;
struct repeating_timer rpm_timer;

static inline int pollADC() {
    return floorf((adc_read() * steps_per_rev) / 4095.0f);
}

int time_ms() {
    return to_ms_since_boot(get_absolute_time());
}

void handle_spi_transfer() {

    // spi input is in the format of:
    // [0] = 0xAA (start byte)
    // [1..9] = speed data (double, 8 bytes)
    // [10...17] = steering data (double, 8 bytes)
    // 18 => XOR checksum of bytes [0..17]

    // first we'll wait till we see a 0xAA byte
    printf("Waiting for data...\n"); 
    bool data_received = false;
    while (!data_received) {
        uint8_t rx_byte = 0;
        spi_read_blocking(SPI_PORT, 0x00, &rx_byte, 1);
        if (rx_byte == 0xAA) {
            data_received = true;
        }
    }

    printf("Data received, entering main loop...\n");
    
    // stupid ahh spi read byte by byte because pico sdk spi_read_blocking is broken
    for (int i = 0; i < 17; i++) {
        uint8_t byte = 0;
        spi_read_blocking(SPI_PORT, 0x00, NULL, 1);
    }

    while (true) {
        // do a wait for start byte again
        data_received = false;
        while (!data_received) {
            uint8_t rx_byte = 0;
            spi_read_blocking(SPI_PORT, 0x00, &rx_byte, 1);
            if (rx_byte == 0xAA) {
                data_received = true;
            }
        }

        // get data from spi
        uint8_t rx_data[17] = {0};

        // stupid ahh spi read byte by byte because pico sdk spi_read_blocking is broken
        for (int i = 0; i < 17; i++) {
            uint8_t byte = 0;
            spi_read_blocking(SPI_PORT, 0x00, &byte, 1);
            //printf("Read byte %d: %02X\n", i, byte);
            rx_data[i] = byte;
        }
        
        
        double speed = 0.0;
        double steering = 0.0;

        // calculate checksum
        uint8_t checksum = 0;
        for (int i = 0; i < 16; i++) {
            checksum ^= rx_data[i];
        }

        //printf("Received checksum: %02X, Calculated checksum: %02X\n", rx_data[16], checksum);

        if (checksum == rx_data[16]) { // make sure checksum matches
            // valid data, extract speed and steering
            memcpy(&speed, &rx_data[0], sizeof(double));
            memcpy(&steering, &rx_data[8], sizeof(double));

            if(speed > 0) {
                requested_dir = 0;
            } else {
                requested_dir = 1;
                speed *= -1;
            }

            requested_speed = (int)round(speed);

            requested_angle = (int)(2000.0f * ((steering + (m_pi/2.0f)) / (m_pi))); 

            printf("speed_cmd: %lf, angle_cmd: %lf\nrequested speed: %i, requested angle: %i\n", speed, steering, requested_speed, requested_angle);
            
        } else {
            //printf("Checksum error!\n");
        }
    } 

    
}

bool stepper_timer_callback(struct repeating_timer *t) {
    if (steps_remaining > 0) {
        step_state = !step_state;
        gpio_put(STEP_PIN, step_state);

        if (step_state) { //only counting on the rising edge
            steps_remaining--;
            if (requested_angle < current_step_angle) {
                current_step_angle--;
            } else if (requested_angle > current_step_angle) {
                current_step_angle++;
            }
        }
    } else {
        gpio_put(STEP_PIN, 0);
    }
    return true;
}

void rpm_timer_callback(struct repeating_timer *t) {
    static absolute_time_t last_time;
    absolute_time_t now = get_absolute_time();

    float dt = absolute_time_diff_us(last_time, now) / 1e6;
    last_time = now;

    int left, right;

    // atomic snapshot
    uint32_t irq_state = save_and_disable_interrupts();
    left = left_pulse;
    right = right_pulse;
    left_pulse = 0;
    right_pulse = 0;
    restore_interrupts(irq_state);

    float left_rpm  = (left  * 60.0f) / (magnets_per_rev * dt);
    float right_rpm = (right * 60.0f) / (magnets_per_rev * dt);

    printf("L: %.2f RPM | R: %.2f RPM\n", left_rpm, right_rpm);

    return true;
}

void hall_sensor_callback(uint gpio, uint32_t events) {
    if(gpio==LEFT_WHEEL_PIN) {
        left_pulse++;
    }
    else if(gpio==RIGHT_WHEEL_PIN) {
        right_pulse++;
    }
}

void core1_entry() {

    while (true) {
        handle_spi_transfer();
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


    current_step_angle = 1000;

    gpio_init(RIGHT_WHEEL_PIN);
    gpio_set_dir(RIGHT_WHEEL_PIN, GPIO_IN);

    gpio_init(LEFT_WHEEL_PIN);
    gpio_set_dir(LEFT_WHEEL_PIN, GPIO_IN);

    gpio_set_irq_enabled_with_callback(
        RIGHT_WHEEL_PIN,
        GPIO_IRQ_EDGE_FALL,
        true,
        &hall_sensor_callback
    );

    gpio_set_irq_enabled_with_callback(
        LEFT_WHEEL_PIN,
        GPIO_IRQ_EDGE_FALL,
        true,
        &hall_sensor_callback
    );



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
    add_repeating_timer_us(-300, stepper_timer_callback, NULL, &step_timer);
    add_repeating_timer_ns(-6667, rpm_timer_callback, NULL, &rpm_timer);

    while (true) {
        int adc_value = requested_angle;
        int error = current_step_angle - adc_value;

        uint dc_slice_num = pwm_gpio_to_slice_num(DC_PWM_PIN);
        pwm_set_chan_level(dc_slice_num, PWM_CHAN_A, requested_speed);
        //pwm_set_chan_level(dc_slice_num, PWM_CHAN_A, 1000);

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
                gpio_put(DIR_PIN, error > 0 ? 0 : 1);
                steps_remaining = 1;
            }
        }
        
    }
}
