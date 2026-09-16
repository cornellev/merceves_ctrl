#include "stepper.h"
#include "pico/stdlib.h"

static uint STEP_PIN;
static uint DIR_PIN;
static uint ENA_PIN;

static absolute_time_t last_step_time;

void stepper_init(uint pin_step, uint pin_dir, uint pin_ena) {
    STEP_PIN = pin_step;
    DIR_PIN = pin_dir;
    ENA_PIN = pin_ena;

    gpio_init(STEP_PIN);
    gpio_init(DIR_PIN);
    gpio_init(ENA_PIN);

    gpio_set_dir(STEP_PIN, GPIO_OUT);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_set_dir(ENA_PIN, GPIO_OUT);

    stepper_enable(true);
    last_step_time = get_absolute_time();
}

void stepper_enable(bool en) {
    gpio_put(ENA_PIN, en ? 1 : 0); // Active high enable
}

bool stepper_step(bool dir, int step_us) {
    absolute_time_t now = get_absolute_time();
   if (absolute_time_diff_us(last_step_time, now) >= step_us) {
        last_step_time = now;
        gpio_put(DIR_PIN, dir ? 1 : 0);
        gpio_put(STEP_PIN, 1);
        sleep_us(step_us / 2);
        gpio_put(STEP_PIN, 0);
        sleep_us(step_us / 2);
        return true;
    }
    return false;
}