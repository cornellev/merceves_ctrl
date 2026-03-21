#pragma once
#include <stdbool.h>
#include "pico/stdlib.h"

void stepper_init(uint pin_step, uint pin_dir, uint pin_ena);
void stepper_enable(bool en);
bool stepper_step(bool dir, int step_us);