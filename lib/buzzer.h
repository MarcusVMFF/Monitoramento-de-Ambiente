#ifndef BUZZER_H
#define BUZZER_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"

void buzzer_init(uint buzzer_pin);
void buzzer_beep(uint buzzer_pin, uint freq_hz, uint duration_ms);
void buzzer_stop(uint slice_num);
#endif
