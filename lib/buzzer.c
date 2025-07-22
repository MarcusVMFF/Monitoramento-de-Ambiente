#ifndef BUZZER_H
#define BUZZER_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/time.h"

// Declaração antecipada (forward declaration)
int64_t buzzer_stop(alarm_id_t id, void *slice_num);

// Inicializa o buzzer
void buzzer_init(uint buzzer_pin) {
    gpio_set_function(buzzer_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(buzzer_pin);
    
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0f); // Divisor para base de 1MHz
    pwm_config_set_wrap(&config, 1000);     // Valor inicial do wrap
    pwm_init(slice_num, &config, false);    // Inicia desligado
    
    pwm_set_gpio_level(buzzer_pin, 0);
}

// Toca um beep com duração específica
void buzzer_beep(uint buzzer_pin, uint freq_hz, uint duration_ms) {
    if (freq_hz == 0 || duration_ms == 0) {
        pwm_set_gpio_level(buzzer_pin, 0);
        return;
    }

    uint slice_num = pwm_gpio_to_slice_num(buzzer_pin);
    uint32_t wrap = 1000000 / freq_hz;
    
    pwm_set_wrap(slice_num, wrap);
    pwm_set_gpio_level(buzzer_pin, wrap / 2);
    pwm_set_enabled(slice_num, true);
    
    add_alarm_in_ms(duration_ms, buzzer_stop, (void*)slice_num, true);
}

// Para o buzzer (callback do alarme)
int64_t buzzer_stop(alarm_id_t id, void *slice_num) {
    (void)id; // Não utilizado
    pwm_set_enabled((uint)slice_num, false);
    return 0;
}

#endif
