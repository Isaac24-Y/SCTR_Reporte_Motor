#include "VelocityHandler.h"
#include "EncoderHandler.h"
#include <Arduino.h>
#include "esp_timer.h"

// ------------------- Configuración -------------------
#define VELOCITY_TASK_PERIOD_MS 5
#define VELOCITY_TASK_PRIORITY  3
#define VELOCITY_TASK_STACK     2048

// Encoder incremental: pulsos por revolución (ajustar al hardware real)
#define ENCODER_PPR 512.0f

// ------------------- Variables internas -------------------
static QueueHandle_t velocity_queue = NULL;

static float filtered_velocity = 0.0f;
const float alpha = 0.2f;  // 0 < alpha < 1

// ------------------- Tarea -------------------
static void velocity_task(void *parameter) {

    TickType_t last_wake_time = xTaskGetTickCount();
    int32_t last_ticks = 0;
    uint64_t last_exec_us = esp_timer_get_time();
    uint32_t max_jitter_us = 0;
    const uint32_t target_period_us = VELOCITY_TASK_PERIOD_MS * 1000U;
    static float jitter_mean = 0.0f;
    static float jitter_var = 0.0f;
    static uint32_t n = 0;

    for (;;) {

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(VELOCITY_TASK_PERIOD_MS)
        );

        uint64_t now_us = esp_timer_get_time();
        uint32_t real_period_us = (uint32_t)(now_us - last_exec_us);
        last_exec_us = now_us;

        float jitter_us = (int32_t)real_period_us - (int32_t)target_period_us;
        
        n++;
        float delta = jitter_us - jitter_mean;
        jitter_mean += delta / n;
        jitter_var += delta * (jitter_us - jitter_mean);

        float jitter_std = (n > 1) 
                   ? sqrt(jitter_var / (n - 1))
                   : 0.0f;

        int32_t current_ticks = encoder_get_ticks();
        int32_t delta_ticks   = current_ticks - last_ticks;
        last_ticks = current_ticks;

        float dt_s = real_period_us / 1000000.0f;
        float velocity_rps = (dt_s > 0.0f)
            ? (delta_ticks / ENCODER_PPR) / dt_s
            : 0.0f;

        filtered_velocity = alpha * velocity_rps + 
                (1.0f - alpha) * filtered_velocity;

        velocity_sample_t sample = {
            .velocity = filtered_velocity*60,
            .period_us = real_period_us, 
            .jitter_us = jitter_std,
            .timestamp_us = now_us
        };

        xQueueOverwrite(velocity_queue, &sample);
    }
}

// ------------------- Inicialización -------------------
uint8_t init_velocity_task(QueueHandle_t queue) {

    if (queue == NULL) return 0;
    velocity_queue = queue;

    BaseType_t result = xTaskCreatePinnedToCore(
        velocity_task,
        "VelocityTask",
        VELOCITY_TASK_STACK,
        NULL,
        VELOCITY_TASK_PRIORITY,
        NULL,
        0 // Core 0
    );

    return (result == pdPASS);
}