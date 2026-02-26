
#include "PIDHandler.h"
#include "VelocityHandler.h"
#include <Arduino.h>
#include "esp_timer.h"

// ---------------- Configuración ----------------
#define PID_TASK_PERIOD_MS   10
#define PID_TASK_PRIORITY    2
#define PID_TASK_STACK       3072

// ---------------- Queues ----------------
static QueueHandle_t velocity_queue  = NULL;
static QueueHandle_t setpoint_queue  = NULL;
static QueueHandle_t output_queue    = NULL;

// ---------------- Parámetros PID ----------------
static pid_params_t pid = {1.0f, 0.0001f, 0.0001f};
static sensor_values sensor_buffer = {0.0f, 0.0f, 0.0f};

// ---------------- Variables internas ----------------
static float integral = 0.0f;
static float prev_error = 0.0f;

// ---------------- Tarea PID ----------------
static void pid_task(void *parameter) {

    TickType_t last_wake_time = xTaskGetTickCount();
    float sp = 0.0f;
    velocity_sample_t sample = {0};
    uint64_t last_exec_us = esp_timer_get_time();
    uint32_t max_jitter_us = 0;
    const uint32_t target_period_us = PID_TASK_PERIOD_MS * 1000U;


    static float jitter_mean = 0.0f;
    static float jitter_var = 0.0f;
    static uint32_t n = 0;

    for (;;) {

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(PID_TASK_PERIOD_MS)
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

        // Leer velocidad y setpoint (no bloqueante, mantiene último valor)
        xQueueReceive(velocity_queue, &sample, 0);
        xQueueReceive(setpoint_queue, &sp, 0);

        // PID 
        float pv = sample.velocity;
        float error = sp - pv;
        float dt = PID_TASK_PERIOD_MS / 1000.0f;

        float proportional = error*pid.kp;

        //sensor_buffer.val_2 = sensor_buffer.val_1;
        //sensor_buffer.val_1 = sensor_buffer.val_0;
        //sensor_buffer.val_0 = pv;
        
        float derivative = pid.kp * pid.kd *(error - prev_error) / dt;
        integral += pid.kp * (dt / pid.ki) * error;
        prev_error = error;

        float output = proportional + derivative + integral;
        if (output > PWM_MAX) output = PWM_MAX;
        if (output < PWM_MIN) output = PWM_MIN;

    /*     
        integral += pid.ki * error * dt;

        float derivative = pid.kd * (error - prev_error) / dt;
        prev_error = error;

        float output = proportional + integral + derivative;
        //output = proportional;
    */

        // Valores del PID
        pid_output_t out = {
            .control = output,
            .error   = error,
            .pv      = pv,
            .sp      = sp,
            .velocity_period_us = sample.period_us,
            .velocity_jitter_us = sample.jitter_us,
            .pid_period_us = real_period_us,
            .pid_jitter_us = jitter_std,
            .pid_latency_us = (uint32_t)(now_us - sample.timestamp_us),
            .sample_timestamp_us = sample.timestamp_us,
            .control_timestamp_us = now_us
        };

        xQueueOverwrite(output_queue, &out);
    }
}

// ---------------- Inicialización ----------------
uint8_t init_pid_task(
    QueueHandle_t velocity_q,
    QueueHandle_t setpoint_q,
    QueueHandle_t output_q
) {
    if (!velocity_q || !setpoint_q || !output_q)
        return 0;

    velocity_queue = velocity_q;
    setpoint_queue = setpoint_q;
    output_queue   = output_q;

    BaseType_t result = xTaskCreatePinnedToCore(
        pid_task,
        "PIDTask",
        PID_TASK_STACK,
        NULL,
        PID_TASK_PRIORITY,
        NULL,
        1 // Core 1
    );

    return (result == pdPASS);
}

// ---------------- API ----------------
void pid_update_params(pid_params_t params) {
    pid = params;
}

