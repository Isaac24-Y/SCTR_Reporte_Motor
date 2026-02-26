#ifndef PID_HANDLER_H
#define PID_HANDLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

#define PWM_MAX 255
#define PWM_MIN 0

// --------- Estructuras ---------
typedef struct {
    float kp;
    float ki;
    float kd;
} pid_params_t;

typedef struct {
    float val_0;
    float val_1;
    float val_2;
} sensor_values;

typedef struct {
    float control;
    float error;
    float pv;
    float sp;
    uint32_t velocity_period_us;
    float velocity_jitter_us;
    uint32_t pid_period_us;
    float pid_jitter_us;
    uint32_t pid_latency_us;
    uint64_t sample_timestamp_us;
    uint64_t control_timestamp_us;
} pid_output_t;

typedef struct {
    float sp;
    float pv;
    float op;
    float error;
    uint32_t velocity_period_us;
    float   velocity_jitter_us;
    uint32_t  pid_period_us;
    float   pid_jitter_us;
    uint32_t pid_latency_us;
    uint64_t control_to_pwm_latency_us;
    uint64_t sensor_to_pwm_latency_us;
} telemetry_t;

// --------- API ---------
uint8_t init_pid_task(
    QueueHandle_t velocity_q,
    QueueHandle_t setpoint_q,
    QueueHandle_t output_q
);

void pid_update_params(pid_params_t params);

#endif