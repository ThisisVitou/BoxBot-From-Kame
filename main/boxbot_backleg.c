#include <stdio.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"

#include <math.h> // For sinf, M_PI

static char *TAG = "Logs";

// For Servo
#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2500
#define SERVO_MIN_DEGREE 0
#define SERVO_MAX_DEGREE 180

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000
#define SERVO_TIMEBASE_PERIOD 20000

#define SERVO_COUNT 8
const int SERVO_GPIO[SERVO_COUNT] = {32, 33, 25, 26, 27, 14, 12, 13};

#define TIMER_COUNT 2
#define OPERATOR_PER_TIMER 2
#define OPERATOR_COUNT (TIMER_COUNT * OPERATOR_PER_TIMER)

mcpwm_timer_handle_t timers[TIMER_COUNT] = {NULL};
mcpwm_oper_handle_t operators[OPERATOR_COUNT] = {NULL};
mcpwm_cmpr_handle_t comparators[SERVO_COUNT] = {NULL};
mcpwm_gen_handle_t generators[SERVO_COUNT] = {NULL};

static inline uint32_t angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

void servo_set_angle(int servo_index, int angle)
{
    if (servo_index < 0 || servo_index >= SERVO_COUNT)
        return;
    if (angle < SERVO_MIN_DEGREE)
        angle = SERVO_MIN_DEGREE;
    if (angle > SERVO_MAX_DEGREE)
        angle = SERVO_MAX_DEGREE;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[servo_index], angle_to_compare(angle)));
}

void servo_init()
{
    ESP_LOGI(TAG, "Create timers");
    for (int t = 0; t < TIMER_COUNT; t++)
    {
        mcpwm_timer_config_t timer_config = {
            .group_id = t, // timer0 in group 0, timer1 in group 1
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
            .period_ticks = SERVO_TIMEBASE_PERIOD,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        };
        ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timers[t]));
    }

    ESP_LOGI(TAG, "Create operators and connect to timers");
    for (int o = 0; o < OPERATOR_COUNT; o++)
    {
        int group_id = o / OPERATOR_PER_TIMER; // 0 for first 2 operators, 1 for next 2
        mcpwm_operator_config_t operator_config = {
            .group_id = group_id,
        };
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators[o]));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[o], timers[group_id]));
    }

    ESP_LOGI(TAG, "Create comparators and generators for each servo");
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        int op_idx = i / 2; // 2 servos per operator
        mcpwm_comparator_config_t comparator_config = {
            .flags.update_cmp_on_tez = true,
        };
        ESP_ERROR_CHECK(mcpwm_new_comparator(operators[op_idx], &comparator_config, &comparators[i]));

        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = SERVO_GPIO[i],
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(operators[op_idx], &generator_config, &generators[i]));

        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[i], angle_to_compare(90)));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generators[i],
                                                                  MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generators[i],
                                                                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i], MCPWM_GEN_ACTION_LOW)));
    }

    ESP_LOGI(TAG, "Enable and start timers");
    for (int t = 0; t < TIMER_COUNT; t++)
    {
        ESP_ERROR_CHECK(mcpwm_timer_enable(timers[t]));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(timers[t], MCPWM_TIMER_START_NO_STOP));
    }
}

//reach are how much degree from 90 degrees
// k is 0 for first servo, 1 for second servo
// d is for the direction of the servo movement
// for the primary link

// positive d mean servo will move to the right for << front left >> and back right will move the opposite
// positive d mean servo will move to the right for << front right >> and back left will move the opposite

// for the secondary link
// front left and back right will move up with << positive >> d
// front right and back left will move up with << negative >> d

void front_left(short reach, short k, short d)
{
    reach = reach * d;
    if (k == 0)
    {
        short x = 110;
        servo_set_angle(2, x + reach);
    }
    else if (k == 1)
    {
        short x = 90;
        servo_set_angle(3, x + reach);
    }
}
void front_right(short reach, short k, short d)
{
    reach = reach * d;
    if (k == 0)
    {
        short x = 80;
        servo_set_angle(7, x + reach);
    }
    else if (k == 1)
    {
        short x = 90;
        servo_set_angle(6, x + reach);
    }
}
void back_left(short reach, short k, short d)
{
    reach = reach * d;
    if (k == 0)
    {
        short x = 90;
        servo_set_angle(4, x + reach);
    }
    else if (k == 1)
    {
        short x = 90;
        servo_set_angle(5, x + reach);
    }
}
void back_right(short reach, short k, short d)
{
    reach = reach * d;
    if (k == 0)
    {
        short x = 90;
        servo_set_angle(0, x + reach);
    }
    else if (k == 1)
    {
        short x = 90;
        servo_set_angle(1, x + reach);
    }
}

void starting_sequence()
{
    ESP_LOGI(TAG, "Starting sequence");

    front_left(0, 0, -1); // Front Left
    front_right(0, 0, 1); // Front Right
    back_left(0, 0, 1);   // Back Left
    back_right(0, 0, -1); // Back Right

    front_left(0, 1, -1); // Front Left
    front_right(0, 1, 1); // Front Right
    back_left(0, 1, 1);   // Back Left
    back_right(0, 1, -1); // Back Right
}

int mapReach(uint8_t x, uint8_t value)
{
    return x - value;
}

int reverseMap(uint8_t value, uint8_t up, uint8_t down)
{
    // Maps value in [down, up] to [up, down]
    return up + down - value;
}

void smooth_walk_sequence()
{
    uint8_t up = 30;
    uint8_t down = 5;

    uint8_t reach;
    uint8_t init_reach = 35;

    for (uint8_t i = up; i >= down; i--)
    {
        // up 1
        front_left(reverseMap(i, up, down), 1, 1);
        back_right(reverseMap(i, up, down), 1, 1);

        // down 2
        front_right(i, 1, -1);
        back_left(i, 1, -1);

        vTaskDelay(pdMS_TO_TICKS(15));
    }

    for (reach = 0; reach <= init_reach; reach++)
    {
        // fowards 1
        front_left(mapReach(init_reach, reach), 0, -1);
        back_right(reach, 0, -1);

        // backward 2   
        front_right(reach, 0, 1);
        back_left(mapReach(init_reach, reach), 0, 1);

        vTaskDelay(pdMS_TO_TICKS(15));
    }

    for (int i = up; i >= down; i--)
    {
        // down 1
        front_left(i, 1, 1);
        back_right(i, 1, 1);

        // up 2
        front_right(reverseMap(i, up, down), 1, -1);
        back_left(reverseMap(i, up, down), 1, -1);
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    
    for (reach = 0; reach <= init_reach; reach++)
    {
        // backward 1
        front_left(reach, 0, -1);
        back_right(mapReach(init_reach, reach), 0, -1);

        // fowards 2
        front_right(mapReach(init_reach, reach), 0, 1);
        back_left(reach, 0, 1);

        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

void laying_sequence(int x)
{
    ESP_LOGI(TAG, "Laying sequence");

    front_left(x, 1, 1); // Front Left
    back_right(x, 1, 1); // Back Right

    front_right(x, 1, -1); // Front Right
    back_left(x, 1, -1);   // Back Left

    front_left(0, 0, -1); // Front Left
    back_right(0, 0, -1); // Back Right
    front_right(0, 0, 1); // Front Right
    back_left(0, 0, 1);   // Back Left
}

void app_main(void)
{
    servo_init();

    starting_sequence();
    ESP_LOGI(TAG, "Starting sequence | wait 2 seconds");
    vTaskDelay(pdMS_TO_TICKS(2000));

    laying_sequence(20);

    while (1)
    {
        smooth_walk_sequence();
        ESP_LOGI(TAG, "Walking sequence completed");
    }
}