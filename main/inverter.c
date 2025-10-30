#include "stdio.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "MAIN"

#define PWM_A_PIN GPIO_NUM_2
#define PWM_B_PIN GPIO_NUM_4

#define MCPWM_RESOLUTION (1e6) // 1us por tick
#define MCPWM_PERIOD_US (10)
#define MCPWM_TIMER_COUNT MCPWM_TIMER_COUNT_MODE_UP_DOWN
#define MCPWM_DUTY MCPWM_PERIOD_US / 2
#define MCPWM_DEPHASE MCPWM_PERIOD_US / 2
#define MCPWM_DEAD_TICKS (2)

#define TASK_NAME "control task"
#define TASK_STACK_SIZE 1024 * 1
#define TASK_PRIOR 2

static mcpwm_timer_handle_t timer;
static mcpwm_oper_handle_t operator;
static mcpwm_cmpr_handle_t comparator;
static mcpwm_gen_handle_t generator_a, generator_b;
static mcpwm_sync_handle_t timer_sync;

esp_err_t mcpwm_init(void)
{
    mcpwm_timer_config_t timer_config = {
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .group_id = 0,
        .resolution_hz = MCPWM_RESOLUTION,
        .period_ticks = MCPWM_PERIOD_US,
        .count_mode = MCPWM_TIMER_COUNT,
    };

    esp_err_t ret = mcpwm_new_timer(&timer_config, &timer);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create new timer A (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };

    ret = mcpwm_new_operator(&operator_config, &operator);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create new operator A (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_operator_connect_timer(operator, timer);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to connect operator with timer A (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };

    ret = mcpwm_new_comparator(operator, &comparator_config, &comparator);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create new comparator A (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_comparator_set_compare_value(comparator, (MCPWM_DUTY));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set value comparator (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    mcpwm_generator_config_t generator_config_a = {
        .gen_gpio_num = PWM_A_PIN,
    };

    ret = mcpwm_new_generator(operator, &generator_config_a, &generator_a);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create new generator A (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    mcpwm_generator_config_t generator_config_b = {
        .gen_gpio_num = PWM_B_PIN,
    };

    ret = mcpwm_new_generator(operator, &generator_config_b, &generator_b);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create new generator B (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_generator_set_action_on_timer_event(generator_a,
                                                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = mcpwm_generator_set_actions_on_compare_event(generator_a,
                                                       MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_HIGH),
                                                       MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, comparator, MCPWM_GEN_ACTION_LOW),
                                                       MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    if (ret != ESP_OK)
    {
        return ret;
    }

    mcpwm_dead_time_config_t dt_config_a = {
        .posedge_delay_ticks = MCPWM_DEAD_TICKS,
        .negedge_delay_ticks = 0,
    };

    ret = mcpwm_generator_set_dead_time(generator_a, generator_a, &dt_config_a);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set dead time for generator A (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    mcpwm_dead_time_config_t dt_config_b = {
        .posedge_delay_ticks = 0,
        .negedge_delay_ticks = MCPWM_DEAD_TICKS,
        .flags.invert_output = true,
    };

    ret = mcpwm_generator_set_dead_time(generator_a ,generator_b , &dt_config_b);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set dead time for generator B (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_timer_enable(timer);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable timer A (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start timer A (E:%s)", esp_err_to_name(ret));
    }
    return ret;
}

void app_main(void)
{
    esp_err_t ret = mcpwm_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to init timer");
        return;
    }
}