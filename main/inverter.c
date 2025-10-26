#include "stdio.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG                     "MAIN"

#define PWM_A_PIN               GPIO_NUM_2
#define PWM_B_PIN               GPIO_NUM_4

#define MCPWM_RESOLUTION        (1e6) // 1us por tick
#define MCPWM_PERIOD_US         (10)
#define MCPWM_TIMER_COUNT       MCPWM_TIMER_COUNT_MODE_UP
#define MCPWM_DUTY              MCPWM_PERIOD_US / 2
#define MCPWM_DEPHASE           MCPWM_PERIOD_US / 2

#define TASK_NAME               "control task"
#define TASK_STACK_SIZE         1024 * 1
#define TASK_PRIOR              2

static mcpwm_timer_handle_t timer_a, timer_b;
static mcpwm_oper_handle_t operator_a, operator_b;
static mcpwm_cmpr_handle_t comparator_a, comparator_b;
static mcpwm_gen_handle_t generator_a, generator_b;
static mcpwm_sync_handle_t timer_sync;


esp_err_t mcpwm_timer_init(void)
{
    mcpwm_timer_config_t timer_config = {
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .group_id = 0,
        .resolution_hz = MCPWM_RESOLUTION,
        .period_ticks = MCPWM_PERIOD_US,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };

    esp_err_t ret = mcpwm_new_timer(&timer_config, &timer_a);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create new timer A (E:%s)", esp_err_to_name(ret));
        return ret;
    }
    ret = mcpwm_new_timer(&timer_config, &timer_b);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create new timer B (E:%s)", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t mcpwm_operator_init(void)
{
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };

    esp_err_t ret = mcpwm_new_operator(&operator_config, &operator_a);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create new operator A (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_new_operator(&operator_config, &operator_b);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create new operator B (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_operator_connect_timer(operator_a, timer_a);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to connect operator with timer A (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_operator_connect_timer(operator_b, timer_b);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to connect operator with timer B (E:%s)", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t mcpwm_comparator_init(void)
{
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };

    esp_err_t ret = mcpwm_new_comparator(operator_a, &comparator_config, &comparator_a);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create new comparator A (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_new_comparator(operator_b, &comparator_config, &comparator_b);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create new comparator B (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_comparator_set_compare_value(comparator_a, (MCPWM_DUTY));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set value comparator (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_comparator_set_compare_value(comparator_b, (MCPWM_DUTY));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set value comparator (E:%s)", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t mcpwm_generator_init(void)
{
    mcpwm_generator_config_t generator_config_a = {
        .gen_gpio_num = PWM_A_PIN,
    };

    esp_err_t ret = mcpwm_new_generator(operator_a, &generator_config_a, &generator_a);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create new generator A (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    mcpwm_generator_config_t generator_config_b = {
        .gen_gpio_num = PWM_B_PIN,
    };

    ret = mcpwm_new_generator(operator_b, &generator_config_b, &generator_b);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create new generator B (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_generator_set_action_on_timer_event(
        generator_a,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set generator action on empty event A (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_generator_set_action_on_compare_event(
        generator_a,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       comparator_a,
                                       MCPWM_GEN_ACTION_LOW));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set generator action on compare event A (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_generator_set_action_on_timer_event(
        generator_b,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set generator action on empty event B (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_generator_set_action_on_compare_event(
        generator_b,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       comparator_b,
                                       MCPWM_GEN_ACTION_LOW));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set generator action on compare event B (E:%s)", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t mcpwm_timer_start(void)
{

    esp_err_t ret = mcpwm_timer_enable(timer_a);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable timer A (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_timer_start_stop(timer_a, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start timer A (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_timer_enable(timer_b);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable timer A (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_timer_start_stop(timer_b, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start timer B (E:%s)", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t sync_pwm(void)
{
    // sincronia para defasagem
    mcpwm_timer_sync_src_config_t timer_sync_config = {
        .timer_event = MCPWM_TIMER_EVENT_EMPTY,
    };
    esp_err_t ret = mcpwm_new_timer_sync_src(timer_a, &timer_sync_config, &timer_sync);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to sync timer A (E:%s)", esp_err_to_name(ret));
        return ret;
    }

    mcpwm_timer_sync_phase_config_t sync_phase_config = {
        .count_value = MCPWM_DEPHASE,
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .sync_src = timer_sync,
    };
    ret = mcpwm_timer_set_phase_on_sync(timer_b, &sync_phase_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set phase on sync (E:%s)", esp_err_to_name(ret));
    }
    return ret;
}

void app_main(void)
{
    esp_err_t ret = mcpwm_timer_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to init timer");
        return;
    }

    ret = mcpwm_operator_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to init operators");
        return;
    }

    ret = mcpwm_comparator_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to init comparators");
        return;
    }

    ret = mcpwm_generator_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to init generators");
        return;
    }

    ret = mcpwm_timer_start();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start timers");
        return;
    }

    ret = sync_pwm();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to sync pwm signal");
        return;
    }

    // BaseType_t xRet = xTaskCreate()
}