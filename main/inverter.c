#include "stdio.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "MAIN"

#define PWM_A_PIN                       GPIO_NUM_2
#define PWM_B_PIN                       GPIO_NUM_4

#define MCPWM_RESOLUTION                (1e7) // 100ns por tick
#define MCPWM_PERIOD_TICKS              (100) // 10us
#define MCPWM_TIMER_COUNT               MCPWM_TIMER_COUNT_MODE_UP
#define MCPWM_DUTY_TICKS                MCPWM_PERIOD_TICKS / 2
#define MCPWM_DEAD_TICKS                (10) // 1us

#define TASK_NAME                       "control task"
#define TASK_STACK_SIZE                 1024 * 1
#define TASK_PRIOR                      2

static mcpwm_timer_handle_t timer;
static mcpwm_oper_handle_t operator_a, operator_b;
static mcpwm_cmpr_handle_t comparator_a_high, comparator_a_low; 
static mcpwm_cmpr_handle_t comparator_b_high, comparator_b_low;
static mcpwm_gen_handle_t generator_a, generator_b;

esp_err_t mcpwm_set_duty(uint8_t percent)  // Esperado valores de 0 - 100
{
    if(percent > 100)
        percent = 100;
    
    uint32_t ticks =  ((MCPWM_DUTY_TICKS - MCPWM_DEAD_TICKS) * (percent / 100.0f))+ MCPWM_DEAD_TICKS;

    ESP_LOGI(TAG, "Set ticks: %d + %d + %ld\n\n",MCPWM_DUTY_TICKS, MCPWM_DEAD_TICKS, ticks);
    esp_err_t ret = mcpwm_comparator_set_compare_value(comparator_a_low, ticks);
    if(ret != ESP_OK)
    {
        return ret;
    }

    // Caso o valor do comparador seja igual ao periodo, não desliga o sinal B
    if((MCPWM_DUTY_TICKS + ticks) >= MCPWM_PERIOD_TICKS) 
        ticks -= 1;
    ret = mcpwm_comparator_set_compare_value(comparator_b_low, MCPWM_DUTY_TICKS + ticks);
    return ret;
}

esp_err_t mcpwm_init(void)
{
    // Configuração e criação do handle do temporizador
    mcpwm_timer_config_t timer_config = {
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .group_id = 0,
        .resolution_hz = MCPWM_RESOLUTION,
        .period_ticks = MCPWM_PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT,
    };

    esp_err_t ret = mcpwm_new_timer(&timer_config, &timer);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Configuração, criação e conexão dos operadores com o temporizador
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };

    ret = mcpwm_new_operator(&operator_config, &operator_a);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = mcpwm_operator_connect_timer(operator_a, timer);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = mcpwm_new_operator(&operator_config, &operator_b);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = mcpwm_operator_connect_timer(operator_b, timer);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Configuração e criação dos comparadores
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };

    ret = mcpwm_new_comparator(operator_a, &comparator_config, &comparator_a_high); // Aplica o dead time
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = mcpwm_new_comparator(operator_a, &comparator_config, &comparator_a_low);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = mcpwm_new_comparator(operator_b, &comparator_config, &comparator_b_high);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Resposável por fazer o sinal B seja 180° defasado e tenha o dead time
    ret = mcpwm_comparator_set_compare_value(comparator_b_high, (MCPWM_DUTY_TICKS + MCPWM_DEAD_TICKS));
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = mcpwm_new_comparator(operator_b, &comparator_config, &comparator_b_low);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Controla o duty sinal B
    ret = mcpwm_comparator_set_compare_value(comparator_b_low, (MCPWM_PERIOD_TICKS - 1));
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Configuração e criação dos geradores
    mcpwm_generator_config_t generator_config_a = {
        .gen_gpio_num = PWM_A_PIN,
    };

    ret = mcpwm_new_generator(operator_a, &generator_config_a, &generator_a);
    if (ret != ESP_OK)
    {
        return ret;
    }

    mcpwm_generator_config_t generator_config_b = {
        .gen_gpio_num = PWM_B_PIN,
    };

    ret = mcpwm_new_generator(operator_b, &generator_config_b, &generator_b);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = mcpwm_generator_set_actions_on_compare_event(generator_a,
                                                        // Quando o comparator A HIGH atingir o valor, seta o sinal A para 1
                                                        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_a_high, MCPWM_GEN_ACTION_HIGH),
                                                        // Quando o comparator A LOW atingir o valor, seta o sinal A para 0
                                                        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_a_low, MCPWM_GEN_ACTION_LOW),
                                                        MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = mcpwm_generator_set_actions_on_compare_event(generator_b,
                                                        // Quando o comparator B HIGH atingir o valor, seta o sinal B para 1
                                                        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_b_high, MCPWM_GEN_ACTION_HIGH),
                                                        // Quando o comparator B LOW atingir o valor, seta o sinal B para 0
                                                        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_b_low, MCPWM_GEN_ACTION_LOW),
                                                        MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    if (ret != ESP_OK)
    {
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