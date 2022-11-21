#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_log.h"

#include "string.h"
#include "stdio.h"

#define GPIO_FSK_INPUT GPIO_NUM_1
#define ESP_INTR_FLAG_DEFAULT (0)
#define EVENT_QUEUE_SIZE (32)
#define FSK_DEPTH (250)

#define TIME_ONE_MAX (256 + 32)
#define TIME_ONE_MIN (256 - 32)
#define TIME_ZERO_MAX (512 + 64)
#define TIME_ZERO_MIN (512 - 64)

#define JUDGE_ONE(x) ((x >= TIME_ONE_MIN) && (x <= TIME_ONE_MAX))
#define JUDGE_ZERO(x) ((x >= TIME_ZERO_MIN) && (x <= TIME_ZERO_MAX))

typedef struct
{
    uint8_t start;
    uint8_t half_one;
    uint8_t data_8_bit;
    uint8_t bit_ii;
    uint8_t mode; // 1 - ack / 2 - data
    uint8_t data_start;
    uint8_t data_xor;
    uint8_t data_stop;
} fsk_data_t;

static const char *TAG = "FSK";

static xQueueHandle fsk_event_queue = NULL;
static uint8_t fsk_buf[16] = {0};
static uint8_t fsk_buf_ii = 0;
static fsk_data_t fsk_data;

static void IRAM_ATTR fsk_pwm_isr_handler(void *arg);
static int fsk_dmod_handler(uint32_t period);
static inline void fsk_dmod_reset(void);

void fsk_init(void)
{
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = (1ULL << GPIO_FSK_INPUT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE};
    ESP_ERROR_CHECK(gpio_config(&gpio_cfg));

    fsk_event_queue = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(uint32_t));

    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_FSK_INPUT, fsk_pwm_isr_handler, NULL));
}

static void IRAM_ATTR fsk_pwm_isr_handler(void *arg)
{
    static uint32_t counter = 0;
    static uint32_t timing_start = 0;
    static uint32_t timing_end = 0;
    static uint32_t period_0, period_1, period_2 = 0;
    static uint8_t cycle_index = 0;
    static uint8_t push_flag = 0;
    static uint32_t up_flag = 0;
    static uint32_t down_flag = 0;
    static uint32_t both_flag = 0;

    switch (cycle_index)
    {
    case 7:
        timing_end = cpu_hal_get_cycle_count();
        break;
    case 8:
        period_0 = period_1;
        period_1 = period_2;
        break;
    case 9:
        period_2 = timing_end - timing_start;
        timing_start = timing_end;
        break;
    case 10:
        counter += 16;
        up_flag = up_flag >> 1;
        down_flag = down_flag >> 1;
        both_flag = both_flag>>1;
        break;
    case 12:
        if ((up_flag == 0 && both_flag ==0) && (period_2 > period_0 + FSK_DEPTH))
        {
            up_flag = 0xff;
            both_flag = 0x0f;
            push_flag = 1;
        }
        break;
    case 13:
        if ((down_flag == 0 && both_flag ==0) && (period_2 < period_0 - FSK_DEPTH))
        {
            down_flag = 0xff;
            both_flag = 0x0f;
            push_flag = 1;
        }
        break;
    case 14:
        if (push_flag || counter == 4096)
        {
            xQueueSendFromISR(fsk_event_queue, &counter, NULL);
            counter = 0;
        }
        break;
    case 16:
        push_flag = 0;
        cycle_index = 0;
        break;
    }
    cycle_index++;
}

void fsk_task(void *param)
{
    static uint32_t period = 0;
    int ret;
    while (true)
    {
        if (xQueueReceive(fsk_event_queue, &period, portMAX_DELAY))
        {
            ret = fsk_dmod_handler(period);
            if (ret == 1)
            {
                ESP_LOG_BUFFER_HEX(TAG, fsk_buf, fsk_buf_ii);
                fsk_dmod_reset();
            }
            else if (ret == -1)
            {
                if (fsk_data.bit_ii != 0)
                {
                    ESP_LOGI(TAG, "data_8_bit->%02x, half_one->%u, bit_ii->%u, period->%u",
                             fsk_data.data_8_bit, fsk_data.half_one, fsk_data.bit_ii, period);
                }
                fsk_dmod_reset();
            }
        }
    }
    vTaskDelete(NULL);
}

static int fsk_dmod_handler(uint32_t period)
{
    if (fsk_data.start == 0)
    {
        if (JUDGE_ONE(period) && (fsk_data.half_one == 0))
        {
            fsk_data.half_one = 1;
        }
        else if (JUDGE_ONE(period) && (fsk_data.half_one == 1))
        {
            fsk_data.half_one = 0;
            fsk_data.data_8_bit |= (0x01 << fsk_data.bit_ii);
            fsk_data.bit_ii++;
        }
        else if (JUDGE_ZERO(period) && (fsk_data.half_one == 0))
        {
            fsk_data.bit_ii++;
        }
        else
        {
            return -1;
        }
        if (fsk_data.bit_ii == 4)
        {
            if (fsk_data.data_8_bit == 0x0f)
            {
                fsk_data.mode = 1; // ack mode
                fsk_data.start = 1;
            }
            else
            {
                fsk_data.mode = 2; // data mode
                fsk_data.start = 1;
            }
        }
    }
    else
    {
        if (fsk_data.mode == 1) // ack mode
        {
            if (JUDGE_ONE(period) && (fsk_data.half_one == 0))
            {
                fsk_data.half_one = 1;
            }
            else if (JUDGE_ONE(period) && (fsk_data.half_one == 1))
            {
                fsk_data.half_one = 0;
                fsk_data.data_8_bit |= (0x01 << fsk_data.bit_ii);
                fsk_data.bit_ii++;
            }
            else if (JUDGE_ZERO(period) && (fsk_data.half_one == 0))
            {
                fsk_data.bit_ii++;
            }
            else
            {
                if ((fsk_data.bit_ii == 7) && ((period == 0) || (period >= TIME_ZERO_MAX)))
                {
                    if (fsk_data.half_one == 1)
                    {
                        fsk_data.half_one = 0;
                        fsk_data.data_8_bit |= 0x01 << fsk_data.bit_ii;
                        fsk_data.bit_ii++;
                    }
                    else
                    {
                        fsk_data.bit_ii++;
                    }
                }
                else
                {
                    return -1;
                }
            }
            if (fsk_data.bit_ii == 8)
            {
                fsk_buf[fsk_buf_ii] = fsk_data.data_8_bit;
                fsk_buf_ii++;
                return 1;
            }
        }
        else if (fsk_data.mode == 2)
        {
            return -1;
        }
    }
    return 0;
}

static inline void fsk_dmod_reset(void)
{
    memset(&fsk_data, 0, sizeof(fsk_data_t));
    fsk_buf_ii = 0;
}
