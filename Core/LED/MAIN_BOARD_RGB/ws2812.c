#include "ws2812.h"
#include "targets.h"
#include "cmsis_os.h"
#include "semphr.h"

// External handles
extern TIM_HandleTypeDef WS2812_TIMER;
extern DMA_HandleTypeDef WS2812_DMA;
extern SemaphoreHandle_t ws2812_dma_semaphore;
extern TaskHandle_t ws2812_task_handle;

volatile uint8_t ws2812_data_sent_flag = 0;

// Double buffers
static uint32_t pwm_buffer_a[WS2812_BUFFER_SIZE];
static uint32_t pwm_buffer_b[WS2812_BUFFER_SIZE];
static uint32_t *active_buffer = pwm_buffer_a;
static uint32_t *prepare_buffer = pwm_buffer_b;

static uint8_t led_data[LED_COUNT][3];
static volatile float global_brightness = 1.0f; // Made volatile to avoid compiler optimizations

#define WS2812_LATCH_DELAY_MS 1

static inline void swap_buffers(void) {
    uint32_t *temp = active_buffer;
    active_buffer = prepare_buffer;
    prepare_buffer = temp;
}

void WS2812_Init(TIM_HandleTypeDef *htim) {
    // Initialize buffers
    for (uint32_t i = 0; i < WS2812_BUFFER_SIZE; i++) {
        pwm_buffer_a[i] = DUTY_RESET;
        pwm_buffer_b[i] = DUTY_RESET;
    }
}

void WS2812_SetGlobalBrightness(float brightness) {
    if (brightness < 0.0f) brightness = 0.0f;
    if (brightness > 1.0f) brightness = 1.0f;
    global_brightness = brightness; // No critical section needed (single write)
}

void main_led(uint32_t led_index, uint8_t red, uint8_t green, uint8_t blue, float brightness) {
    if (led_index < LED_COUNT) {
        if (brightness < 0.0f) brightness = 0.0f;
        if (brightness > 1.0f) brightness = 1.0f;

        // Avoid critical section by reading global_brightness once
        float total_brightness = brightness * global_brightness;

        // Update led_data atomically (single write per element)
        led_data[led_index][0] = (uint8_t)(green * total_brightness);
        led_data[led_index][1] = (uint8_t)(red * total_brightness);
        led_data[led_index][2] = (uint8_t)(blue * total_brightness);
    }
}

static void encode_leds_to_pwm(uint32_t *buffer) {
    uint32_t buffer_index = 0;
    for (uint32_t i = 0; i < WS2812_BUFFER_SIZE; i++) {
        buffer[i] = DUTY_RESET;
    }
    for (uint32_t led = 0; led < LED_COUNT; led++) {
        for (uint32_t color = 0; color < 3; color++) {
            for (int8_t bit = 7; bit >= 0; bit--) {
                buffer[buffer_index++] =
                    (led_data[led][color] & (1 << bit)) ? DUTY_1 : DUTY_0;
            }
        }
    }
}

void main_led_update(void) {
    // Check if DMA is busy to avoid conflicts
    if (HAL_DMA_GetState(&WS2812_DMA) == HAL_DMA_STATE_BUSY) {
        return; // Skip update if DMA is still active
    }

    // Prepare the buffer
    encode_leds_to_pwm(prepare_buffer);
    swap_buffers();

    // Stop previous DMA and timer
    if (HAL_TIM_PWM_Stop_DMA(&WS2812_TIMER, WS2812_TIMER_CHANNEL) != HAL_OK ||
        HAL_TIM_Base_Stop(&WS2812_TIMER) != HAL_OK) {
        // Handle error (e.g., log)
        return;
    }

    ws2812_data_sent_flag = 0;

    // Start new DMA transfer
    if (HAL_TIM_Base_Start(&WS2812_TIMER) != HAL_OK ||
        HAL_TIM_PWM_Start_DMA(&WS2812_TIMER, WS2812_TIMER_CHANNEL,
                              active_buffer, WS2812_BUFFER_SIZE) != HAL_OK) {
        // Handle error
        return;
    }

    // Notify WS2812_Task
    if (ws2812_task_handle != NULL) {
        xTaskNotifyGive(ws2812_task_handle);
    }
}

void main_led_PWM_Callback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == WS2812_TIMER.Instance) {
        HAL_TIM_PWM_Stop_DMA(&WS2812_TIMER, WS2812_TIMER_CHANNEL);
        HAL_TIM_Base_Stop(&WS2812_TIMER);

        ws2812_data_sent_flag = 1;

        if (ws2812_dma_semaphore != NULL) {
            BaseType_t hpw = pdFALSE;
            xSemaphoreGiveFromISR(ws2812_dma_semaphore, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}

void WS2812_Task(void *argument) {
    for (;;) {
        // LED control sequence
        main_led(0, 0, 0, 0, 1.0); // Black
        main_led_update();
        // Wait for notification and DMA completion
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        if (xSemaphoreTake(ws2812_dma_semaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(WS2812_LATCH_DELAY_MS)); // Latch delay
            vTaskDelay(pdMS_TO_TICKS(500 - WS2812_LATCH_DELAY_MS)); // Remaining delay
        } else {
            // Handle timeout (e.g., log)
        }

        main_led(0, 255, 0, 0, 1.0); // Red
        main_led_update();
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        if (xSemaphoreTake(ws2812_dma_semaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(WS2812_LATCH_DELAY_MS)); // Latch delay
            vTaskDelay(pdMS_TO_TICKS(30 - WS2812_LATCH_DELAY_MS)); // Remaining delay
        } else {
            // Handle timeout
        }
    }
}
