#include "ws2812.h"
#include "targets.h"
#include "cmsis_os.h"
#include "semphr.h"

// External handles
extern TIM_HandleTypeDef WS2812_TIMER;
extern DMA_HandleTypeDef WS2812_DMA;
extern SemaphoreHandle_t ws2812_dma_semaphore;

volatile uint8_t ws2812_data_sent_flag = 0;

// Double buffers
static uint32_t pwm_buffer_a[WS2812_BUFFER_SIZE];
static uint32_t pwm_buffer_b[WS2812_BUFFER_SIZE];
static uint32_t *active_buffer = pwm_buffer_a;
static uint32_t *prepare_buffer = pwm_buffer_b;

static uint8_t led_data[LED_COUNT][3];
static float global_brightness = 1.0f;

#define WS2812_LATCH_DELAY_MS 1

static inline void swap_buffers(void) {
    uint32_t *temp = active_buffer;
    active_buffer = prepare_buffer;
    prepare_buffer = temp;
}

void WS2812_Init(TIM_HandleTypeDef *htim) {
    for (uint32_t i = 0; i < WS2812_BUFFER_SIZE; i++) {
        pwm_buffer_a[i] = DUTY_RESET;
        pwm_buffer_b[i] = DUTY_RESET;
    }
}

void WS2812_SetGlobalBrightness(float brightness) {
    if (brightness < 0.0f) brightness = 0.0f;
    if (brightness > 1.0f) brightness = 1.0f;
    global_brightness = brightness;
}

void main_led(uint32_t led_index, uint8_t red, uint8_t green, uint8_t blue, float brightness) {
    if (led_index < LED_COUNT) {
        if (brightness < 0.0f) brightness = 0.0f;
        if (brightness > 1.0f) brightness = 1.0f;

        float total_brightness = brightness * global_brightness;
        led_data[led_index][0] = (uint8_t)(green * total_brightness);
        led_data[led_index][1] = (uint8_t)(red   * total_brightness);
        led_data[led_index][2] = (uint8_t)(blue  * total_brightness);
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

/**
 * Non-blocking WS2812 update
 */
void main_led_update(void) {
    encode_leds_to_pwm(prepare_buffer);
    swap_buffers();

    taskENTER_CRITICAL();
    HAL_TIM_PWM_Stop_DMA(&WS2812_TIMER, WS2812_TIMER_CHANNEL);
    HAL_TIM_Base_Stop(&WS2812_TIMER);
    ws2812_data_sent_flag = 0;

    HAL_TIM_Base_Start(&WS2812_TIMER);
    HAL_TIM_PWM_Start_DMA(&WS2812_TIMER, WS2812_TIMER_CHANNEL,
                          active_buffer, WS2812_BUFFER_SIZE);
    taskEXIT_CRITICAL();

    // Return immediately — no blocking wait here
}

/**
 * Called from DMA complete ISR
 */
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

/**
 * Optional: task to handle post-DMA latch delay
 */
void WS2812_Task(void *argument) {
    for (;;) {
        // Wait for DMA complete signal
        if (xSemaphoreTake(ws2812_dma_semaphore, portMAX_DELAY) == pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(WS2812_LATCH_DELAY_MS));
            // At this point, LEDs are fully updated
        }
    }
}
