#include "Button.h"

#define BUTTON_GPIO_PORT                GPIOC
#define BUTTON_GPIO_PIN                 GPIO_Pin_2
#define BUTTON_DEBOUNCE_TICKS            3U      /* 30 ms */
#define BUTTON_DOUBLE_CLICK_TICKS       50U      /* 500 ms */

static volatile uint8_t s_button_events = BUTTON_EVENT_NONE;
static uint8_t s_initialized = 0U;
static uint8_t s_raw_pressed = 0U;
static uint8_t s_stable_pressed = 0U;
static uint8_t s_debounce_ticks = 0U;
static uint8_t s_click_pending = 0U;
static uint8_t s_click_window_ticks = 0U;

void Button_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = BUTTON_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BUTTON_GPIO_PORT, &GPIO_InitStructure);

    s_raw_pressed = (uint8_t)(GPIO_ReadInputDataBit(BUTTON_GPIO_PORT,
                                                    BUTTON_GPIO_PIN) == Bit_RESET);
    s_stable_pressed = s_raw_pressed;
    s_debounce_ticks = 0U;
    s_click_pending = 0U;
    s_click_window_ticks = 0U;
    s_button_events = BUTTON_EVENT_NONE;
    s_initialized = 1U;
}

void Button_Tick10ms(void)
{
    uint8_t raw_pressed;
    uint8_t press_event = 0U;

    if (!s_initialized)
    {
        return;
    }

    raw_pressed = (uint8_t)(GPIO_ReadInputDataBit(BUTTON_GPIO_PORT,
                                                  BUTTON_GPIO_PIN) == Bit_RESET);

    if (raw_pressed == s_raw_pressed)
    {
        if (s_debounce_ticks < BUTTON_DEBOUNCE_TICKS)
        {
            s_debounce_ticks++;
        }
    }
    else
    {
        s_raw_pressed = raw_pressed;
        s_debounce_ticks = 1U;
    }

    if (s_debounce_ticks >= BUTTON_DEBOUNCE_TICKS &&
        s_stable_pressed != s_raw_pressed)
    {
        s_stable_pressed = s_raw_pressed;
        if (s_stable_pressed)
        {
            press_event = 1U;
        }
    }

    if (press_event)
    {
        if (s_click_pending &&
            s_click_window_ticks < BUTTON_DOUBLE_CLICK_TICKS)
        {
            s_click_pending = 0U;
            s_click_window_ticks = 0U;
            s_button_events |= BUTTON_EVENT_DOUBLE;
        }
        else
        {
            s_click_pending = 1U;
            s_click_window_ticks = 0U;
        }
    }
    else if (s_click_pending)
    {
        if (s_click_window_ticks < BUTTON_DOUBLE_CLICK_TICKS)
        {
            s_click_window_ticks++;
        }

        if (s_click_window_ticks >= BUTTON_DOUBLE_CLICK_TICKS)
        {
            s_click_pending = 0U;
            s_click_window_ticks = 0U;
            s_button_events |= BUTTON_EVENT_SINGLE;
        }
    }
}

uint8_t Button_GetEvents(void)
{
    uint8_t events;

    __disable_irq();
    events = s_button_events;
    s_button_events = BUTTON_EVENT_NONE;
    __enable_irq();

    return events;
}

uint8_t Button_PeekEvents(void)
{
    return s_button_events;
}

void Button_ClearEvents(void)
{
    __disable_irq();
    s_button_events = BUTTON_EVENT_NONE;
    __enable_irq();
}
