#include "BluetoothControl.h"
#include "ArmKinematics.h"
#include "Motor.h"
#include "MPU6050.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BLUETOOTH_RX_BUFFER_SIZE             64U
#define BLUETOOTH_JOYSTICK_LIMIT            100
#define BLUETOOTH_JOYSTICK_DEADZONE           8
#define BLUETOOTH_MAX_DRIVE_PERCENT          50U
#define BLUETOOTH_PWM_PERIOD_COUNTS         100U
#define BLUETOOTH_COMMAND_TIMEOUT_SAMPLES   100U /* 100 * 10 ms = 1 s */
#define BLUETOOTH_DEBUG_PERIOD_SAMPLES       10U
#define BLUETOOTH_STATUS_PERIOD_SAMPLES     100U /* 100 * 10 ms = 1 s */
#define BLUETOOTH_SLIDER_VALUE_LIMIT       1000.0f

/* Change either sign to -1 if the installed joystick or motor direction is reversed. */
#define BLUETOOTH_FORWARD_SIGN                1
#define BLUETOOTH_TURN_SIGN                   1

static volatile char s_build_buffer[BLUETOOTH_RX_BUFFER_SIZE];
static volatile char s_packet_buffer[BLUETOOTH_RX_BUFFER_SIZE];
static volatile uint8_t s_build_index = 0U;
static volatile uint8_t s_receive_state = 0U;
static volatile uint8_t s_packet_ready = 0U;
static volatile uint8_t s_enabled = 0U;
static volatile uint32_t s_rx_byte_count = 0U;
static volatile uint32_t s_rx_frame_count = 0U;
static volatile uint8_t s_last_rx_byte = 0U;

static uint8_t s_driving = 0U;
static uint8_t s_debug_first = 1U;
static uint32_t s_last_command_sample = 0U;
static uint32_t s_last_debug_sample = 0U;
static uint32_t s_last_status_sample = 0U;

static int16_t BluetoothControl_ClampJoystick(long value);
static uint8_t BluetoothControl_ParseValue(char *token, int16_t *value);
static uint8_t BluetoothControl_ParseJoystick(char *packet,
                                               int16_t *left_horizontal,
                                               int16_t *left_vertical,
                                               int16_t *right_horizontal,
                                               int16_t *right_vertical);
static char *BluetoothControl_TrimToken(char *token);
static uint8_t BluetoothControl_ParseFloat(char *token, float *value);
static uint8_t BluetoothControl_ParseSlider(char *packet,
                                             uint8_t *servo_number,
                                             float *value);
static void BluetoothControl_HandleReceivedByte(uint8_t data);
static void BluetoothControl_PollReceive(void);
static void BluetoothControl_ApplyJoystick(int16_t left_vertical,
                                           int16_t right_horizontal);
static void BluetoothControl_ApplyWheel(Motor_Numtypedef motor, int16_t command);
static void BluetoothControl_ApplyServoSlider(uint8_t servo_number, float angle_deg);

void BluetoothControl_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

    /* USART3 partial remap: PB10/PB11 -> PC10/PC11. */
    GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = BLUETOOTH_CONTROL_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART3, &USART_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1U;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2U;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART3, ENABLE);
    BluetoothControl_SetEnabled(1U);
}

void BluetoothControl_SetEnabled(uint8_t enabled)
{
    __disable_irq();
    s_enabled = (uint8_t)(enabled != 0U);
    s_build_index = 0U;
    s_receive_state = 0U;
    s_packet_ready = 0U;
    __enable_irq();

    BluetoothControl_Stop();
}

void BluetoothControl_Stop(void)
{
    PWM_Brake();
    s_driving = 0U;
}

uint8_t BluetoothControl_IsDriving(void)
{
    return s_driving;
}

void BluetoothControl_Process(void)
{
    char packet[BLUETOOTH_RX_BUFFER_SIZE];
    uint8_t has_packet = 0U;
    int16_t left_horizontal;
    int16_t left_vertical;
    int16_t right_horizontal;
    int16_t right_vertical;
    uint8_t servo_number;
    float arm_value;
    uint32_t now = MPU6050_YawSampleCount;

    /* Polling is a fallback if the USART3 RXNE interrupt is not dispatched. */
    BluetoothControl_PollReceive();

    if (!s_enabled)
    {
        return;
    }

    __disable_irq();
    if (s_packet_ready)
    {
        uint8_t i;
        for (i = 0U; i < BLUETOOTH_RX_BUFFER_SIZE; i++)
        {
            packet[i] = (char)s_packet_buffer[i];
            if (packet[i] == '\0')
            {
                break;
            }
        }
        packet[BLUETOOTH_RX_BUFFER_SIZE - 1U] = '\0';
        s_packet_ready = 0U;
        has_packet = 1U;
    }
    __enable_irq();

    if (has_packet)
    {
        printf("USART3 RX frame=[%s]\r\n", packet);

        if (strncmp(packet, "joystick,", 9U) == 0 &&
            BluetoothControl_ParseJoystick(packet,
                                            &left_horizontal,
                                            &left_vertical,
                                            &right_horizontal,
                                            &right_vertical))
        {
            (void)left_horizontal;
            (void)right_vertical;
            BluetoothControl_ApplyJoystick(left_vertical, right_horizontal);
            s_last_command_sample = now;

            if (s_debug_first ||
                (uint32_t)(now - s_last_debug_sample) >= BLUETOOTH_DEBUG_PERIOD_SAMPLES)
            {
                s_debug_first = 0U;
                s_last_debug_sample = now;
                printf("BT joystick LH=%d LV=%d RH=%d RV=%d driving=%u\r\n",
                       left_horizontal, left_vertical,
                       right_horizontal, right_vertical, s_driving);
            }
        }
        else if (strncmp(packet, "slider,", 7U) == 0 &&
                 BluetoothControl_ParseSlider(packet, &servo_number, &arm_value))
        {
            /* Arm operation and vehicle motion are mutually exclusive. */
            BluetoothControl_Stop();
            BluetoothControl_ApplyServoSlider(servo_number, arm_value);
        }
        else
        {
            printf("BT ignored frame: [%s]\r\n", packet);
            BluetoothControl_Printf("[servo,error,bad-frame]");
        }
    }

    if (s_driving &&
        (uint32_t)(now - s_last_command_sample) >= BLUETOOTH_COMMAND_TIMEOUT_SAMPLES)
    {
        BluetoothControl_Stop();
        printf("BT joystick timeout: motor braked\r\n");
    }
}

void BluetoothControl_DebugStatus(void)
{
    static uint8_t hardware_status_printed = 0U;
    uint32_t now = MPU6050_YawSampleCount;
    uint32_t byte_count;
    uint32_t frame_count;
    uint8_t last_byte;
    uint8_t enabled;
    uint8_t receive_state;
    uint8_t packet_ready;

    BluetoothControl_PollReceive();

    if ((uint32_t)(now - s_last_status_sample) < BLUETOOTH_STATUS_PERIOD_SAMPLES)
    {
        return;
    }
    s_last_status_sample = now;

    __disable_irq();
    byte_count = s_rx_byte_count;
    frame_count = s_rx_frame_count;
    last_byte = s_last_rx_byte;
    enabled = s_enabled;
    receive_state = s_receive_state;
    packet_ready = s_packet_ready;
    __enable_irq();

    printf("USART3 status: enabled=%u bytes=%lu frames=%lu last=0x%02X "
           "state=%u ready=%u\r\n",
           enabled, (unsigned long)byte_count, (unsigned long)frame_count,
           last_byte, receive_state, packet_ready);

    if (!hardware_status_printed)
    {
        hardware_status_printed = 1U;
        printf("USART3 HW: remap=0x%02lX GPIOC_CFGHR=0x%08lX "
               "CTLR1=0x%04lX BRR=0x%04lX STATR=0x%04lX\r\n",
               (unsigned long)(AFIO->PCFR1 & AFIO_PCFR1_USART3_REMAP),
               (unsigned long)GPIOC->CFGHR,
               (unsigned long)USART3->CTLR1,
               (unsigned long)USART3->BRR,
               (unsigned long)USART3->STATR);
    }
}

static int16_t BluetoothControl_ClampJoystick(long value)
{
    if (value > BLUETOOTH_JOYSTICK_LIMIT)
    {
        return BLUETOOTH_JOYSTICK_LIMIT;
    }
    if (value < -BLUETOOTH_JOYSTICK_LIMIT)
    {
        return -BLUETOOTH_JOYSTICK_LIMIT;
    }
    return (int16_t)value;
}

static uint8_t BluetoothControl_ParseValue(char *token, int16_t *value)
{
    char *end;
    long parsed;

    if (token == 0 || value == 0)
    {
        return 0U;
    }

    parsed = strtol(token, &end, 10);
    while (*end == ' ' || *end == '\t')
    {
        end++;
    }
    if (*end != '\0')
    {
        return 0U;
    }

    *value = BluetoothControl_ClampJoystick(parsed);
    return 1U;
}

static uint8_t BluetoothControl_ParseJoystick(char *packet,
                                               int16_t *left_horizontal,
                                               int16_t *left_vertical,
                                               int16_t *right_horizontal,
                                               int16_t *right_vertical)
{
    char *tag = strtok(packet, ",");
    char *lh;
    char *lv;
    char *rh;
    char *rv;

    if (tag == 0 || strcmp(tag, "joystick") != 0)
    {
        return 0U;
    }

    lh = strtok(0, ",");
    lv = strtok(0, ",");
    rh = strtok(0, ",");
    rv = strtok(0, ",");

    if (!BluetoothControl_ParseValue(lh, left_horizontal) ||
        !BluetoothControl_ParseValue(lv, left_vertical) ||
        !BluetoothControl_ParseValue(rh, right_horizontal) ||
        !BluetoothControl_ParseValue(rv, right_vertical) ||
        strtok(0, ",") != 0)
    {
        return 0U;
    }

    return 1U;
}

static char *BluetoothControl_TrimToken(char *token)
{
    char *end;

    if (token == 0)
    {
        return 0;
    }

    while (*token == ' ' || *token == '\t')
    {
        token++;
    }

    end = token + strlen(token);
    while (end > token && (end[-1] == ' ' || end[-1] == '\t'))
    {
        end--;
    }
    *end = '\0';
    return token;
}

static uint8_t BluetoothControl_ParseFloat(char *token, float *value)
{
    char *end;
    float parsed;

    token = BluetoothControl_TrimToken(token);
    if (token == 0 || *token == '\0' || value == 0)
    {
        return 0U;
    }

    parsed = strtof(token, &end);
    while (*end == ' ' || *end == '\t')
    {
        end++;
    }
    if (end == token || *end != '\0' || parsed != parsed ||
        parsed < -BLUETOOTH_SLIDER_VALUE_LIMIT ||
        parsed > BLUETOOTH_SLIDER_VALUE_LIMIT)
    {
        return 0U;
    }

    *value = parsed;
    return 1U;
}

static uint8_t BluetoothControl_ParseSlider(char *packet,
                                             uint8_t *servo_number,
                                             float *value)
{
    char *tag = strtok(packet, ",");
    char *name = BluetoothControl_TrimToken(strtok(0, ","));
    char *value_token = strtok(0, ",");

    if (tag == 0 || strcmp(BluetoothControl_TrimToken(tag), "slider") != 0 ||
        name == 0 || servo_number == 0 || value == 0 ||
        !BluetoothControl_ParseFloat(value_token, value) || strtok(0, ",") != 0)
    {
        return 0U;
    }

    /* "ACK" is accepted as servo 1 for compatibility with the supplied UI. */
    if (strcmp(name, "1") == 0 || strcmp(name, "J1") == 0 ||
        strcmp(name, "j1") == 0 || strcmp(name, "S1") == 0 ||
        strcmp(name, "s1") == 0 || strcmp(name, "ACK") == 0)
    {
        *servo_number = 1U;
    }
    else if (strcmp(name, "2") == 0 || strcmp(name, "J2") == 0 ||
             strcmp(name, "j2") == 0 || strcmp(name, "S2") == 0 ||
             strcmp(name, "s2") == 0)
    {
        *servo_number = 2U;
    }
    else if (strcmp(name, "3") == 0 || strcmp(name, "J3") == 0 ||
             strcmp(name, "j3") == 0 || strcmp(name, "S3") == 0 ||
             strcmp(name, "s3") == 0)
    {
        *servo_number = 3U;
    }
    else if (strcmp(name, "4") == 0 || strcmp(name, "J4") == 0 ||
             strcmp(name, "j4") == 0 || strcmp(name, "S4") == 0 ||
             strcmp(name, "s4") == 0 || strcmp(name, "GRIP") == 0 ||
             strcmp(name, "grip") == 0)
    {
        *servo_number = 4U;
    }
    else
    {
        return 0U;
    }

    return 1U;
}

static void BluetoothControl_ApplyJoystick(int16_t left_vertical,
                                           int16_t right_horizontal)
{
    int32_t forward;
    int32_t turn;
    int32_t left_command;
    int32_t right_command;
    int32_t peak;

    if (left_vertical > -BLUETOOTH_JOYSTICK_DEADZONE &&
        left_vertical < BLUETOOTH_JOYSTICK_DEADZONE)
    {
        left_vertical = 0;
    }
    if (right_horizontal > -BLUETOOTH_JOYSTICK_DEADZONE &&
        right_horizontal < BLUETOOTH_JOYSTICK_DEADZONE)
    {
        right_horizontal = 0;
    }

    forward = (int32_t)left_vertical * BLUETOOTH_FORWARD_SIGN;
    turn = (int32_t)right_horizontal * BLUETOOTH_TURN_SIGN;
    left_command = forward + turn;
    right_command = forward - turn;

    peak = labs(left_command);
    if (labs(right_command) > peak)
    {
        peak = labs(right_command);
    }
    if (peak > BLUETOOTH_JOYSTICK_LIMIT)
    {
        left_command = left_command * BLUETOOTH_JOYSTICK_LIMIT / peak;
        right_command = right_command * BLUETOOTH_JOYSTICK_LIMIT / peak;
    }

    if (left_command == 0 && right_command == 0)
    {
        BluetoothControl_Stop();
        return;
    }

    BluetoothControl_ApplyWheel(Motor_Left, (int16_t)left_command);
    BluetoothControl_ApplyWheel(Motor_Right, (int16_t)right_command);
    s_driving = 1U;
}

static void BluetoothControl_ApplyWheel(Motor_Numtypedef motor, int16_t command)
{
    uint16_t magnitude;
    uint16_t drive_percent;
    uint16_t compare;
    Motor_Directtypedef direction;

    if (command == 0)
    {
        /* With PWM1 and the other bridge input held high, CCR=ARR+1 brakes. */
        PWM_EnableChannel(motor, BLUETOOTH_PWM_PERIOD_COUNTS, Forward);
        return;
    }

    magnitude = (uint16_t)((command < 0) ? -command : command);
    drive_percent = (uint16_t)((magnitude * BLUETOOTH_MAX_DRIVE_PERCENT + 50U) /
                               BLUETOOTH_JOYSTICK_LIMIT);
    compare = (uint16_t)(BLUETOOTH_PWM_PERIOD_COUNTS - drive_percent);
    direction = (command > 0) ? Forward : Reverse;
    PWM_EnableChannel(motor, compare, direction);
}

static void BluetoothControl_ApplyServoSlider(uint8_t servo_number, float angle_deg)
{
    printf("BT servo slider S%u=%.1f deg\r\n",
           (unsigned int)servo_number, angle_deg);

    if (!Arm_SetServoAngle(servo_number, angle_deg))
    {
        BluetoothControl_Printf("[servo,error,%u,%.1f]",
                                (unsigned int)servo_number, angle_deg);
        return;
    }

    BluetoothControl_Printf("[servo,ok,%u,%.1f]",
                            (unsigned int)servo_number,
                            ArmServo_GetAngle((uint8_t)(servo_number - 1U)));
}

void BluetoothControl_SendByte(uint8_t data)
{
    USART_SendData(USART3, data);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
    {
    }
}

void BluetoothControl_SendString(const char *string)
{
    if (string == 0)
    {
        return;
    }

    while (*string != '\0')
    {
        BluetoothControl_SendByte((uint8_t)*string++);
    }
}

void BluetoothControl_Printf(const char *format, ...)
{
    char buffer[100];
    va_list args;

    if (format == 0)
    {
        return;
    }

    va_start(args, format);
    (void)vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    BluetoothControl_SendString(buffer);
}

static void BluetoothControl_HandleReceivedByte(uint8_t data)
{
    s_rx_byte_count++;
    s_last_rx_byte = data;

    if (data == '[')
    {
        s_receive_state = 1U;
        s_build_index = 0U;
    }
    else if (s_receive_state)
    {
        if (data == ']')
        {
            uint8_t i;
            s_build_buffer[s_build_index] = '\0';
            s_receive_state = 0U;

            if (s_build_index > 0U)
            {
                s_rx_frame_count++;
            }

            if (s_enabled && s_build_index > 0U)
            {
                /* Keep the newest complete frame while a slow arm move runs. */
                for (i = 0U; i <= s_build_index; i++)
                {
                    s_packet_buffer[i] = s_build_buffer[i];
                }
                s_packet_ready = 1U;
            }
            s_build_index = 0U;
        }
        else if (s_build_index < (BLUETOOTH_RX_BUFFER_SIZE - 1U))
        {
            s_build_buffer[s_build_index++] = (char)data;
        }
        else
        {
            s_receive_state = 0U;
            s_build_index = 0U;
        }
    }
}

static void BluetoothControl_PollReceive(void)
{
    while (1)
    {
        uint8_t data;

        /* Prevent the RXNE ISR from reading the same byte between test and read. */
        __disable_irq();
        if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET)
        {
            __enable_irq();
            break;
        }

        data = (uint8_t)USART_ReceiveData(USART3);
        BluetoothControl_HandleReceivedByte(data);
        __enable_irq();
    }
}

void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        uint8_t data = (uint8_t)USART_ReceiveData(USART3);
        BluetoothControl_HandleReceivedByte(data);
        /* Reading DATAR clears RXNE; no explicit pending-bit clear is required. */
    }
}
