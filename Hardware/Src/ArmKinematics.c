#include "ArmKinematics.h"
#include "debug.h"
#include "Motor.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef ARM_PI
#define ARM_PI 3.14159265358979323846f
#endif

typedef struct
{
    ArmJointAngles joint;
    float servo[3];
    uint8_t valid;
} ArmIKCandidate;

static volatile char s_host_rx_buf[ARM_HOST_RX_BUF_SIZE];
static volatile char s_host_pending_buf[ARM_HOST_RX_BUF_SIZE];
static volatile uint8_t s_host_rx_index = 0U;
static volatile uint8_t s_host_cmd_ready = 0U;
static char s_host_cmd_buf[ARM_HOST_RX_BUF_SIZE];

static float Arm_DegToRad(float deg)
{
    return deg * ARM_PI / 180.0f;
}

static float Arm_RadToDeg(float rad)
{
    return rad * 180.0f / ARM_PI;
}

static float Arm_LimitFloat(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static uint8_t Arm_RawServoAngleIsSafe(float angle_deg)
{
    return (uint8_t)(angle_deg >= ARM_DEBUG_SERVO_MIN_DEG &&
                     angle_deg <= ARM_DEBUG_SERVO_MAX_DEG);
}

static uint8_t Arm_JointToServoAngles(const ArmJointAngles *joint, float servo[3])
{
    if (joint == 0 || servo == 0)
    {
        return 0U;
    }

    servo[0] = ARM_J1_SERVO_ZERO_DEG + ARM_J1_SERVO_SIGN * joint->j1;
    servo[1] = ARM_J2_SERVO_ZERO_DEG + ARM_J2_SERVO_SIGN * joint->j2;
    servo[2] = ARM_J3_SERVO_ZERO_DEG + ARM_J3_SERVO_SIGN * joint->j3;

    return (uint8_t)(Arm_RawServoAngleIsSafe(servo[0]) &&
                     Arm_RawServoAngleIsSafe(servo[1]) &&
                     Arm_RawServoAngleIsSafe(servo[2]));
}

static void Arm_ServoToJointAngles(float servo1, float servo2, float servo3,
                                   ArmJointAngles *joint)
{
    if (joint == 0)
    {
        return;
    }

    joint->j1 = (servo1 - ARM_J1_SERVO_ZERO_DEG) / ARM_J1_SERVO_SIGN;
    joint->j2 = (servo2 - ARM_J2_SERVO_ZERO_DEG) / ARM_J2_SERVO_SIGN;
    joint->j3 = (servo3 - ARM_J3_SERVO_ZERO_DEG) / ARM_J3_SERVO_SIGN;
}

static void Arm_GetCurrentJointAngles(ArmJointAngles *joint)
{
    Arm_ServoToJointAngles(ArmServo_GetAngle(ARM_SERVO_J1_BASE),
                           ArmServo_GetAngle(ARM_SERVO_J2_SHOULDER),
                           ArmServo_GetAngle(ARM_SERVO_J3_WRIST), joint);
}

void Arm_ForwardKinematics(const ArmJointAngles *angles, ArmPoint3D *point)
{
    float q1;
    float q2;
    float q23;
    float radial;

    if (angles == 0 || point == 0)
    {
        return;
    }

    q1 = Arm_DegToRad(angles->j1);
    q2 = Arm_DegToRad(angles->j2);
    q23 = Arm_DegToRad(angles->j2 + angles->j3);

    radial = ARM_LINK_J2_TO_J3_MM * cosf(q2) +
             ARM_LINK_J3_TO_GRIP_CENTER_MM * cosf(q23);
    point->x = radial * cosf(q1);
    point->y = radial * sinf(q1);
    point->z = ARM_J2_AXIS_HEIGHT_MM +
               ARM_LINK_J2_TO_J3_MM * sinf(q2) +
               ARM_LINK_J3_TO_GRIP_CENTER_MM * sinf(q23);
}

uint8_t Arm_GetCurrentPosition(ArmPoint3D *point)
{
    ArmJointAngles joint;

    if (point == 0)
    {
        return 0U;
    }

    Arm_GetCurrentJointAngles(&joint);
    Arm_ForwardKinematics(&joint, point);
    return 1U;
}

static uint8_t Arm_CalculateIKCandidates(float x, float y, float z,
                                         ArmIKCandidate candidates[2])
{
    float radial = sqrtf(x * x + y * y);
    float height = z - ARM_J2_AXIS_HEIGHT_MM;
    float l1 = ARM_LINK_J2_TO_J3_MM;
    float l2 = ARM_LINK_J3_TO_GRIP_CENTER_MM;
    float d;
    float root;
    float q1;
    uint8_t i;

    if (candidates == 0 || l1 <= 0.0f || l2 <= 0.0f)
    {
        return 0U;
    }

    memset(candidates, 0, sizeof(ArmIKCandidate) * 2U);
    d = (radial * radial + height * height - l1 * l1 - l2 * l2) /
        (2.0f * l1 * l2);
    if (d < -1.0001f || d > 1.0001f)
    {
        return 0U;
    }

    d = Arm_LimitFloat(d, -1.0f, 1.0f);
    root = sqrtf(Arm_LimitFloat(1.0f - d * d, 0.0f, 1.0f));
    q1 = atan2f(y, x);

    for (i = 0U; i < 2U; i++)
    {
        float q3 = atan2f((i == 0U) ? root : -root, d);
        float q2 = atan2f(height, radial) -
                   atan2f(l2 * sinf(q3), l1 + l2 * cosf(q3));

        candidates[i].joint.j1 = Arm_RadToDeg(q1);
        candidates[i].joint.j2 = Arm_RadToDeg(q2);
        candidates[i].joint.j3 = Arm_RadToDeg(q3);
        candidates[i].valid =
            Arm_JointToServoAngles(&candidates[i].joint, candidates[i].servo);
    }

    return 1U;
}

static int8_t Arm_SelectIKCandidate(const ArmIKCandidate candidates[2])
{
    ArmJointAngles current;
    float best_cost = 1.0e30f;
    int8_t best = -1;
    uint8_t i;

    Arm_GetCurrentJointAngles(&current);
    for (i = 0U; i < 2U; i++)
    {
        float d1;
        float d2;
        float d3;
        float cost;

        if (!candidates[i].valid)
        {
            continue;
        }

        d1 = candidates[i].joint.j1 - current.j1;
        d2 = candidates[i].joint.j2 - current.j2;
        d3 = candidates[i].joint.j3 - current.j3;
        cost = d1 * d1 + d2 * d2 + d3 * d3;
        if (cost < best_cost)
        {
            best_cost = cost;
            best = (int8_t)i;
        }
    }

    return best;
}

uint8_t Arm_InverseKinematics(float x, float y, float z, ArmJointAngles *angles)
{
    ArmIKCandidate candidates[2];
    int8_t selected;

    if (angles == 0 || !Arm_CalculateIKCandidates(x, y, z, candidates))
    {
        return 0U;
    }

    selected = Arm_SelectIKCandidate(candidates);
    if (selected < 0)
    {
        return 0U;
    }

    *angles = candidates[(uint8_t)selected].joint;
    return 1U;
}

uint8_t Arm_SetJointAngles(const ArmJointAngles *angles)
{
    float servo[3];

    if (!Arm_JointToServoAngles(angles, servo))
    {
        printf("ARM joint rejected: servo limit\r\n");
        return 0U;
    }

    ArmServo_ClearStop();
    ArmServo_SoftSetAngle(ARM_SERVO_J1_BASE, servo[0]);
    if (ArmServo_StopRequested()) return 0U;
    ArmServo_SoftSetAngle(ARM_SERVO_J2_SHOULDER, servo[1]);
    if (ArmServo_StopRequested()) return 0U;
    ArmServo_SoftSetAngle(ARM_SERVO_J3_WRIST, servo[2]);
    if (ArmServo_StopRequested()) return 0U;

    printf("ARM moved: q1=%.1f q2=%.1f q3=%.1f; servo=%.1f,%.1f,%.1f\r\n",
           angles->j1, angles->j2, angles->j3,
           servo[0], servo[1], servo[2]);
    return 1U;
}

uint8_t Arm_MoveToXYZ(float x, float y, float z)
{
#if ARM_CARTESIAN_MOVE_ENABLED
    ArmJointAngles angles;

    if (!Arm_InverseKinematics(x, y, z, &angles))
    {
        printf("ARM move failed: unreachable or servo limit\r\n");
        return 0U;
    }
    return Arm_SetJointAngles(&angles);
#else
    (void)x;
    (void)y;
    (void)z;
    printf("ARM Cartesian move locked: calibrate dimensions/zeros, then set "
           "ARM_CARTESIAN_MOVE_ENABLED=1\r\n");
    return 0U;
#endif
}

void Arm_Init(void)
{
    ArmServo_Init();
    ArmServo_SetInitialAngle(ARM_SERVO_J1_BASE, ARM_J1_HOME_DEG);
    ArmServo_SetInitialAngle(ARM_SERVO_J2_SHOULDER, ARM_J2_HOME_DEG);
    ArmServo_SetInitialAngle(ARM_SERVO_J3_WRIST, ARM_J3_HOME_DEG);
    ArmServo_SetInitialAngle(ARM_SERVO_GRIPPER, ARM_GRIP_OPEN_DEG);

    printf("ARM debug ready; PWM outputs idle until S/HOME/OPEN/CLOSE\r\n");
    printf("ARM provisional geometry: H=%.1f L1=%.1f L2=%.1f mm, Cartesian=%s\r\n",
           ARM_J2_AXIS_HEIGHT_MM, ARM_LINK_J2_TO_J3_MM,
           ARM_LINK_J3_TO_GRIP_CENTER_MM,
           ARM_CARTESIAN_MOVE_ENABLED ? "enabled" : "locked");
}

void Arm_GripOpen(void)
{
    ArmServo_ClearStop();
    ArmServo_SoftSetAngle(ARM_SERVO_GRIPPER, ARM_GRIP_OPEN_DEG);
    printf(ArmServo_StopRequested() ? "ARM grip stopped\r\n" : "ARM grip open\r\n");
}

void Arm_GripClose(void)
{
    ArmServo_ClearStop();
    ArmServo_SoftSetAngle(ARM_SERVO_GRIPPER, ARM_GRIP_CLOSE_DEG);
    printf(ArmServo_StopRequested() ? "ARM grip stopped\r\n" : "ARM grip close\r\n");
}

static void Arm_Home(void)
{
    float home[ARM_SERVO_COUNT] = {
        ARM_J1_HOME_DEG,
        ARM_J2_HOME_DEG,
        ARM_J3_HOME_DEG,
        ARM_GRIP_OPEN_DEG
    };

    ArmServo_ClearStop();
    ArmServo_SoftSetAll(home);
    printf(ArmServo_StopRequested() ? "ARM HOME stopped\r\n" : "ARM HOME done\r\n");
}

uint8_t Arm_SetServoAngle(uint8_t servo_number, float angle_deg)
{
    uint8_t servo_id;

    if (servo_number < 1U || servo_number > ARM_SERVO_COUNT ||
        !Arm_RawServoAngleIsSafe(angle_deg))
    {
        printf("ARM S rejected: id=1..4 angle=%.0f..%.0f\r\n",
               ARM_DEBUG_SERVO_MIN_DEG, ARM_DEBUG_SERVO_MAX_DEG);
        return 0U;
    }

    servo_id = (uint8_t)(servo_number - 1U);
    ArmServo_ClearStop();
    ArmServo_SoftSetAngle(servo_id, angle_deg);
    if (ArmServo_StopRequested())
    {
        printf("ARM S%u stopped at %.1f\r\n", (unsigned int)servo_number,
               ArmServo_GetAngle(servo_id));
        return 0U;
    }

    printf("ARM S%u=%.1f pulse=%u us\r\n", (unsigned int)servo_number,
           ArmServo_GetAngle(servo_id),
           ArmServo_AngleToPulse(ArmServo_GetAngle(servo_id)));
    return 1U;
}

static void Arm_PrintState(void)
{
    ArmJointAngles joint;
    ArmPoint3D point;

    Arm_GetCurrentJointAngles(&joint);
    Arm_ForwardKinematics(&joint, &point);
    printf("ARM RAW S1=%.1f(%u) S2=%.1f(%u) S3=%.1f(%u) S4=%.1f(%u)\r\n",
           ArmServo_GetAngle(0U), ArmServo_IsEnabled(0U),
           ArmServo_GetAngle(1U), ArmServo_IsEnabled(1U),
           ArmServo_GetAngle(2U), ArmServo_IsEnabled(2U),
           ArmServo_GetAngle(3U), ArmServo_IsEnabled(3U));
    printf("ARM JOINT q1=%.1f q2=%.1f q3=%.1f\r\n",
           joint.j1, joint.j2, joint.j3);
    printf("ARM FK claw-center x=%.1f y=%.1f z=%.1f mm\r\n",
           point.x, point.y, point.z);
}

static void Arm_DebugIK(float x, float y, float z)
{
    ArmIKCandidate candidates[2];
    int8_t selected;
    uint8_t i;

    printf("ARM IK target x=%.1f y=%.1f z=%.1f mm\r\n", x, y, z);
#if !ARM_GEOMETRY_CALIBRATED
    printf("ARM IK warning: provisional dimensions/servo zeros\r\n");
#endif

    if (!Arm_CalculateIKCandidates(x, y, z, candidates))
    {
        printf("ARM IK unreachable by geometry\r\n");
        return;
    }

    for (i = 0U; i < 2U; i++)
    {
        printf("ARM IK %c q=%.1f,%.1f,%.1f servo=%.1f,%.1f,%.1f valid=%u\r\n",
               (i == 0U) ? 'A' : 'B',
               candidates[i].joint.j1, candidates[i].joint.j2,
               candidates[i].joint.j3, candidates[i].servo[0],
               candidates[i].servo[1], candidates[i].servo[2],
               candidates[i].valid);
    }

    selected = Arm_SelectIKCandidate(candidates);
    if (selected < 0)
    {
        printf("ARM IK no solution inside servo limits\r\n");
    }
    else
    {
        printf("ARM IK selected=%c (dry-run, no movement)\r\n",
               (selected == 0) ? 'A' : 'B');
    }
}

static void Arm_PrintHelp(void)
{
    printf("ARM CMD: S id angle | HOME | READ | FK | IK x y z\r\n");
    printf("ARM CMD: P x y z | G x y z | OPEN | CLOSE | STOP | HELP\r\n");
    printf("ARM USART1 packets are ASCII and require CRLF\r\n");
}

static void Arm_SkipSeparators(char **cursor)
{
    while (**cursor == ' ' || **cursor == '\t' || **cursor == ',')
    {
        (*cursor)++;
    }
}

static uint8_t Arm_ParseFloat(char **cursor, float *value)
{
    char *end;

    Arm_SkipSeparators(cursor);
    if (**cursor == '\0')
    {
        return 0U;
    }

    *value = strtof(*cursor, &end);
    if (end == *cursor)
    {
        return 0U;
    }
    *cursor = end;
    return 1U;
}

static uint8_t Arm_ParseLong(char **cursor, long *value)
{
    char *end;

    Arm_SkipSeparators(cursor);
    if (**cursor == '\0')
    {
        return 0U;
    }

    *value = strtol(*cursor, &end, 10);
    if (end == *cursor)
    {
        return 0U;
    }
    *cursor = end;
    return 1U;
}

static uint8_t Arm_AtCommandEnd(char *cursor)
{
    Arm_SkipSeparators(&cursor);
    return (uint8_t)(*cursor == '\0');
}

static uint8_t Arm_IsStopLine(const volatile char *line, uint8_t length)
{
    return (uint8_t)(length == 4U &&
                     (line[0] == 'S' || line[0] == 's') &&
                     (line[1] == 'T' || line[1] == 't') &&
                     (line[2] == 'O' || line[2] == 'o') &&
                     (line[3] == 'P' || line[3] == 'p'));
}

void Arm_HostUartInit(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1U;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0U;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        char data = (char)USART_ReceiveData(USART1);

        if (data == '\r' || data == '\n')
        {
            if (s_host_rx_index > 0U)
            {
                uint8_t i;
                uint8_t is_stop;
                s_host_rx_buf[s_host_rx_index] = '\0';
                is_stop = Arm_IsStopLine(s_host_rx_buf, s_host_rx_index);
                if (is_stop)
                {
                    ArmServo_RequestStop();
                }
                /* STOP has priority and replaces any queued non-safety command. */
                if (is_stop || !s_host_cmd_ready)
                {
                    for (i = 0U; i <= s_host_rx_index; i++)
                    {
                        s_host_pending_buf[i] = s_host_rx_buf[i];
                    }
                    s_host_cmd_ready = 1U;
                }
            }
            s_host_rx_index = 0U;
        }
        else if (s_host_rx_index < (ARM_HOST_RX_BUF_SIZE - 1U))
        {
            s_host_rx_buf[s_host_rx_index++] = data;
        }
        else
        {
            s_host_rx_index = 0U;
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

void Arm_ProcessHostCommand(void)
{
    char command[12];
    char *cursor;
    uint8_t command_length = 0U;

    if (!s_host_cmd_ready)
    {
        return;
    }

    __disable_irq();
    strncpy(s_host_cmd_buf, (const char *)s_host_pending_buf, ARM_HOST_RX_BUF_SIZE);
    s_host_cmd_buf[ARM_HOST_RX_BUF_SIZE - 1U] = '\0';
    s_host_cmd_ready = 0U;
    __enable_irq();

    cursor = s_host_cmd_buf;
    Arm_SkipSeparators(&cursor);
    while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t' &&
           *cursor != ',' && command_length < (sizeof(command) - 1U))
    {
        command[command_length++] = (char)toupper((unsigned char)*cursor++);
    }
    command[command_length] = '\0';

    printf("USART1 ARM cmd='%s'\r\n", s_host_cmd_buf);

    if (strcmp(command, "S") == 0)
    {
        long servo_number;
        float angle;
        if (!Arm_ParseLong(&cursor, &servo_number) ||
            !Arm_ParseFloat(&cursor, &angle) || !Arm_AtCommandEnd(cursor))
        {
            printf("ARM usage: S id angle\r\n");
        }
        else
        {
            if (servo_number < 1L || servo_number > (long)ARM_SERVO_COUNT)
            {
                printf("ARM S rejected: id=1..4\r\n");
            }
            else
            {
                (void)Arm_SetServoAngle((uint8_t)servo_number, angle);
            }
        }
    }
    else if (strcmp(command, "HOME") == 0 && Arm_AtCommandEnd(cursor))
    {
        Arm_Home();
    }
    else if ((strcmp(command, "READ") == 0 || strcmp(command, "FK") == 0) &&
             Arm_AtCommandEnd(cursor))
    {
        Arm_PrintState();
    }
    else if (strcmp(command, "IK") == 0)
    {
        float x;
        float y;
        float z;
        if (!Arm_ParseFloat(&cursor, &x) || !Arm_ParseFloat(&cursor, &y) ||
            !Arm_ParseFloat(&cursor, &z) || !Arm_AtCommandEnd(cursor))
        {
            printf("ARM usage: IK x y z\r\n");
        }
        else
        {
            Arm_DebugIK(x, y, z);
        }
    }
    else if (strcmp(command, "P") == 0 || strcmp(command, "G") == 0)
    {
        float x;
        float y;
        float z;
        if (!Arm_ParseFloat(&cursor, &x) || !Arm_ParseFloat(&cursor, &y) ||
            !Arm_ParseFloat(&cursor, &z) || !Arm_AtCommandEnd(cursor))
        {
            printf("ARM usage: P/G x y z\r\n");
        }
        else if (Arm_MoveToXYZ(x, y, z) && strcmp(command, "G") == 0)
        {
            Arm_GripClose();
        }
    }
    else if (strcmp(command, "OPEN") == 0 && Arm_AtCommandEnd(cursor))
    {
        Arm_GripOpen();
    }
    else if (strcmp(command, "CLOSE") == 0 && Arm_AtCommandEnd(cursor))
    {
        Arm_GripClose();
    }
    else if (strcmp(command, "STOP") == 0 && Arm_AtCommandEnd(cursor))
    {
        ArmServo_RequestStop();
        printf("ARM STOP: holding current angles\r\n");
    }
    else if (strcmp(command, "HELP") == 0 && Arm_AtCommandEnd(cursor))
    {
        Arm_PrintHelp();
    }
    else if ((strcmp(command, "00") == 0 || strcmp(command, "11") == 0 ||
              strcmp(command, "22") == 0 || strcmp(command, "33") == 0 ||
              strcmp(command, "44") == 0 || strcmp(command, "55") == 0 ||
              strcmp(command, "66") == 0 || strcmp(command, "77") == 0 ||
              strcmp(command, "88") == 0) && Arm_AtCommandEnd(cursor))
    {
        (void)Motor_TurnToYawCommand((uint8_t)strtoul(command, 0, 10));
    }
    else
    {
        printf("ARM unknown command; send HELP\r\n");
    }
}
