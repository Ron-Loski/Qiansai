#ifndef __ARM_KINEMATICS_H_
#define __ARM_KINEMATICS_H_

#include "ArmServo.h"
#include <stdint.h>
#include "ch32v30x.h"
/* ==================== 机械臂各轴长度宏定义，单位：mm ==================== */
#define ARM_LINK_A1_BASE_HEIGHT_MM      60.0f   /* A1：底座到肩关节的高度 */
#define ARM_LINK_A2_SHOULDER_MM         90.0f   /* A2：肩关节到肘关节长度 */
#define ARM_LINK_A3_ELBOW_MM            90.0f   /* A3：肘关节到腕关节长度 */
#define ARM_LINK_A4_TOOL_MM             60.0f   /* A4：腕关节到末端执行器长度 */
#define ARM_BASE_OFFSET_P_MM            0.0f    /* P：底座平面偏移，未知时先置 0 */

/* ==================== 舵机安装零位和限制，按实际机械结构可标定 ==================== */
#define ARM_J1_HOME_DEG                 90.0f
#define ARM_J2_HOME_DEG                 90.0f
#define ARM_J3_HOME_DEG                 90.0f
#define ARM_J4_HOME_DEG                 90.0f

#define ARM_J1_MIN_DEG                  0.0f
#define ARM_J1_MAX_DEG                  180.0f
#define ARM_J2_MIN_DEG                  0.0f
#define ARM_J2_MAX_DEG                  180.0f
#define ARM_J3_MIN_DEG                  0.0f
#define ARM_J3_MAX_DEG                  180.0f
#define ARM_J4_MIN_DEG                  -70.0f
#define ARM_J4_MAX_DEG                  180.0f

#define ARM_TOOL_PITCH_DEG              90.0f   /* 末端默认姿态角 alpha，按图中 j2+j3+j4=alpha */
#define ARM_GRIP_OPEN_DEG               40.0f
#define ARM_GRIP_CLOSE_DEG              95.0f

#define ARM_HOST_RX_BUF_SIZE            64U

typedef struct
{
    float x;
    float y;
    float z;
} ArmPoint3D;

typedef struct
{
    float j1;
    float j2;
    float j3;
    float j4;
} ArmJointAngles;

void Arm_Init(void);
uint8_t Arm_InverseKinematics(float x, float y, float z, ArmJointAngles *angles);
uint8_t Arm_SetJointAngles(const ArmJointAngles *angles);
uint8_t Arm_MoveToXYZ(float x, float y, float z);
void Arm_GripOpen(void);
void Arm_GripClose(void);
void Arm_HostUartInit(uint32_t baudrate);
void Arm_ProcessHostCommand(void);

#endif
