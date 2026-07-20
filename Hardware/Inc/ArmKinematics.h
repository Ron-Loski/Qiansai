#ifndef __ARM_KINEMATICS_H_
#define __ARM_KINEMATICS_H_

#include "ArmServo.h"
#include "ch32v30x.h"
#include <stdint.h>

/*
 * Geometry in mm. These provisional values preserve the old project's total
 * dimensions. Replace them with shaft-center measurements before enabling P/G.
 */
#define ARM_WORLD_TO_J1_HEIGHT_MM          50.0f
#define ARM_J1_TO_J2_HEIGHT_MM              20.0f
#define ARM_LINK_J2_TO_J3_MM              100.0f
#define ARM_LINK_J3_TO_GRIP_CENTER_MM      120.0f
#define ARM_J2_AXIS_HEIGHT_MM \
    (ARM_WORLD_TO_J1_HEIGHT_MM + ARM_J1_TO_J2_HEIGHT_MM)

/* Keep Cartesian motion locked until geometry and servo mapping are measured. */
#define ARM_GEOMETRY_CALIBRATED             1U
#define ARM_CARTESIAN_MOVE_ENABLED          1U

/* Raw servo angles corresponding to mechanical q1=q2=q3=0 degrees. */
#define ARM_J1_SERVO_ZERO_DEG              90.0f
#define ARM_J2_SERVO_ZERO_DEG              90.0f
#define ARM_J3_SERVO_ZERO_DEG              90.0f

/* Set to +1 or -1 after checking each servo's positive direction. */
#define ARM_J1_SERVO_SIGN                   1.0f
#define ARM_J2_SERVO_SIGN                   1.0f
#define ARM_J3_SERVO_SIGN                   1.0f

/* Conservative raw-angle limit used by S/HOME and inverse-solution checks. */
#define ARM_DEBUG_SERVO_MIN_DEG             5.0f
#define ARM_DEBUG_SERVO_MAX_DEG           175.0f

#define ARM_J1_HOME_DEG                    90.0f
#define ARM_J2_HOME_DEG                    90.0f
#define ARM_J3_HOME_DEG                    90.0f
#define ARM_GRIP_OPEN_DEG                  40.0f
#define ARM_GRIP_CLOSE_DEG                 95.0f

#define ARM_HOST_RX_BUF_SIZE               64U

typedef struct
{
    float x;
    float y;
    float z;
} ArmPoint3D;

/* Mechanical angles: q1 yaw, q2 from horizontal, q3 relative to link L1. */
typedef struct
{
    float j1;
    float j2;
    float j3;
} ArmJointAngles;

void Arm_Init(void);
void Arm_ForwardKinematics(const ArmJointAngles *angles, ArmPoint3D *point);
uint8_t Arm_GetCurrentPosition(ArmPoint3D *point);
uint8_t Arm_InverseKinematics(float x, float y, float z, ArmJointAngles *angles);
uint8_t Arm_SetJointAngles(const ArmJointAngles *angles);
uint8_t Arm_SetServoAngle(uint8_t servo_number, float angle_deg);
uint8_t Arm_MoveToXYZ(float x, float y, float z);
void Arm_GripOpen(void);
void Arm_GripClose(void);
void Arm_HostUartInit(uint32_t baudrate);
void Arm_ProcessHostCommand(void);

#endif
