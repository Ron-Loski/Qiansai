#ifndef __MPU6050_H_
#define __MPU6050_H_

#include "ch32v30x.h"
#include "Varibles.h"
#include "MPU6050_Reg.h"
#include "math.h"
#include "PID.h"

#define SCL_PORT	GPIOB
#define SCL_PIN		GPIO_Pin_10

void MyI2C_Init(void);
void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8_t Byte);
uint8_t MyI2C_ReceiveByte(void);
void MyI2C_SendAck(uint8_t AckBit);
uint8_t MyI2C_ReceiveAck(void);

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
void MPU6050_Getaxis_Z(MPU6050_Zaxistypedef p);
void MPU6050_GetRoll(void);


#endif