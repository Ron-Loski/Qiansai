#ifndef __VARIBLES_H_
#define __VARIBLES_H_

#include "ch32v30x.h"

typedef struct
{
    int16_t AccX;
	int16_t AccY;
	int16_t AccZ;
	
	int16_t GyroX;
	int16_t GyroY;
	int16_t GyroZ; 
}MPU6050_Datatypedef ;

typedef struct
{
    float AccX;
	float AccY;
	float AccZ;
}MPU6050_AngleAcctypedef;

typedef struct{
	float GyroX;
	float GyroY;
	float GyroZ;
}MPU6050_AngleGyrotypedef;

typedef struct{
	float Roll;
	float Pitch;
	float Yaw;
}MPU6050_AngleFiltertypedef;

typedef struct{
	int16_t AccZ;
	int16_t GyroZ;
}MPU6050_Zaxistypedef;



extern MPU6050_Datatypedef MPU6050_Data;
extern MPU6050_AngleAcctypedef AngleAcc;
extern MPU6050_AngleGyrotypedef AngleGyro;
extern MPU6050_AngleFiltertypedef AngleFilter;
extern MPU6050_Zaxistypedef Zaxis_Data;

extern int16_t AccX;
extern int16_t AccY;
extern int16_t AccZ;
extern int16_t GyroX;
extern int16_t GyroY;
extern int16_t GyroZ;


#endif