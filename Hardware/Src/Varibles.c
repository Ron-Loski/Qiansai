#include "Varibles.h"

MPU6050_Datatypedef MPU6050_Data = {0};          // MPU6050 原始加速度和陀螺仪数据。
MPU6050_AngleAcctypedef AngleAcc = {0};          // 由加速度计计算得到的角度。
MPU6050_AngleGyrotypedef AngleGyro = {0};        // 由陀螺仪积分得到的角度。
MPU6050_AngleFiltertypedef AngleFilter = {0};    // 互补滤波后的姿态角，Roll/Pitch/Yaw。
MPU6050_Zaxistypedef Zaxis_Data = {0};           // Z 轴相关原始数据缓存。

int16_t AccX;
int16_t AccY;
int16_t AccZ;
int16_t GyroX;
int16_t GyroY;
int16_t GyroZ;

uint8_t ch = 0;

volatile uint8_t Rx_flag = 0;