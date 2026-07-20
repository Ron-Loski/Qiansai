#include "Varibles.h"

MPU6050_Datatypedef MPU6050_Data = {0};             //??????????
MPU6050_AngleAcctypedef AngleAcc = {0};             //ACC?????????????
MPU6050_AngleGyrotypedef AngleGyro = {0};
volatile MPU6050_AngleFiltertypedef AngleFilter = {0};       //????X,???Y,???Z
MPU6050_Zaxistypedef Zaxis_Data = {0};              //Z??????+??????

int16_t AccX;
int16_t AccY;
int16_t AccZ;
int16_t GyroX;
int16_t GyroY;
int16_t GyroZ;

uint8_t ch = 0;

volatile uint8_t Rx_flag = 0;











