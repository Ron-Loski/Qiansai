#include "Varibles.h"

MPU6050_Datatypedef MPU6050_Data = {0};             //六轴原始数据
MPU6050_AngleAcctypedef AngleAcc = {0};             //ACC解算的三轴角速度
MPU6050_AngleGyrotypedef AngleGyro = {0};           //三轴角速度
MPU6050_AngleFiltertypedef AngleFilter = {0};       //俯仰X,偏航Y,横滚Z
MPU6050_Zaxistypedef Zaxis_Data = {0};              //Z轴角速度+角加速度

int16_t AccX;
int16_t AccY;
int16_t AccZ;
int16_t GyroX;
int16_t GyroY;
int16_t GyroZ;

int16_t LeftPWM, RightPWM;
int16_t AvePWM, DifPWM;
float SpeedLeft, SpeedRight;
float AveSpeed, DifSpeed;

PID_t AngleLoop = {         //Z轴角度环
    .Target = 0.0f,

    .Kp = 0.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,

    .OutMax = 5.0f,
    .OutMin = -5.0f,

};

PID_t SpeedLoop = {
     .Target = 20.0f,

    .Kp = 0.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,

    .OutMax = 5.0f,
    .OutMin = -5.0f,
};

PID_t TurnLoop = {
     .Target = 0.0f,

    .Kp = 0.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,

    .OutMax = 5.0f,
    .OutMin = -5.0f,
};


















