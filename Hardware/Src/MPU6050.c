#include "MPU6050.h"
#include "ch32v30x_gpio.h"

/**
 * @brief 软件 I2C 写 SCL 时钟线电平。
 * @param BitValue 0 输出低电平，非 0 输出高电平。
 * @return 无。
 * @details 通过 PB10 控制 SCL，并延时约 10us 形成软件 I2C 时序。
 */
void MyI2C_W_SCL(uint8_t BitValue)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)BitValue);
    Delay_Us(10);
}

/**
 * @brief 软件 I2C 写 SDA 数据线电平。
 * @param BitValue 0 输出低电平，非 0 输出高电平。
 * @return 无。
 * @details 通过 PB11 控制 SDA，并延时约 10us 保证数据稳定。
 */
void MyI2C_W_SDA(uint8_t BitValue)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_11, (BitAction)BitValue);
    Delay_Us(10);
}

/**
 * @brief 读取软件 I2C SDA 数据线电平。
 * @param 无。
 * @return 0 表示低电平，1 表示高电平。
 * @details 用于接收 MPU6050 返回的数据位和 ACK 位。
 */
uint8_t MyI2C_R_SDA(void)
{
    uint8_t BitValue;

    BitValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
    Delay_Us(10);

    return BitValue;
}

/**
 * @brief 初始化软件 I2C GPIO。
 * @param 无。
 * @return 无。
 * @details PB10/PB11 配置为开漏输出，并默认释放为高电平，供 MPU6050 通信使用。
 */
void MyI2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);
}

/**
 * @brief 产生软件 I2C 起始信号。
 * @param 无。
 * @return 无。
 * @details 在 SCL 为高电平时将 SDA 从高拉低，随后拉低 SCL，符合 I2C START 时序。
 */
void MyI2C_Start(void)
{
    MyI2C_W_SDA(1);
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(0);
    MyI2C_W_SCL(0);
}

/**
 * @brief 产生软件 I2C 停止信号。
 * @param 无。
 * @return 无。
 * @details 在 SCL 为高电平时将 SDA 从低释放为高，符合 I2C STOP 时序。
 */
void MyI2C_Stop(void)
{
    MyI2C_W_SDA(0);
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(1);
}

/**
 * @brief 软件 I2C 发送一个字节。
 * @param Byte 要发送的 8 位数据。
 * @return 无。
 * @details 从高位到低位依次写 SDA，并翻转 SCL 让从机采样。
 * @note 发送后通常需要调用 MyI2C_ReceiveAck() 读取从机应答。
 */
void MyI2C_SendByte(uint8_t Byte)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        MyI2C_W_SDA(Byte & (0x80 >> i));
        MyI2C_W_SCL(1);
        MyI2C_W_SCL(0);
    }
}

/**
 * @brief 软件 I2C 接收一个字节。
 * @param 无。
 * @return 接收到的 8 位数据。
 * @details 释放 SDA 后依次拉高 SCL 读取每一位，从高位拼接成字节。
 */
uint8_t MyI2C_ReceiveByte(void)
{
    uint8_t i;
    uint8_t Byte = 0x00;

    MyI2C_W_SDA(1);
    for (i = 0; i < 8; i++)
    {
        MyI2C_W_SCL(1);
        if (MyI2C_R_SDA() == 1)
        {
            Byte |= (0x80 >> i);
        }
        MyI2C_W_SCL(0);
    }

    return Byte;
}

/**
 * @brief 软件 I2C 发送 ACK/NACK。
 * @param AckBit 0 表示 ACK，1 表示 NACK。
 * @return 无。
 * @details 主机读取完从机数据后，通过本函数通知是否继续读取。
 */
void MyI2C_SendAck(uint8_t AckBit)
{
    MyI2C_W_SDA(AckBit);
    MyI2C_W_SCL(1);
    MyI2C_W_SCL(0);
}

/**
 * @brief 软件 I2C 接收从机 ACK。
 * @param 无。
 * @return 0 表示收到 ACK，1 表示未应答/NACK。
 * @details 主机发送地址或数据后调用，用来确认 MPU6050 是否响应。
 */
uint8_t MyI2C_ReceiveAck(void)
{
    uint8_t AckBit;

    MyI2C_W_SDA(1);
    MyI2C_W_SCL(1);
    AckBit = MyI2C_R_SDA();
    MyI2C_W_SCL(0);

    return AckBit;
}

#define MPU6050_ADDRESS     0xD0    // MPU6050 7 位地址左移一位后的写地址。

/**
 * @brief 写 MPU6050 指定寄存器。
 * @param RegAddress 寄存器地址。
 * @param Data 要写入的 8 位数据。
 * @return 无。
 * @details 通过软件 I2C 发送设备写地址、寄存器地址和数据，最后发送 STOP。
 * @note MPU6050_Init() 使用本函数配置采样率、滤波器和量程。
 */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(Data);
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}

/**
 * @brief 读取 MPU6050 指定寄存器。
 * @param RegAddress 寄存器地址。
 * @return 读出的 8 位寄存器值。
 * @details 先写寄存器地址，再重复起始并切换到读方向，读取一个字节后发送 NACK 和 STOP。
 */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
    MyI2C_ReceiveAck();
    Data = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);
    MyI2C_Stop();

    return Data;
}

/**
 * @brief 初始化 MPU6050。
 * @param 无。
 * @return 无。
 * @details 初始化软件 I2C 后，配置电源管理、采样分频、低通滤波、陀螺仪量程和加速度量程。
 * @note 使用 MPU6050_GetRoll() 前必须先调用本函数。
 */
void MPU6050_Init(void)
{
    MyI2C_Init();

    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);     // 选择 X 轴陀螺仪作为时钟源并唤醒设备。
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);     // 使能全部轴。
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);     // 设置采样分频。
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);          // 设置数字低通滤波器。
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);    // 陀螺仪量程设置为 +/-2000 deg/s。
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);   // 加速度计量程设置为 +/-16g。
}

/**
 * @brief 读取 MPU6050 设备 ID。
 * @param 无。
 * @return WHO_AM_I 寄存器值。
 * @details 可用于检查 I2C 连线和 MPU6050 是否正常响应。
 */
uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

/**
 * @brief 一次性读取 MPU6050 三轴加速度和三轴陀螺仪原始数据。
 * @param AccX/AccY/AccZ 加速度原始值输出指针。
 * @param GyroX/GyroY/GyroZ 陀螺仪原始值输出指针。
 * @return 无。
 * @details 分别读取高低字节并合成为 int16_t 原始数据，供 MPU6050_GetRoll() 滤波使用。
 */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t DataH;
    uint8_t DataL;

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
    *AccX = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
    *AccY = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
    *AccZ = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
    *GyroX = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
    *GyroY = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
    *GyroZ = (DataH << 8) | DataL;
}

/**
 * @brief 读取 MPU6050 并更新 AngleFilter.Roll。
 * @param 无。
 * @return 无。
 * @details 先调用 MPU6050_GetData() 更新 MPU6050_Data，再用加速度角 AngleAcc.AccZ 和 GyroZ 积分做互补滤波。
 * @note 当前函数名为 Roll，但内部使用 GyroZ 和 AccX/AccY 参与计算；若用于小车朝向，应结合实车标定确认轴向是否正确。
 */
void MPU6050_GetRoll(void)
{
    MPU6050_GetData(&MPU6050_Data.AccX, &MPU6050_Data.AccY, &MPU6050_Data.AccZ,
                    &MPU6050_Data.GyroX, &MPU6050_Data.GyroY, &MPU6050_Data.GyroZ);

    AngleAcc.AccZ = -atan2(MPU6050_Data.AccX, MPU6050_Data.AccY) / 3.14159f * 180.0f;
    AngleAcc.AccZ += 10;

    float Alpha = 0.1f;
    AngleGyro.GyroZ = AngleFilter.Roll + MPU6050_Data.GyroZ / 32768.0f * 2000.0f * 0.01f;
    AngleFilter.Roll = Alpha * AngleAcc.AccZ + (1.0f - Alpha) * AngleGyro.GyroZ;
}