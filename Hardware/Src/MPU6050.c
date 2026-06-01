#include "MPU6050.h"

void MPU6050_W_SCL(uint8_t BitValue)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_10, BitValue ? Bit_SET : Bit_RESET);
    Delay_Us(5);
}


void MPU6050_W_SDA(uint8_t BitValue)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_11, BitValue ? Bit_SET : Bit_RESET);
    Delay_Us(5);
}

uint8_t MPU6050_I2C_R_SDA(void)
{
    uint8_t BitValue;

    BitValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
    return BitValue;
}

void MPU6050_I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* 软件IIC使用PB10-SCL、PB11-SDA，配置为开漏输出。 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);
}

void MPU6050_I2C_Start(void)
{
    MPU6050_W_SDA(1);
    MPU6050_W_SCL(1);
    MPU6050_W_SDA(0);
    MPU6050_W_SCL(0);
}

void MPU6050_I2C_Stop(void)
{
    MPU6050_W_SDA(0);
    MPU6050_W_SCL(1);
    MPU6050_W_SDA(1);
}

void MPU6050_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;

    for(i = 0; i < 8; i++)
    {
        MPU6050_W_SDA(Byte & (0x80 >> i));
        MPU6050_W_SCL(1);
        MPU6050_W_SCL(0);
    }
}

uint8_t MPU6050_I2C_ReceiveByte(void)
{
    uint8_t i;
    uint8_t Byte = 0x00;

    MPU6050_W_SDA(1);

    for(i = 0; i < 8; i++)
    {
        MPU6050_W_SCL(1);
        if(MPU6050_I2C_R_SDA() == 1)
        {
            Byte |= (0x80 >> i);
        }
        MPU6050_W_SCL(0);
    }

    return Byte;
}

void MPU6050_I2C_SendAck(uint8_t AckBit)
{
    MPU6050_W_SDA(AckBit);
    MPU6050_W_SCL(1);
    MPU6050_W_SCL(0);
}

uint8_t MPU6050_I2C_ReceiveAck(void)
{
    uint8_t AckBit;

    MPU6050_W_SDA(1);
    MPU6050_W_SCL(1);
    AckBit = MPU6050_I2C_R_SDA();
    MPU6050_W_SCL(0);

    return AckBit;
}

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    MPU6050_I2C_Start();
    MPU6050_I2C_SendByte(MPU6050_ADDRESS);
    MPU6050_I2C_ReceiveAck();
    MPU6050_I2C_SendByte(RegAddress);
    MPU6050_I2C_ReceiveAck();
    MPU6050_I2C_SendByte(Data);
    MPU6050_I2C_ReceiveAck();
    MPU6050_I2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;

    MPU6050_I2C_Start();
    MPU6050_I2C_SendByte(MPU6050_ADDRESS);
    MPU6050_I2C_ReceiveAck();
    MPU6050_I2C_SendByte(RegAddress);
    MPU6050_I2C_ReceiveAck();

    MPU6050_I2C_Start();
    MPU6050_I2C_SendByte(MPU6050_ADDRESS | 0x01);
    MPU6050_I2C_ReceiveAck();
    Data = MPU6050_I2C_ReceiveByte();
    MPU6050_I2C_SendAck(1);  
    MPU6050_I2C_Stop();

return Data;
}

void MPU6050_Init(void)
{
    MPU6050_I2C_Init();
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07);
    MPU6050_WriteReg(MPU6050_CONFIG, 0x00);
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
					int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;
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
