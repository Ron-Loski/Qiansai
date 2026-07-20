#ifndef __BLUE_H_
#define __BLUE_H_

extern char BlueSerial_RxPacket[];
extern uint8_t BlueSerial_RxFlag;
extern volatile uint8_t isobstacle_flag;

void BlueSerial_Init(void);
void BlueSerial_ResetObstacleResponse(void);
void BlueSerial_SendByte(uint8_t Byte);
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length);
void BlueSerial_SendString(char *String);
void BlueSerial_SendNumber(uint32_t Number, uint8_t Length);
void BlueSerial_Printf(char *format, ...);

#endif
