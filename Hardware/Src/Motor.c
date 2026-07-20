#include "Motor.h"
#include "Astar.h"
#include "MPU6050.h"
#include "Button.h"
#include "Blue.h"

#define MOTOR_TURN_FAST_PWM_COMPARE	40U
#define MOTOR_TURN_SLOW_PWM_COMPARE	20U
#define MOTOR_TURN_SLOW_ZONE_DEG		30.0f
#define MOTOR_TURN_ANGLE_TOLERANCE	1.0f
#define MOTOR_TURN_TIMEOUT_SAMPLES	800U
#define MOTOR_TURN_SETTLE_SAMPLES	100U
#define MOTOR_TURN_STABLE_RATE_DPS	3.0f
#define MOTOR_TURN_BRAKE_TIME_S		0.13f
#define MOTOR_TURN_MAX_BRAKE_LEAD_DEG	22.0f
#define MOTOR_TURN_CODE_45_DEG		11.0f
#define MOTOR_OBSTACLE_RESPONSE_TIMEOUT_SAMPLES	1000U		//USART2障碍物响应超时：1000 * 10ms = 10s
#define MOTOR_FORWARD_TIMEOUT_SAMPLES	1000U		//直行安全超时：1000 * 10ms = 10s

static uint8_t Motor_TurnToYaw(float target_yaw);
static float Motor_TurnCodeToAngle(float turn_code);
static void Motor_SetTurnOutput(uint8_t turn_right, uint16_t pwm_compare);
static void Motor_WaitForYawSettle(void);

void PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_3);

	TIM_TimeBaseInitStructure.TIM_Prescaler = SystemCoreClock / 100000 - 1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM2, ENABLE);

	TIM_Cmd(TIM2, ENABLE);
}

void PWM_Brake(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
}

void PWM_EnableChannel(Motor_Numtypedef Motor_Num, uint16_t Compare, Motor_Directtypedef Direct)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	if (Direct == Forward)
	{
		if (Motor_Num == Motor_Left)	//L1
		{
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			TIM_SetCompare2(TIM2, Compare);
			TIM_CCxCmd(TIM2, TIM_Channel_2, TIM_CCx_Enable);
		}
		if (Motor_Num == Motor_Right)	//L2
		{
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_SET);

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			TIM_SetCompare3(TIM2, Compare);
			TIM_CCxCmd(TIM2, TIM_Channel_3, TIM_CCx_Enable);
		}
	}
	if (Direct == Reverse)
	{
		if (Motor_Num == Motor_Left)	//L1
		{
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			TIM_SetCompare1(TIM2, Compare);
			TIM_CCxCmd(TIM2, TIM_Channel_1, TIM_CCx_Enable);
		}
		if (Motor_Num == Motor_Right)	//L2
		{
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			TIM_SetCompare4(TIM2, Compare);
			TIM_CCxCmd(TIM2, TIM_Channel_4, TIM_CCx_Enable);
		}
	}

	TIM_Cmd(TIM2, ENABLE);
}

volatile uint8_t stop_flag = 0;
uint8_t forward_dis(float distance)
{
	uint32_t start_sample = MPU6050_YawSampleCount;
	uint16_t target_cnt = (uint16_t)(distance * 12.51f * 4.0f * 30.0f / girth + 0.5f);
	uint8_t timeout = 0U;
	uint8_t button_stop = 0U;

	TIM8->CNT = 0;
	TIM10->CNT = 0;
	stop_flag = 0;
	PWM_EnableChannel(Motor_Left, 50, Forward);
	PWM_EnableChannel(Motor_Right, 50, Forward);

	while (stop_flag == 0)
	{
		if (Button_PeekEvents() & BUTTON_EVENT_DOUBLE)
		{
			button_stop = 1U;
			break;
		}
		else if (TIM8->CNT >= target_cnt && TIM10->CNT >= target_cnt)
		{
			stop_flag = 1;
		}
		else if ((uint32_t)(MPU6050_YawSampleCount - start_sample) >=
				 MOTOR_FORWARD_TIMEOUT_SAMPLES)
		{
			timeout = 1U;
			break;
		}
	}

	PWM_Brake();
	TIM8->CNT = 0;
	TIM10->CNT = 0;
	stop_flag = 0;

	if (timeout)
	{
		printf("forward timeout, target_cnt=%u\r\n", target_cnt);
		return 0U;
	}
	if (button_stop)
	{
		printf("forward stopped by double click\r\n");
		return 0U;
	}

	return 1U;
}

/*
 * MPU6050单次偏航控制：只按初始误差选择一次方向，到达目标后立即刹车并退出。
 * 停止后不再反向纠偏，避免在目标角附近来回震荡。
 */
static uint8_t Motor_TurnToYaw(float target_yaw)
{
	float current_yaw = AngleFilter.Yaw;
	float remaining_angle;
	float brake_lead;
	uint32_t start_sample = MPU6050_YawSampleCount;
	uint32_t last_debug_sample = start_sample;
	uint8_t timeout = 0U;
	uint8_t button_stop = 0U;
	uint8_t turn_right;
	uint8_t slow_mode = 0U;

	if (fabsf(target_yaw - current_yaw) <= MOTOR_TURN_ANGLE_TOLERANCE)
	{
		PWM_Brake();
		return 1U;
	}

	turn_right = (uint8_t)(target_yaw > current_yaw);
	Motor_SetTurnOutput(turn_right, MOTOR_TURN_FAST_PWM_COMPARE);

	while (1)
	{
		if (Button_PeekEvents() & BUTTON_EVENT_DOUBLE)
		{
			button_stop = 1U;
			break;
		}

		current_yaw = AngleFilter.Yaw;
		remaining_angle = turn_right ? (target_yaw - current_yaw)
								 : (current_yaw - target_yaw);

		/* 根据当前角速度估计刹车后的惯性转角，日志标定值约为0.13s。 */
		brake_lead = fabsf(MPU6050_GyroZDps) * MOTOR_TURN_BRAKE_TIME_S;
		if (brake_lead < MOTOR_TURN_ANGLE_TOLERANCE)
		{
			brake_lead = MOTOR_TURN_ANGLE_TOLERANCE;
		}
		else if (brake_lead > MOTOR_TURN_MAX_BRAKE_LEAD_DEG)
		{
			brake_lead = MOTOR_TURN_MAX_BRAKE_LEAD_DEG;
		}

		if (remaining_angle <= brake_lead)
		{
			break;
		}

		if (!slow_mode && remaining_angle <= MOTOR_TURN_SLOW_ZONE_DEG)
		{
			slow_mode = 1U;
			Motor_SetTurnOutput(turn_right, MOTOR_TURN_SLOW_PWM_COMPARE);
		}

		if ((uint32_t)(MPU6050_YawSampleCount - last_debug_sample) >= 10U)
		{
			last_debug_sample = MPU6050_YawSampleCount;
			printf("Turning yaw=%.2f rate=%.1f remain=%.2f lead=%.2f pwm=%u\r\n",
				   current_yaw, MPU6050_GyroZDps, remaining_angle, brake_lead,
				   slow_mode ? MOTOR_TURN_SLOW_PWM_COMPARE : MOTOR_TURN_FAST_PWM_COMPARE);
		}

		if ((uint32_t)(MPU6050_YawSampleCount - start_sample) >= MOTOR_TURN_TIMEOUT_SAMPLES)
		{
			timeout = 1U;
			break;
		}
	}

	PWM_Brake();
	if (!button_stop)
	{
		Motor_WaitForYawSettle();
	}
	return (uint8_t)(timeout == 0U && button_stop == 0U);
}

static void Motor_SetTurnOutput(uint8_t turn_right, uint16_t pwm_compare)
{
	if (turn_right)
	{
		PWM_EnableChannel(Motor_Left, pwm_compare, Forward);
		PWM_EnableChannel(Motor_Right, pwm_compare, Reverse);
	}
	else
	{
		PWM_EnableChannel(Motor_Left, pwm_compare, Reverse);
		PWM_EnableChannel(Motor_Right, pwm_compare, Forward);
	}
}

static void Motor_WaitForYawSettle(void)
{
	uint32_t start_sample = MPU6050_YawSampleCount;
	uint32_t last_sample = start_sample;
	uint8_t stable_samples = 0U;

	while ((uint32_t)(MPU6050_YawSampleCount - start_sample) < MOTOR_TURN_SETTLE_SAMPLES)
	{
		if (Button_PeekEvents() & BUTTON_EVENT_DOUBLE)
		{
			break;
		}

		if (MPU6050_YawSampleCount == last_sample)
		{
			continue;
		}
		last_sample = MPU6050_YawSampleCount;

		if (fabsf(MPU6050_GyroZDps) <= MOTOR_TURN_STABLE_RATE_DPS)
		{
			stable_samples++;
			if (stable_samples >= 5U)
			{
				break;
			}
		}
		else
		{
			stable_samples = 0U;
		}
	}
}

static float Motor_TurnCodeToAngle(float turn_code)
{
	return turn_code * (45.0f / MOTOR_TURN_CODE_45_DEG);
}

uint8_t Motor_TurnToAbsoluteYaw(float target_yaw)
{
	return Motor_TurnToYaw(target_yaw);
}

uint8_t turnround_right(float distance)
{
	return Motor_TurnToYaw(AngleFilter.Yaw + Motor_TurnCodeToAngle(distance));
}

uint8_t turnround_left(float distance)
{
	return Motor_TurnToYaw(AngleFilter.Yaw - Motor_TurnCodeToAngle(distance));
}

uint8_t Motor_TurnToYawCommand(uint8_t command)
{
	float target_yaw;
	uint8_t result;

	switch (command)
	{
	case 0U:
		target_yaw = 0.0f;
		break;
	case 11U:
		target_yaw = 45.0f;
		break;
	case 22U:
		target_yaw = 90.0f;
		break;
	case 33U:
		target_yaw = 135.0f;
		break;
	case 44U:
		target_yaw = 180.0f;
		break;
	case 55U:
		target_yaw = 225.0f;
		break;
	case 66U:
		target_yaw = 270.0f;
		break;
	case 77U:
		target_yaw = 315.0f;
		break;
	case 88U:
		target_yaw = 360.0f;
		break;
	default:
		return 0U;
	}

	printf("Turn cmd=%u, current=%.2f, target=%.2f\r\n",
		   command, AngleFilter.Yaw, target_yaw);
	result = Motor_TurnToYaw(target_yaw);
	printf("Turn %s, yaw=%.2f\r\n", result ? "done" : "timeout", AngleFilter.Yaw);
	return result;
}

/*                  小车移动的代码              */
uint8_t car_move(uint16_t cur_x, uint16_t cur_y, uint16_t minf_x, uint16_t minf_y)
{
	int8_t dx = (int8_t)(minf_x - cur_x);
	int8_t dy = (int8_t)(minf_y - cur_y);

	if (dx < -1 || dx > 1 || dy < -1 || dy > 1)
	{
		printf("car_move rejected non-adjacent step (%u,%u)->(%u,%u)\r\n",
			   cur_x, cur_y, minf_x, minf_y);
		return 0U;
	}

	if (dx == -1)
	{
		if (dy == 1)
		{printf("leftupmove\r\n");
			if (!turnround_left(11.0f)) return 0U;
			return forward_dis(steplen * 1.414f);
		}
		else if (dy == 0)
		{printf("leftmove\r\n");
			if (!turnround_left(22.0f)) return 0U;
			return forward_dis(steplen);
		}
		else 
		{printf("leftdownmove\r\n");
			if (!turnround_left(33.0f)) return 0U;
			return forward_dis(steplen * 1.414f);
		}
	}

	else if (dx == 0)
	{
		if (dy == 1)
		{
			printf("upmove\r\n");
			return forward_dis(steplen);
		}
		else if (dy == -1)
		{printf("downmove\r\n");
			if (!turnround_left(44.0f)) return 0U;
			return forward_dis(steplen);
		}
		else 
		{
			return 1U;
		}
	}

	else
	{
		if (dy == 1)
		{printf("rightupmove\r\n");
			if (!turnround_right(11.0f)) return 0U;
			return forward_dis(steplen * 1.414f);
		}
		else if (dy == 0)
		{printf("rightmove\r\n");
			if (!turnround_right(22.0f)) return 0U;
			return forward_dis(steplen);
		}
		else 
		{printf("rightdownmove\r\n");
			if (!turnround_right(33.0f)) return 0U;
			return forward_dis(steplen * 1.414f);
		}		
	}
}

/*               小车转向的代码              */
uint8_t car_turn(uint8_t direction, float scan_base_yaw)
{
	uint32_t response_start_sample;

	if (direction >= dir_count)
	{
		return 0U;
	}

	/* 每个方向都使用扫描起始角的绝对目标，不受continue分支影响。 */
	if (!Motor_TurnToAbsoluteYaw(scan_base_yaw + (float)direction * 45.0f))
	{
		return 0U;
	}
	if (Button_PeekEvents() != BUTTON_EVENT_NONE)
	{
		return 0U;
	}

	BlueSerial_ResetObstacleResponse();
	if ((direction % 2U) == 0U)
	{   
		USART_SendData(USART2, 0xFF);             //垂直or水平的格子发0xFF
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);    
	}

	else 
	{ 
		USART_SendData(USART2, 0xFF);              //斜向的格子发0xFF
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);   
	}
	printf("wait obsing\r\n");
	response_start_sample = MPU6050_YawSampleCount;
	while (Rx_flag == 0)
	{
		if (Button_PeekEvents() != BUTTON_EVENT_NONE)
		{
			printf("obstacle wait interrupted by button\r\n");
			return 0U;
		}

		if ((uint32_t)(MPU6050_YawSampleCount - response_start_sample) >=
			MOTOR_OBSTACLE_RESPONSE_TIMEOUT_SAMPLES)
		{
			printf("obstacle response timeout\r\n");
			return 0U;
		}
	}
	printf("waitfinish\r\n");   //等待接受数据
	return 1U;
}

/*    小车在移动之后回正（车头对准前方）的代码    */
uint8_t car_return(int8_t par_x, int8_t par_y, int8_t x0, int8_t y0)
{
    int8_t dx = x0 - par_x;
	int8_t dy = y0 - par_y;

	if (dx == -1)
	{
		if (dy == 1)
		{
			printf("turnright1hui\r\n");
			return turnround_right(11.0f);
		}

		else if (dy == 0)
		{
			printf("turnright2hui");
			return turnround_right(22.0f);
		}

		else 
		{
			printf("turnright3hui");
			return turnround_right(33.0f);
		}
	}

	else if (dx == 0)
	{
		if (dy == -1)
		{
			printf("turnright4hui");
			return turnround_right(44.0f);
		}

		else if (dy == 1)
		{
			return 1U;
		}
	}

	else 
	{
		if (dy == 1)
		{
			printf("turnleft1hui\r\n");
			return turnround_left(11.0f);
		}

		else if (dy == 0)
		{
			printf("turnleft2hui\r\n");
			return turnround_left(22.0f);
		}

		else 
		{
			printf("turnleft3hui\r\n");
			return turnround_left(33.0f);
		}
	}

	return 1U;
}
