#ifndef ARGOS_PLATFORM_H
#define ARGOS_PLATFORM_H

#include "main.h"

#define MPU_INT_Pin         GPIO_PIN_10
#define MPU_INT_GPIO_Port   GPIOB
#define TOF_LEFT_XSHUT_Pin   GPIO_PIN_8
#define TOF_LEFT_XSHUT_Port  GPIOA
#define TOF_LEFT_INT_Pin     GPIO_PIN_7
#define TOF_LEFT_INT_Port    GPIOC
#define TOF_RIGHT_XSHUT_Pin  GPIO_PIN_9
#define TOF_RIGHT_XSHUT_Port GPIOA
#define TOF_RIGHT_INT_Pin    GPIO_PIN_10
#define TOF_RIGHT_INT_Port   GPIOA
#define LEFT_MOTOR_RPWM_Pin  GPIO_PIN_6
#define LEFT_MOTOR_LPWM_Pin  GPIO_PIN_8
#define LEFT_MOTOR_Port      GPIOC
#define RIGHT_MOTOR_RPWM_Pin GPIO_PIN_0
#define RIGHT_MOTOR_LPWM_Pin GPIO_PIN_1
#define RIGHT_MOTOR_Port     GPIOA
#define RIGHT_ENC_A_Pin      GPIO_PIN_2
#define RIGHT_ENC_B_Pin      GPIO_PIN_3
#define RIGHT_ENC_Port       GPIOC
#define LEFT_ENC_A_Pin       GPIO_PIN_4
#define LEFT_ENC_B_Pin       GPIO_PIN_5
#define LEFT_ENC_Port        GPIOC

void Argos_Platform_Init(void);
void Argos_Platform_Task(void);

#endif /* ARGOS_PLATFORM_H */
