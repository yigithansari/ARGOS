#ifndef ARGOS_OLED_H
#define ARGOS_OLED_H

#include "main.h"

void Argos_Oled_Init(void);
void Argos_Oled_Task(void);
void Argos_Oled_SetNetwork(uint8_t connected);
void Argos_Oled_SetNetworkInfo(const char *mode, const char *address);
void Argos_Oled_SetImu(uint8_t ok);
void Argos_Oled_SetTof(uint16_t left_mm, uint8_t left_ok, uint16_t right_mm, uint8_t right_ok);
void Argos_Oled_SetMotors(int16_t left_percent, int16_t right_percent, const char *safety);

#endif /* ARGOS_OLED_H */
