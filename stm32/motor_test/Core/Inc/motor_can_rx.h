#ifndef __MOTOR_CAN_RX_H__
#define __MOTOR_CAN_RX_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

HAL_StatusTypeDef Motor_CAN_Start(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif
