#ifndef DM_D4340P_H
#define DM_D4340P_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define DM_D4340P_MOTOR_COUNT  4U

typedef enum
{
  DM_D4340P_STATUS_DISABLED = 0x0U,
  DM_D4340P_STATUS_ENABLED = 0x1U,
  DM_D4340P_STATUS_OUTPUT_ENCODER_ERROR = 0x3U,
  DM_D4340P_STATUS_SENSOR_ERROR = 0x4U,
  DM_D4340P_STATUS_MOTOR_ENCODER_ERROR = 0x5U,
  DM_D4340P_STATUS_OVER_VOLTAGE = 0x8U,
  DM_D4340P_STATUS_UNDER_VOLTAGE = 0x9U,
  DM_D4340P_STATUS_OVER_CURRENT = 0xAU,
  DM_D4340P_STATUS_MOS_OVER_TEMPERATURE = 0xBU,
  DM_D4340P_STATUS_MOTOR_OVER_TEMPERATURE = 0xCU,
  DM_D4340P_STATUS_COMMUNICATION_LOST = 0xDU,
  DM_D4340P_STATUS_OVERLOAD = 0xEU
} DM_D4340P_Status_t;

typedef struct
{
  DM_D4340P_Status_t status;
  float position;
  float velocity;
  float torque;
  uint8_t mos_temperature;
  uint8_t rotor_temperature;
} DM_D4340P_Feedback_t;

typedef struct
{
  uint8_t motor_id;
  volatile DM_D4340P_Feedback_t feedback;
} DM_D4340P_Motor_t;

extern DM_D4340P_Motor_t d4340p_motors[DM_D4340P_MOTOR_COUNT];

HAL_StatusTypeDef DM_D4340P_Init(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef DM_D4340P_Enable(const DM_D4340P_Motor_t *motor);
HAL_StatusTypeDef DM_D4340P_Disable(const DM_D4340P_Motor_t *motor);
HAL_StatusTypeDef DM_D4340P_SetZero(const DM_D4340P_Motor_t *motor);
HAL_StatusTypeDef DM_D4340P_ClearError(const DM_D4340P_Motor_t *motor);
HAL_StatusTypeDef DM_D4340P_MitControl(const DM_D4340P_Motor_t *motor,
                                      float position,
                                      float velocity,
                                      float kp,
                                      float kd,
                                      float torque);

#ifdef __cplusplus
}
#endif

#endif /* DM_D4340P_H */
