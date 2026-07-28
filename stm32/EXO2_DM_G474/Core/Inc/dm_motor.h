#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include <main.h>

#define DM_D4340P_MOTOR_COUNT  4U
#define DM_D4310P_MOTOR_COUNT  3U
#define DM_MOTOR_COUNT          7U

typedef enum
{
  DM_MOTOR_MODEL_D4340P = 0U,
  DM_MOTOR_MODEL_D4310P
} DM_MotorModel_t;

typedef enum
{
  DM_MOTOR_STATUS_DISABLED = 0x0U,
  DM_MOTOR_STATUS_ENABLED = 0x1U,
  DM_MOTOR_STATUS_OUTPUT_ENCODER_ERROR = 0x3U,
  DM_MOTOR_STATUS_SENSOR_ERROR = 0x4U,
  DM_MOTOR_STATUS_MOTOR_ENCODER_ERROR = 0x5U,
  DM_MOTOR_STATUS_OVER_VOLTAGE = 0x8U,
  DM_MOTOR_STATUS_UNDER_VOLTAGE = 0x9U,
  DM_MOTOR_STATUS_OVER_CURRENT = 0xAU,
  DM_MOTOR_STATUS_MOS_OVER_TEMPERATURE = 0xBU,
  DM_MOTOR_STATUS_MOTOR_OVER_TEMPERATURE = 0xCU,
  DM_MOTOR_STATUS_COMMUNICATION_LOST = 0xDU,
  DM_MOTOR_STATUS_OVERLOAD = 0xEU
} DM_MotorStatus_t;

typedef struct
{
  DM_MotorStatus_t status;
  float position;
  float velocity;
  float torque;
  uint8_t mos_temperature;
  uint8_t rotor_temperature;
} DM_MotorFeedback_t;

typedef struct
{
  uint8_t motor_id;
  DM_MotorModel_t model;
  volatile DM_MotorFeedback_t feedback;
} DM_Motor_t;

extern DM_Motor_t dm_motors[DM_MOTOR_COUNT];

HAL_StatusTypeDef DM_Motor_Init(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef DM_Motor_Enable(const DM_Motor_t *motor);
HAL_StatusTypeDef DM_Motor_Disable(const DM_Motor_t *motor);
HAL_StatusTypeDef DM_Motor_SetZero(const DM_Motor_t *motor);
HAL_StatusTypeDef DM_Motor_ClearError(const DM_Motor_t *motor);
HAL_StatusTypeDef DM_Motor_MitControl(const DM_Motor_t *motor,
                                      float position,
                                      float velocity,
                                      float kp,
                                      float kd,
                                      float torque);
HAL_StatusTypeDef DM_Motor_CanAnalyzerTestSend(uint32_t counter);

#endif /* DM_MOTOR_H */
