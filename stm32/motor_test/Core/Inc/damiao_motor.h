#ifndef __DAMIAO_MOTOR_H__
#define __DAMIAO_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define DAMIAO_MST_ID  0x00U
#define DAMIAO_MOTOR_ID  0x01U

typedef struct
{
  float p_min;
  float p_max;
  float v_min;
  float v_max;
  float t_min;
  float t_max;
} Damiao_LimitTypeDef;

typedef struct
{
  uint8_t valid;
  uint8_t motor_id;
  uint8_t status;
  float position;
  float velocity;
  float torque;
  uint8_t mos_temp;
  uint8_t rotor_temp;
  uint32_t rx_tick;
} Damiao_FeedbackTypeDef;

extern const Damiao_LimitTypeDef DAMIAO_D4340P_DEFAULT_LIMIT;
extern volatile Damiao_FeedbackTypeDef damiao_feedback;

HAL_StatusTypeDef Damiao_Enable(CAN_HandleTypeDef *hcan, uint16_t motor_id);
HAL_StatusTypeDef Damiao_Disable(CAN_HandleTypeDef *hcan, uint16_t motor_id);
HAL_StatusTypeDef Damiao_SetZero(CAN_HandleTypeDef *hcan, uint16_t motor_id);
HAL_StatusTypeDef Damiao_ClearError(CAN_HandleTypeDef *hcan, uint16_t motor_id);
HAL_StatusTypeDef Damiao_MIT_Control(CAN_HandleTypeDef *hcan,
                                      uint16_t motor_id,
                                      float position,
                                      float velocity,
                                      float kp,
                                      float kd,
                                      float torque,
                                      const Damiao_LimitTypeDef *limit);
HAL_StatusTypeDef Damiao_PosVel_Control(CAN_HandleTypeDef *hcan,
                                         uint16_t motor_id,
                                         float position,
                                         float velocity_limit);
HAL_StatusTypeDef Damiao_Vel_Control(CAN_HandleTypeDef *hcan,
                                      uint16_t motor_id,
                                      float velocity);
uint8_t Damiao_HandleFrame(const CAN_RxHeaderTypeDef *rx_header,
                           const uint8_t data[8]);

#ifdef __cplusplus
}
#endif

#endif
