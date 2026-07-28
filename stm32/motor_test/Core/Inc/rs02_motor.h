#ifndef __RS02_MOTOR_H__
#define __RS02_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define RS02_DEFAULT_MOTOR_ID   0x7FU
#define RS02_DEFAULT_MASTER_ID  0xFDU

typedef enum
{
  RS02_RUN_MODE_MIT = 0,
  RS02_RUN_MODE_POSITION = 1,
  RS02_RUN_MODE_VELOCITY = 2
} RS02_RunModeTypeDef;

typedef enum
{
  RS02_PRIVATE_MODE_GET_ID = 0,
  RS02_PRIVATE_MODE_MOTION_CONTROL = 1,
  RS02_PRIVATE_MODE_FEEDBACK = 2,
  RS02_PRIVATE_MODE_ENABLE = 3,
  RS02_PRIVATE_MODE_STOP = 4,
  RS02_PRIVATE_MODE_SET_ZERO = 6,
  RS02_PRIVATE_MODE_SET_CAN_ID = 7,
  RS02_PRIVATE_MODE_READ_PARAM = 17,
  RS02_PRIVATE_MODE_WRITE_PARAM = 18
} RS02_PrivateModeTypeDef;

typedef enum
{
  RS02_STATE_RESET = 0,
  RS02_STATE_CALIBRATION = 1,
  RS02_STATE_MOTOR = 2
} RS02_StateTypeDef;

typedef struct
{
  float p_min;
  float p_max;
  float v_min;
  float v_max;
  float t_min;
  float t_max;
} RS02_LimitTypeDef;

typedef struct
{
  uint8_t valid;
  uint8_t motor_id;
  RS02_StateTypeDef state;
  uint8_t fault;
  uint8_t warning;
  float position;
  float velocity;
  float torque;
  float temperature;
  uint32_t rx_tick;
} RS02_FeedbackTypeDef;

extern const RS02_LimitTypeDef RS02_DEFAULT_LIMIT;
extern volatile RS02_FeedbackTypeDef rs02_feedback;

HAL_StatusTypeDef RS02_Enable(CAN_HandleTypeDef *hcan, uint16_t motor_id);
HAL_StatusTypeDef RS02_Stop(CAN_HandleTypeDef *hcan, uint16_t motor_id);
HAL_StatusTypeDef RS02_SetZero(CAN_HandleTypeDef *hcan, uint16_t motor_id);
HAL_StatusTypeDef RS02_ClearError(CAN_HandleTypeDef *hcan, uint16_t motor_id);
HAL_StatusTypeDef RS02_SetMode(CAN_HandleTypeDef *hcan,
                               uint16_t motor_id,
                               RS02_RunModeTypeDef mode);
HAL_StatusTypeDef RS02_MIT_Control(CAN_HandleTypeDef *hcan,
                                   uint16_t motor_id,
                                   float position,
                                   float velocity,
                                   float kp,
                                   float kd,
                                   float torque,
                                   const RS02_LimitTypeDef *limit);
HAL_StatusTypeDef RS02_Position_Control(CAN_HandleTypeDef *hcan,
                                        uint16_t motor_id,
                                        float position,
                                        float velocity_limit);
HAL_StatusTypeDef RS02_Velocity_Control(CAN_HandleTypeDef *hcan,
                                        uint16_t motor_id,
                                        float velocity,
                                        float current_limit);
HAL_StatusTypeDef RS02_Private_Enable(CAN_HandleTypeDef *hcan,
                                      uint8_t motor_id,
                                      uint16_t master_id);
HAL_StatusTypeDef RS02_Private_Stop(CAN_HandleTypeDef *hcan,
                                    uint8_t motor_id,
                                    uint16_t master_id);
HAL_StatusTypeDef RS02_Private_MotionControl(CAN_HandleTypeDef *hcan,
                                             uint8_t motor_id,
                                             float torque,
                                             float position,
                                             float velocity,
                                             float kp,
                                             float kd,
                                             const RS02_LimitTypeDef *limit);
uint8_t RS02_Private_HandleFrame(const CAN_RxHeaderTypeDef *rx_header,
                                 const uint8_t data[8],
                                 uint16_t master_id);
uint8_t RS02_HandleFrame(const CAN_RxHeaderTypeDef *rx_header,
                         const uint8_t data[8],
                         uint16_t master_id);

#ifdef __cplusplus
}
#endif

#endif
