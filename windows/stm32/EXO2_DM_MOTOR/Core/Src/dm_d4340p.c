#include "dm_d4340p.h"

#define DM_D4340P_MASTER_ID              0x000U

#define DM_D4340P_POSITION_MAX           12.5f
#define DM_D4340P_VELOCITY_MAX           20.0f
#define DM_D4340P_TORQUE_MAX             28.0f
#define DM_D4340P_KP_MAX                 500.0f
#define DM_D4340P_KD_MAX                 5.0f

#define DM_D4340P_UINT12_MAX             4095U
#define DM_D4340P_UINT16_MAX             65535U

#define DM_D4340P_COMMAND_CLEAR_ERROR    0xFBU
#define DM_D4340P_COMMAND_ENABLE         0xFCU
#define DM_D4340P_COMMAND_DISABLE        0xFDU
#define DM_D4340P_COMMAND_SET_ZERO       0xFEU

DM_D4340P_Motor_t d4340p_motors[DM_D4340P_MOTOR_COUNT];

static CAN_HandleTypeDef *s_hcan;

static float DM_D4340P_Clamp(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }

  if (value > maximum)
  {
    return maximum;
  }

  return value;
}

static uint16_t DM_D4340P_FloatToUint(float value,
                                     float minimum,
                                     float maximum,
                                     uint32_t integer_maximum)
{
  value = DM_D4340P_Clamp(value, minimum, maximum);
  return (uint16_t)((value - minimum) * (float)integer_maximum /
                    (maximum - minimum));
}

static float DM_D4340P_UintToFloat(uint16_t value,
                                  float minimum,
                                  float maximum,
                                  uint32_t integer_maximum)
{
  return ((float)value * (maximum - minimum) / (float)integer_maximum) +
         minimum;
}

static HAL_StatusTypeDef DM_D4340P_Send(uint16_t standard_id, uint8_t data[8])
{
  CAN_TxHeaderTypeDef header;
  uint32_t mailbox;

  if (HAL_CAN_GetTxMailboxesFreeLevel(s_hcan) == 0U)
  {
    return HAL_BUSY;
  }

  header.StdId = standard_id;
  header.ExtId = 0U;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8U;
  header.TransmitGlobalTime = DISABLE;

  return HAL_CAN_AddTxMessage(s_hcan, &header, data, &mailbox);
}

static HAL_StatusTypeDef DM_D4340P_SendCommand(const DM_D4340P_Motor_t *motor, uint8_t command)
{
  uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU,
                     0xFFU, 0xFFU, 0xFFU, command};

  return DM_D4340P_Send(motor->motor_id, data);
}

HAL_StatusTypeDef DM_D4340P_Init(CAN_HandleTypeDef *hcan)
{
  if (hcan == NULL)
  {
    return HAL_ERROR;
  }

  s_hcan = hcan;
  return HAL_OK;
}

HAL_StatusTypeDef DM_D4340P_Enable(const DM_D4340P_Motor_t *motor)
{
  return DM_D4340P_SendCommand(motor, DM_D4340P_COMMAND_ENABLE);
}

HAL_StatusTypeDef DM_D4340P_Disable(const DM_D4340P_Motor_t *motor)
{
  return DM_D4340P_SendCommand(motor, DM_D4340P_COMMAND_DISABLE);
}

HAL_StatusTypeDef DM_D4340P_SetZero(const DM_D4340P_Motor_t *motor)
{
  return DM_D4340P_SendCommand(motor, DM_D4340P_COMMAND_SET_ZERO);
}

HAL_StatusTypeDef DM_D4340P_ClearError(const DM_D4340P_Motor_t *motor)
{
  return DM_D4340P_SendCommand(motor, DM_D4340P_COMMAND_CLEAR_ERROR);
}

HAL_StatusTypeDef DM_D4340P_MitControl(const DM_D4340P_Motor_t *motor,
                                      float position,
                                      float velocity,
                                      float kp,
                                      float kd,
                                      float torque)
{
  uint8_t data[8];
  uint16_t position_raw;
  uint16_t velocity_raw;
  uint16_t kp_raw;
  uint16_t kd_raw;
  uint16_t torque_raw;

  position_raw = DM_D4340P_FloatToUint(position, -DM_D4340P_POSITION_MAX, DM_D4340P_POSITION_MAX, DM_D4340P_UINT16_MAX);
  velocity_raw = DM_D4340P_FloatToUint(velocity, -DM_D4340P_VELOCITY_MAX, DM_D4340P_VELOCITY_MAX, DM_D4340P_UINT12_MAX);
  kp_raw = DM_D4340P_FloatToUint(kp, 0.0f, DM_D4340P_KP_MAX, DM_D4340P_UINT12_MAX);
  kd_raw = DM_D4340P_FloatToUint(kd, 0.0f, DM_D4340P_KD_MAX, DM_D4340P_UINT12_MAX);
  torque_raw = DM_D4340P_FloatToUint(torque, -DM_D4340P_TORQUE_MAX, DM_D4340P_TORQUE_MAX, DM_D4340P_UINT12_MAX);

  data[0] = (uint8_t)(position_raw >> 8);
  data[1] = (uint8_t)position_raw;
  data[2] = (uint8_t)(velocity_raw >> 4);
  data[3] = (uint8_t)((velocity_raw << 4) | (kp_raw >> 8));
  data[4] = (uint8_t)kp_raw;
  data[5] = (uint8_t)(kd_raw >> 4);
  data[6] = (uint8_t)((kd_raw << 4) | (torque_raw >> 8));
  data[7] = (uint8_t)torque_raw;

  return DM_D4340P_Send(motor->motor_id, data);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef header;
  DM_D4340P_Motor_t *motor;
  DM_D4340P_Feedback_t feedback;
  uint8_t data[8];
  uint8_t motor_id;
  uint16_t position_raw;
  uint16_t velocity_raw;
  uint16_t torque_raw;

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
  {
    return;
  }

  if ((header.StdId != DM_D4340P_MASTER_ID) ||
      (header.IDE != CAN_ID_STD) ||
      (header.RTR != CAN_RTR_DATA) ||
      (header.DLC != 8U))
  {
    return;
  }

  motor_id = data[0] & 0x0FU;
  if ((motor_id == 0U) || (motor_id > DM_D4340P_MOTOR_COUNT))
  {
    return;
  }

  motor = &d4340p_motors[motor_id - 1U];
  position_raw = ((uint16_t)data[1] << 8) | data[2];
  velocity_raw = ((uint16_t)data[3] << 4) | (data[4] >> 4);
  torque_raw = ((uint16_t)(data[4] & 0x0FU) << 8) | data[5];

  feedback.status = (DM_D4340P_Status_t)(data[0] >> 4);
  feedback.position = DM_D4340P_UintToFloat(position_raw, -DM_D4340P_POSITION_MAX, DM_D4340P_POSITION_MAX, DM_D4340P_UINT16_MAX);
  feedback.velocity = DM_D4340P_UintToFloat(velocity_raw, -DM_D4340P_VELOCITY_MAX, DM_D4340P_VELOCITY_MAX, DM_D4340P_UINT12_MAX);
  feedback.torque = DM_D4340P_UintToFloat(torque_raw, -DM_D4340P_TORQUE_MAX, DM_D4340P_TORQUE_MAX, DM_D4340P_UINT12_MAX);
  feedback.mos_temperature = data[6];
  feedback.rotor_temperature = data[7];
  motor->feedback = feedback;
}
