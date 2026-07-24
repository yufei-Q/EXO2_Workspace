#include "damiao_motor.h"
#include "can.h"

const Damiao_LimitTypeDef DAMIAO_D4340P_DEFAULT_LIMIT = {
  -12.5f, 12.5f,
  -20.0f, 20.0f,
  -28.0f, 28.0f
};

volatile Damiao_FeedbackTypeDef damiao_feedback;

static float clampf(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static uint16_t float_to_uint(float value, float min_value, float max_value, uint8_t bits)
{
  float span = max_value - min_value;
  uint32_t max_int = (1UL << bits) - 1UL;

  value = clampf(value, min_value, max_value);
  return (uint16_t)((value - min_value) * (float)max_int / span);
}

static float uint_to_float(uint16_t value, float min_value, float max_value, uint8_t bits)
{
  float span = max_value - min_value;
  uint32_t max_int = (1UL << bits) - 1UL;

  return ((float)value) * span / (float)max_int + min_value;
}

static HAL_StatusTypeDef can_send(CAN_HandleTypeDef *hcan,
                                  uint16_t std_id,
                                  uint8_t data[8],
                                  uint8_t dlc)
{
  CAN_TxHeaderTypeDef tx_header;
  uint32_t mailbox;

  tx_header.StdId = std_id;
  tx_header.ExtId = 0;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = dlc;
  tx_header.TransmitGlobalTime = DISABLE;

  return HAL_CAN_AddTxMessage(hcan, &tx_header, data, &mailbox);
}

static HAL_StatusTypeDef send_system_cmd(CAN_HandleTypeDef *hcan,
                                         uint16_t motor_id,
                                         uint8_t cmd)
{
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
  return can_send(hcan, motor_id, data, 8);
}

static void pack_float_le(float value, uint8_t *data)
{
  union
  {
    float f;
    uint8_t b[4];
  } converter;

  converter.f = value;
  data[0] = converter.b[0];
  data[1] = converter.b[1];
  data[2] = converter.b[2];
  data[3] = converter.b[3];
}

HAL_StatusTypeDef Damiao_Enable(CAN_HandleTypeDef *hcan, uint16_t motor_id)
{
  return send_system_cmd(hcan, motor_id, 0xFC);
}

HAL_StatusTypeDef Damiao_Disable(CAN_HandleTypeDef *hcan, uint16_t motor_id)
{
  return send_system_cmd(hcan, motor_id, 0xFD);
}

HAL_StatusTypeDef Damiao_SetZero(CAN_HandleTypeDef *hcan, uint16_t motor_id)
{
  return send_system_cmd(hcan, motor_id, 0xFE);
}

HAL_StatusTypeDef Damiao_ClearError(CAN_HandleTypeDef *hcan, uint16_t motor_id)
{
  return send_system_cmd(hcan, motor_id, 0xFB);
}

HAL_StatusTypeDef Damiao_MIT_Control(CAN_HandleTypeDef *hcan,
                                      uint16_t motor_id,
                                      float position,
                                      float velocity,
                                      float kp,
                                      float kd,
                                      float torque,
                                      const Damiao_LimitTypeDef *limit)
{
  uint16_t p_uint;
  uint16_t v_uint;
  uint16_t kp_uint;
  uint16_t kd_uint;
  uint16_t t_uint;
  uint8_t data[8];

  if (limit == 0)
  {
    limit = &DAMIAO_D4340P_DEFAULT_LIMIT;
  }

  p_uint = float_to_uint(position, limit->p_min, limit->p_max, 16);
  v_uint = float_to_uint(velocity, limit->v_min, limit->v_max, 12);
  kp_uint = float_to_uint(kp, 0.0f, 500.0f, 12);
  kd_uint = float_to_uint(kd, 0.0f, 5.0f, 12);
  t_uint = float_to_uint(torque, limit->t_min, limit->t_max, 12);

  data[0] = (uint8_t)(p_uint >> 8);
  data[1] = (uint8_t)p_uint;
  data[2] = (uint8_t)(v_uint >> 4);
  data[3] = (uint8_t)(((v_uint & 0x0F) << 4) | (kp_uint >> 8));
  data[4] = (uint8_t)kp_uint;
  data[5] = (uint8_t)(kd_uint >> 4);
  data[6] = (uint8_t)(((kd_uint & 0x0F) << 4) | (t_uint >> 8));
  data[7] = (uint8_t)t_uint;

  return can_send(hcan, motor_id, data, 8);
}

HAL_StatusTypeDef Damiao_PosVel_Control(CAN_HandleTypeDef *hcan,
                                         uint16_t motor_id,
                                         float position,
                                         float velocity_limit)
{
  uint8_t data[8];

  pack_float_le(position, &data[0]);
  pack_float_le(velocity_limit, &data[4]);
  return can_send(hcan, 0x100U + motor_id, data, 8);
}

HAL_StatusTypeDef Damiao_Vel_Control(CAN_HandleTypeDef *hcan,
                                      uint16_t motor_id,
                                      float velocity)
{
  uint8_t data[8] = {0};

  pack_float_le(velocity, &data[0]);
  return can_send(hcan, 0x200U + motor_id, data, 8);
}

uint8_t Damiao_HandleFrame(const CAN_RxHeaderTypeDef *rx_header,
                           const uint8_t data[8])
{
  uint16_t p_uint;
  uint16_t v_uint;
  uint16_t t_uint;
  Damiao_FeedbackTypeDef feedback;

  if ((rx_header->IDE != CAN_ID_STD) ||
      (rx_header->DLC < 8) ||
      (rx_header->StdId != DAMIAO_MST_ID))
  {
    return 0;
  }

  p_uint = ((uint16_t)data[1] << 8) | data[2];
  v_uint = ((uint16_t)data[3] << 4) | (data[4] >> 4);
  t_uint = ((uint16_t)(data[4] & 0x0F) << 8) | data[5];

  feedback.valid = 1;
  feedback.motor_id = data[0] & 0x0F;
  feedback.status = data[0] >> 4;
  feedback.position = uint_to_float(p_uint,
                                    DAMIAO_D4340P_DEFAULT_LIMIT.p_min,
                                    DAMIAO_D4340P_DEFAULT_LIMIT.p_max,
                                    16);
  feedback.velocity = uint_to_float(v_uint,
                                    DAMIAO_D4340P_DEFAULT_LIMIT.v_min,
                                    DAMIAO_D4340P_DEFAULT_LIMIT.v_max,
                                    12);
  feedback.torque = uint_to_float(t_uint,
                                  DAMIAO_D4340P_DEFAULT_LIMIT.t_min,
                                  DAMIAO_D4340P_DEFAULT_LIMIT.t_max,
                                  12);
  feedback.mos_temp = data[6];
  feedback.rotor_temp = data[7];
  feedback.rx_tick = HAL_GetTick();

  damiao_feedback = feedback;
  return 1;
}
