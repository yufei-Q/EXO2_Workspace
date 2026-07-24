#include "rs02_motor.h"
#include "can.h"

const RS02_LimitTypeDef RS02_DEFAULT_LIMIT = {
  -12.57f, 12.57f,
  -44.0f, 44.0f,
  -17.0f, 17.0f
};

volatile RS02_FeedbackTypeDef rs02_feedback;

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

static HAL_StatusTypeDef can_send_ext(CAN_HandleTypeDef *hcan,
                                      uint32_t ext_id,
                                      uint8_t data[8],
                                      uint8_t dlc)
{
  CAN_TxHeaderTypeDef tx_header;
  uint32_t mailbox;

  tx_header.StdId = 0;
  tx_header.ExtId = ext_id;
  tx_header.IDE = CAN_ID_EXT;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = dlc;
  tx_header.TransmitGlobalTime = DISABLE;

  return HAL_CAN_AddTxMessage(hcan, &tx_header, data, &mailbox);
}

static uint32_t private_ext_id(RS02_PrivateModeTypeDef mode,
                               uint16_t data,
                               uint8_t motor_id)
{
  return ((uint32_t)mode << 24) | ((uint32_t)data << 8) | motor_id;
}

static HAL_StatusTypeDef send_system_cmd(CAN_HandleTypeDef *hcan,
                                         uint16_t motor_id,
                                         uint8_t cmd_value,
                                         uint8_t cmd)
{
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd_value, cmd};
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

HAL_StatusTypeDef RS02_Enable(CAN_HandleTypeDef *hcan, uint16_t motor_id)
{
  return send_system_cmd(hcan, motor_id, 0xFF, 0xFC);
}

HAL_StatusTypeDef RS02_Stop(CAN_HandleTypeDef *hcan, uint16_t motor_id)
{
  return send_system_cmd(hcan, motor_id, 0xFF, 0xFD);
}

HAL_StatusTypeDef RS02_SetZero(CAN_HandleTypeDef *hcan, uint16_t motor_id)
{
  return send_system_cmd(hcan, motor_id, 0xFF, 0xFE);
}

HAL_StatusTypeDef RS02_ClearError(CAN_HandleTypeDef *hcan, uint16_t motor_id)
{
  return send_system_cmd(hcan, motor_id, 0xFF, 0xFB);
}

HAL_StatusTypeDef RS02_SetMode(CAN_HandleTypeDef *hcan,
                               uint16_t motor_id,
                               RS02_RunModeTypeDef mode)
{
  return send_system_cmd(hcan, motor_id, (uint8_t)mode, 0xFC);
}

HAL_StatusTypeDef RS02_MIT_Control(CAN_HandleTypeDef *hcan,
                                   uint16_t motor_id,
                                   float position,
                                   float velocity,
                                   float kp,
                                   float kd,
                                   float torque,
                                   const RS02_LimitTypeDef *limit)
{
  uint16_t p_uint;
  uint16_t v_uint;
  uint16_t kp_uint;
  uint16_t kd_uint;
  uint16_t t_uint;
  uint8_t data[8];

  if (limit == 0)
  {
    limit = &RS02_DEFAULT_LIMIT;
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

HAL_StatusTypeDef RS02_Position_Control(CAN_HandleTypeDef *hcan,
                                        uint16_t motor_id,
                                        float position,
                                        float velocity_limit)
{
  uint8_t data[8];

  pack_float_le(position, &data[0]);
  pack_float_le(velocity_limit, &data[4]);
  return can_send(hcan, (uint16_t)(0x100U | motor_id), data, 8);
}

HAL_StatusTypeDef RS02_Velocity_Control(CAN_HandleTypeDef *hcan,
                                        uint16_t motor_id,
                                        float velocity,
                                        float current_limit)
{
  uint8_t data[8];

  pack_float_le(velocity, &data[0]);
  pack_float_le(current_limit, &data[4]);
  return can_send(hcan, (uint16_t)(0x200U | motor_id), data, 8);
}

HAL_StatusTypeDef RS02_Private_Enable(CAN_HandleTypeDef *hcan,
                                      uint8_t motor_id,
                                      uint16_t master_id)
{
  uint8_t data[8] = {0};
  uint32_t ext_id = private_ext_id(RS02_PRIVATE_MODE_ENABLE, master_id, motor_id);

  return can_send_ext(hcan, ext_id, data, 8);
}

HAL_StatusTypeDef RS02_Private_Stop(CAN_HandleTypeDef *hcan,
                                    uint8_t motor_id,
                                    uint16_t master_id)
{
  uint8_t data[8] = {0};
  uint32_t ext_id = private_ext_id(RS02_PRIVATE_MODE_STOP, master_id, motor_id);

  return can_send_ext(hcan, ext_id, data, 8);
}

HAL_StatusTypeDef RS02_Private_MotionControl(CAN_HandleTypeDef *hcan,
                                             uint8_t motor_id,
                                             float torque,
                                             float position,
                                             float velocity,
                                             float kp,
                                             float kd,
                                             const RS02_LimitTypeDef *limit)
{
  uint16_t p_uint;
  uint16_t v_uint;
  uint16_t kp_uint;
  uint16_t kd_uint;
  uint16_t t_uint;
  uint32_t ext_id;
  uint8_t data[8];

  if (limit == 0)
  {
    limit = &RS02_DEFAULT_LIMIT;
  }

  p_uint = float_to_uint(position, limit->p_min, limit->p_max, 16);
  v_uint = float_to_uint(velocity, limit->v_min, limit->v_max, 16);
  kp_uint = float_to_uint(kp, 0.0f, 500.0f, 16);
  kd_uint = float_to_uint(kd, 0.0f, 5.0f, 16);
  t_uint = float_to_uint(torque, limit->t_min, limit->t_max, 16);
  ext_id = private_ext_id(RS02_PRIVATE_MODE_MOTION_CONTROL, t_uint, motor_id);

  data[0] = (uint8_t)(p_uint >> 8);
  data[1] = (uint8_t)p_uint;
  data[2] = (uint8_t)(v_uint >> 8);
  data[3] = (uint8_t)v_uint;
  data[4] = (uint8_t)(kp_uint >> 8);
  data[5] = (uint8_t)kp_uint;
  data[6] = (uint8_t)(kd_uint >> 8);
  data[7] = (uint8_t)kd_uint;

  return can_send_ext(hcan, ext_id, data, 8);
}

uint8_t RS02_Private_HandleFrame(const CAN_RxHeaderTypeDef *rx_header,
                                 const uint8_t data[8],
                                 uint16_t master_id)
{
  uint8_t mode;
  uint8_t host_id;
  uint16_t id_data;
  uint16_t p_uint;
  uint16_t v_uint;
  uint16_t t_uint;
  uint16_t temp_uint;
  RS02_FeedbackTypeDef feedback;

  if ((rx_header->IDE != CAN_ID_EXT) || (rx_header->DLC < 8))
  {
    return 0;
  }

  mode = (uint8_t)((rx_header->ExtId >> 24) & 0x1FU);
  id_data = (uint16_t)((rx_header->ExtId >> 8) & 0xFFFFU);
  host_id = (uint8_t)(rx_header->ExtId & 0xFFU);
  if ((mode != RS02_PRIVATE_MODE_FEEDBACK) || (host_id != (uint8_t)master_id))
  {
    return 0;
  }

  p_uint = ((uint16_t)data[0] << 8) | data[1];
  v_uint = ((uint16_t)data[2] << 8) | data[3];
  t_uint = ((uint16_t)data[4] << 8) | data[5];
  temp_uint = ((uint16_t)data[6] << 8) | data[7];

  feedback.valid = 1;
  feedback.motor_id = (uint8_t)(id_data & 0xFFU);
  feedback.state = (RS02_StateTypeDef)((id_data >> 14) & 0x03U);
  feedback.fault = ((id_data >> 8) & 0x3FU) != 0U;
  feedback.warning = 0;
  feedback.position = uint_to_float(p_uint,
                                    RS02_DEFAULT_LIMIT.p_min,
                                    RS02_DEFAULT_LIMIT.p_max,
                                    16);
  feedback.velocity = uint_to_float(v_uint,
                                    RS02_DEFAULT_LIMIT.v_min,
                                    RS02_DEFAULT_LIMIT.v_max,
                                    16);
  feedback.torque = uint_to_float(t_uint,
                                  RS02_DEFAULT_LIMIT.t_min,
                                  RS02_DEFAULT_LIMIT.t_max,
                                  16);
  feedback.temperature = (float)temp_uint * 0.1f;
  feedback.rx_tick = HAL_GetTick();

  rs02_feedback = feedback;
  return 1;
}

uint8_t RS02_HandleFrame(const CAN_RxHeaderTypeDef *rx_header,
                         const uint8_t data[8],
                         uint16_t master_id)
{
  uint16_t p_uint;
  uint16_t v_uint;
  uint16_t t_uint;
  uint16_t temp_uint;
  RS02_FeedbackTypeDef feedback;

  if ((rx_header->IDE != CAN_ID_STD) ||
      (rx_header->DLC < 8) ||
      (rx_header->StdId != master_id))
  {
    return 0;
  }

  p_uint = ((uint16_t)data[1] << 8) | data[2];
  v_uint = ((uint16_t)data[3] << 4) | (data[4] >> 4);
  t_uint = ((uint16_t)(data[4] & 0x0F) << 8) | data[5];
  temp_uint = ((uint16_t)(data[6] & 0x0F) << 8) | data[7];

  feedback.valid = 1;
  feedback.motor_id = data[0];
  feedback.state = (RS02_StateTypeDef)(data[6] >> 6);
  feedback.fault = (data[6] >> 5) & 0x01U;
  feedback.warning = (data[6] >> 4) & 0x01U;
  feedback.position = uint_to_float(p_uint,
                                    RS02_DEFAULT_LIMIT.p_min,
                                    RS02_DEFAULT_LIMIT.p_max,
                                    16);
  feedback.velocity = uint_to_float(v_uint,
                                    RS02_DEFAULT_LIMIT.v_min,
                                    RS02_DEFAULT_LIMIT.v_max,
                                    12);
  feedback.torque = uint_to_float(t_uint,
                                  RS02_DEFAULT_LIMIT.t_min,
                                  RS02_DEFAULT_LIMIT.t_max,
                                  12);
  feedback.temperature = (float)temp_uint * 0.1f;
  feedback.rx_tick = HAL_GetTick();

  rs02_feedback = feedback;
  return 1;
}
