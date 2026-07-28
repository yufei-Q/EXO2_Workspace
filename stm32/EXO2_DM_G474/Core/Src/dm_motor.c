#include <dm_motor.h>

#define DM_MOTOR_MASTER_ID              0x000U

/* These values must match PMAX, VMAX and TMAX inside each motor. */
#define DM_D4340P_POSITION_MAX           12.5f
#define DM_D4340P_VELOCITY_MAX           20.0f
#define DM_D4340P_TORQUE_MAX             28.0f

#define DM_D4310P_POSITION_MAX           12.5f
#define DM_D4310P_VELOCITY_MAX           30.0f
#define DM_D4310P_TORQUE_MAX             12.5f

#define DM_MOTOR_KP_MAX                  500.0f
#define DM_MOTOR_KD_MAX                  5.0f

#define DM_MOTOR_UINT12_MAX              4095U
#define DM_MOTOR_UINT16_MAX              65535U

#define DM_MOTOR_COMMAND_CLEAR_ERROR     0xFBU
#define DM_MOTOR_COMMAND_ENABLE          0xFCU
#define DM_MOTOR_COMMAND_DISABLE         0xFDU
#define DM_MOTOR_COMMAND_SET_ZERO        0xFEU

#define CAN_TEST_PERIODIC_TX_ID           0x100U
#define CAN_TEST_REQUEST_ID               0x200U
#define CAN_TEST_RESPONSE_ID              0x201U

typedef struct
{
  float position_max;
  float velocity_max;
  float torque_max;
} DM_MotorLimits_t;

DM_Motor_t dm_motors[DM_MOTOR_COUNT] =
{
  {1U, DM_MOTOR_MODEL_D4340P, {DM_MOTOR_STATUS_DISABLED, 0.0f, 0.0f, 0.0f, 0U, 0U}},
  {2U, DM_MOTOR_MODEL_D4340P, {DM_MOTOR_STATUS_DISABLED, 0.0f, 0.0f, 0.0f, 0U, 0U}},
  {3U, DM_MOTOR_MODEL_D4340P, {DM_MOTOR_STATUS_DISABLED, 0.0f, 0.0f, 0.0f, 0U, 0U}},
  {4U, DM_MOTOR_MODEL_D4340P, {DM_MOTOR_STATUS_DISABLED, 0.0f, 0.0f, 0.0f, 0U, 0U}},
  {5U, DM_MOTOR_MODEL_D4310P, {DM_MOTOR_STATUS_DISABLED, 0.0f, 0.0f, 0.0f, 0U, 0U}},
  {6U, DM_MOTOR_MODEL_D4310P, {DM_MOTOR_STATUS_DISABLED, 0.0f, 0.0f, 0.0f, 0U, 0U}},
  {7U, DM_MOTOR_MODEL_D4310P, {DM_MOTOR_STATUS_DISABLED, 0.0f, 0.0f, 0.0f, 0U, 0U}}
};

static FDCAN_HandleTypeDef *s_hfdcan;
static volatile uint8_t s_can_test_enabled = CAN_ANALYZER_LOOPBACK_TEST;

static const DM_MotorLimits_t s_motor_limits[] =
{
  {DM_D4340P_POSITION_MAX, DM_D4340P_VELOCITY_MAX, DM_D4340P_TORQUE_MAX},
  {DM_D4310P_POSITION_MAX, DM_D4310P_VELOCITY_MAX, DM_D4310P_TORQUE_MAX}
};

static float DM_Motor_Clamp(float value, float minimum, float maximum)
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

static uint16_t DM_Motor_FloatToUint(float value,
                                     float minimum,
                                     float maximum,
                                     uint32_t integer_maximum)
{
  value = DM_Motor_Clamp(value, minimum, maximum);
  return (uint16_t)((value - minimum) * (float)integer_maximum /
                    (maximum - minimum));
}

static float DM_Motor_UintToFloat(uint16_t value,
                                  float minimum,
                                  float maximum,
                                  uint32_t integer_maximum)
{
  return ((float)value * (maximum - minimum) / (float)integer_maximum) +
         minimum;
}

static HAL_StatusTypeDef DM_Motor_Send(uint16_t standard_id, uint8_t data[8])
{
  FDCAN_TxHeaderTypeDef header;

  if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) == 0U)
  {
    return HAL_BUSY;
  }

  header.Identifier = standard_id;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = FDCAN_DLC_BYTES_8;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_ON;
  header.FDFormat = FDCAN_FD_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0U;

  return HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan, &header, data);
}

static HAL_StatusTypeDef DM_Motor_SendCommand(const DM_Motor_t *motor,
                                              uint8_t command)
{
  uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU,
                     0xFFU, 0xFFU, 0xFFU, command};

  return DM_Motor_Send(motor->motor_id, data);
}

HAL_StatusTypeDef DM_Motor_Init(FDCAN_HandleTypeDef *hfdcan)
{
  if (hfdcan == NULL)
  {
    return HAL_ERROR;
  }

  s_hfdcan = hfdcan;
  return HAL_OK;
}

HAL_StatusTypeDef DM_Motor_Enable(const DM_Motor_t *motor)
{
  return DM_Motor_SendCommand(motor, DM_MOTOR_COMMAND_ENABLE);
}

HAL_StatusTypeDef DM_Motor_Disable(const DM_Motor_t *motor)
{
  return DM_Motor_SendCommand(motor, DM_MOTOR_COMMAND_DISABLE);
}

HAL_StatusTypeDef DM_Motor_SetZero(const DM_Motor_t *motor)
{
  return DM_Motor_SendCommand(motor, DM_MOTOR_COMMAND_SET_ZERO);
}

HAL_StatusTypeDef DM_Motor_ClearError(const DM_Motor_t *motor)
{
  return DM_Motor_SendCommand(motor, DM_MOTOR_COMMAND_CLEAR_ERROR);
}

HAL_StatusTypeDef DM_Motor_MitControl(const DM_Motor_t *motor,
                                      float position,
                                      float velocity,
                                      float kp,
                                      float kd,
                                      float torque)
{
  const DM_MotorLimits_t *limits;
  uint8_t data[8];
  uint16_t position_raw;
  uint16_t velocity_raw;
  uint16_t kp_raw;
  uint16_t kd_raw;
  uint16_t torque_raw;

  limits = &s_motor_limits[motor->model];
  position_raw = DM_Motor_FloatToUint(position,
                                     -limits->position_max,
                                     limits->position_max,
                                     DM_MOTOR_UINT16_MAX);
  velocity_raw = DM_Motor_FloatToUint(velocity,
                                     -limits->velocity_max,
                                     limits->velocity_max,
                                     DM_MOTOR_UINT12_MAX);
  kp_raw = DM_Motor_FloatToUint(kp, 0.0f, DM_MOTOR_KP_MAX,
                               DM_MOTOR_UINT12_MAX);
  kd_raw = DM_Motor_FloatToUint(kd, 0.0f, DM_MOTOR_KD_MAX,
                               DM_MOTOR_UINT12_MAX);
  torque_raw = DM_Motor_FloatToUint(torque,
                                   -limits->torque_max,
                                   limits->torque_max,
                                   DM_MOTOR_UINT12_MAX);

  data[0] = (uint8_t)(position_raw >> 8);
  data[1] = (uint8_t)position_raw;
  data[2] = (uint8_t)(velocity_raw >> 4);
  data[3] = (uint8_t)((velocity_raw << 4) | (kp_raw >> 8));
  data[4] = (uint8_t)kp_raw;
  data[5] = (uint8_t)(kd_raw >> 4);
  data[6] = (uint8_t)((kd_raw << 4) | (torque_raw >> 8));
  data[7] = (uint8_t)torque_raw;

  return DM_Motor_Send(motor->motor_id, data);
}

HAL_StatusTypeDef DM_Motor_CanAnalyzerTestSend(uint32_t counter)
{
  uint8_t data[8];

  data[0] = 0xA5U;
  data[1] = 0x5AU;
  data[2] = (uint8_t)counter;
  data[3] = (uint8_t)(counter >> 8);
  data[4] = (uint8_t)(counter >> 16);
  data[5] = (uint8_t)(counter >> 24);
  data[6] = 0x47U;
  data[7] = 0x34U;

  return DM_Motor_Send(CAN_TEST_PERIODIC_TX_ID, data);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t rx_fifo0_interrupts)
{
  FDCAN_RxHeaderTypeDef header;
  DM_Motor_t *motor;
  DM_MotorFeedback_t feedback;
  const DM_MotorLimits_t *limits;
  uint8_t data[8];
  uint8_t motor_id;
  uint16_t position_raw;
  uint16_t velocity_raw;
  uint16_t torque_raw;

  if ((rx_fifo0_interrupts & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
  {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) != 0U)
  {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                              &header, data) != HAL_OK)
    {
      return;
    }

    if (s_can_test_enabled != 0U)
    {
      if ((header.Identifier == CAN_TEST_REQUEST_ID) &&
          (header.IdType == FDCAN_STANDARD_ID) &&
          (header.RxFrameType == FDCAN_DATA_FRAME) &&
          (header.DataLength == FDCAN_DLC_BYTES_8))
      {
        (void)DM_Motor_Send(CAN_TEST_RESPONSE_ID, data);
      }
      continue;
    }

    if ((header.Identifier != DM_MOTOR_MASTER_ID) ||
        (header.IdType != FDCAN_STANDARD_ID) ||
        (header.RxFrameType != FDCAN_DATA_FRAME) ||
        (header.DataLength != FDCAN_DLC_BYTES_8))
    {
      continue;
    }

    motor_id = data[0] & 0x0FU;
    if ((motor_id == 0U) || (motor_id > DM_MOTOR_COUNT))
    {
      continue;
    }

    motor = &dm_motors[motor_id - 1U];
    limits = &s_motor_limits[motor->model];
    position_raw = ((uint16_t)data[1] << 8) | data[2];
    velocity_raw = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    torque_raw = ((uint16_t)(data[4] & 0x0FU) << 8) | data[5];

    feedback.status = (DM_MotorStatus_t)(data[0] >> 4);
    feedback.position = DM_Motor_UintToFloat(position_raw,
                                             -limits->position_max,
                                             limits->position_max,
                                             DM_MOTOR_UINT16_MAX);
    feedback.velocity = DM_Motor_UintToFloat(velocity_raw,
                                             -limits->velocity_max,
                                             limits->velocity_max,
                                             DM_MOTOR_UINT12_MAX);
    feedback.torque = DM_Motor_UintToFloat(torque_raw,
                                           -limits->torque_max,
                                           limits->torque_max,
                                           DM_MOTOR_UINT12_MAX);
    feedback.mos_temperature = data[6];
    feedback.rotor_temperature = data[7];
    motor->feedback = feedback;
  }
}
