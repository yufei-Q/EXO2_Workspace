#include "usb_motor_comm.h"

#include "usbd_cdc_if.h"

#include <string.h>

#define USB_FRAME_HEADER_0              0xAAU
#define USB_FRAME_HEADER_1              0x55U
#define USB_FRAME_TYPE_COMMAND          0x01U
#define USB_FRAME_TYPE_FEEDBACK         0x81U

#define USB_FRAME_PAYLOAD_OFFSET        7U
#define USB_FRAME_OVERHEAD              9U
#define USB_MOTOR_COMMAND_SIZE          21U
#define USB_MOTOR_FEEDBACK_SIZE         15U
#define USB_COMMAND_PAYLOAD_SIZE        \
  (USB_MOTOR_COMMAND_SIZE * DM_D4340P_MOTOR_COUNT)
#define USB_FEEDBACK_PAYLOAD_SIZE       \
  (USB_MOTOR_FEEDBACK_SIZE * DM_D4340P_MOTOR_COUNT)
#define USB_MAX_FRAME_SIZE              \
  (USB_FRAME_OVERHEAD + USB_COMMAND_PAYLOAD_SIZE)

static uint8_t s_rx_frame[USB_MAX_FRAME_SIZE];
static uint16_t s_rx_count;
static volatile uint8_t s_command_ready;
static volatile uint8_t s_tx_busy;
static USB_MotorCommand_t s_command;
static uint16_t s_feedback_sequence;
static uint8_t s_tx_frame[USB_FRAME_OVERHEAD + USB_FEEDBACK_PAYLOAD_SIZE];

static uint16_t USB_MotorComm_Crc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc;
  uint16_t index;
  uint8_t bit;

  crc = 0xFFFFU;
  for (index = 0U; index < length; ++index)
  {
    crc ^= (uint16_t)data[index] << 8;
    for (bit = 0U; bit < 8U; ++bit)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      }
      else
      {
        crc <<= 1;
      }
    }
  }

  return crc;
}

static uint16_t USB_MotorComm_ReadUint16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void USB_MotorComm_WriteUint16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
}

static float USB_MotorComm_ReadFloat(const uint8_t *data)
{
  float value;

  memcpy(&value, data, sizeof(value));
  return value;
}

static void USB_MotorComm_WriteFloat(uint8_t *data, float value)
{
  memcpy(data, &value, sizeof(value));
}

static void USB_MotorComm_ParseCommand(const uint8_t *frame)
{
  USB_MotorCommand_t command;
  const uint8_t *payload;
  uint8_t index;

  command.sequence = USB_MotorComm_ReadUint16(&frame[5]);
  payload = &frame[USB_FRAME_PAYLOAD_OFFSET];

  for (index = 0U; index < DM_D4340P_MOTOR_COUNT; ++index)
  {
    command.motor[index].flags = payload[0];
    command.motor[index].position = USB_MotorComm_ReadFloat(&payload[1]);
    command.motor[index].velocity = USB_MotorComm_ReadFloat(&payload[5]);
    command.motor[index].kp = USB_MotorComm_ReadFloat(&payload[9]);
    command.motor[index].kd = USB_MotorComm_ReadFloat(&payload[13]);
    command.motor[index].torque = USB_MotorComm_ReadFloat(&payload[17]);
    payload += USB_MOTOR_COMMAND_SIZE;
  }

  s_command = command;
  s_command_ready = 1U;
}

static void USB_MotorComm_ProcessFrame(const uint8_t *frame,
                                       uint16_t frame_length)
{
  uint16_t payload_length;
  uint16_t received_crc;
  uint16_t calculated_crc;

  payload_length = USB_MotorComm_ReadUint16(&frame[3]);
  received_crc = USB_MotorComm_ReadUint16(&frame[frame_length - 2U]);
  calculated_crc = USB_MotorComm_Crc16(&frame[2],
                                       (uint16_t)(payload_length + 5U));

  if ((received_crc == calculated_crc) &&
      (frame[2] == USB_FRAME_TYPE_COMMAND) &&
      (payload_length == USB_COMMAND_PAYLOAD_SIZE))
  {
    USB_MotorComm_ParseCommand(frame);
  }
}

void USB_MotorComm_Receive(const uint8_t *data, uint32_t length)
{
  uint32_t index;
  uint16_t payload_length;
  uint16_t frame_length;
  uint8_t byte;

  for (index = 0U; index < length; ++index)
  {
    byte = data[index];

    if (s_rx_count == 0U)
    {
      if (byte == USB_FRAME_HEADER_0)
      {
        s_rx_frame[s_rx_count++] = byte;
      }
      continue;
    }

    if (s_rx_count == 1U)
    {
      if (byte == USB_FRAME_HEADER_1)
      {
        s_rx_frame[s_rx_count++] = byte;
      }
      else
      {
        s_rx_count = (byte == USB_FRAME_HEADER_0) ? 1U : 0U;
      }
      continue;
    }

    if (s_rx_count >= USB_MAX_FRAME_SIZE)
    {
      s_rx_count = 0U;
      continue;
    }

    s_rx_frame[s_rx_count++] = byte;

    if (s_rx_count >= 5U)
    {
      payload_length = USB_MotorComm_ReadUint16(&s_rx_frame[3]);
      if (payload_length > USB_COMMAND_PAYLOAD_SIZE)
      {
        s_rx_count = 0U;
        continue;
      }

      frame_length = (uint16_t)(payload_length + USB_FRAME_OVERHEAD);
      if (s_rx_count == frame_length)
      {
        USB_MotorComm_ProcessFrame(s_rx_frame, frame_length);
        s_rx_count = 0U;
      }
    }
  }
}

uint8_t USB_MotorComm_GetCommand(USB_MotorCommand_t *command)
{
  uint32_t interrupt_state;

  if ((command == NULL) || (s_command_ready == 0U))
  {
    return 0U;
  }

  interrupt_state = __get_PRIMASK();
  __disable_irq();
  *command = s_command;
  s_command_ready = 0U;
  if (interrupt_state == 0U)
  {
    __enable_irq();
  }

  return 1U;
}

uint8_t USB_MotorComm_SendFeedback(void)
{
  DM_D4340P_Feedback_t feedback[DM_D4340P_MOTOR_COUNT];
  uint8_t *payload;
  uint16_t crc;
  uint32_t interrupt_state;
  uint8_t index;

  if (s_tx_busy != 0U)
  {
    return USBD_BUSY;
  }

  interrupt_state = __get_PRIMASK();
  __disable_irq();
  for (index = 0U; index < DM_D4340P_MOTOR_COUNT; ++index)
  {
    feedback[index] = d4340p_motors[index].feedback;
  }
  if (interrupt_state == 0U)
  {
    __enable_irq();
  }

  s_tx_frame[0] = USB_FRAME_HEADER_0;
  s_tx_frame[1] = USB_FRAME_HEADER_1;
  s_tx_frame[2] = USB_FRAME_TYPE_FEEDBACK;
  USB_MotorComm_WriteUint16(&s_tx_frame[3], USB_FEEDBACK_PAYLOAD_SIZE);
  USB_MotorComm_WriteUint16(&s_tx_frame[5], s_feedback_sequence++);

  payload = &s_tx_frame[USB_FRAME_PAYLOAD_OFFSET];
  for (index = 0U; index < DM_D4340P_MOTOR_COUNT; ++index)
  {
    payload[0] = (uint8_t)feedback[index].status;
    USB_MotorComm_WriteFloat(&payload[1], feedback[index].position);
    USB_MotorComm_WriteFloat(&payload[5], feedback[index].velocity);
    USB_MotorComm_WriteFloat(&payload[9], feedback[index].torque);
    payload[13] = feedback[index].mos_temperature;
    payload[14] = feedback[index].rotor_temperature;
    payload += USB_MOTOR_FEEDBACK_SIZE;
  }

  crc = USB_MotorComm_Crc16(&s_tx_frame[2], (uint16_t)(USB_FEEDBACK_PAYLOAD_SIZE + 5U));
  USB_MotorComm_WriteUint16(&s_tx_frame[USB_FRAME_PAYLOAD_OFFSET + USB_FEEDBACK_PAYLOAD_SIZE], crc);

  s_tx_busy = 1U;
  if (CDC_Transmit_FS(s_tx_frame, sizeof(s_tx_frame)) == USBD_OK)
  {
    return USBD_OK;
  }

  s_tx_busy = 0U;
  return USBD_BUSY;
}

void USB_MotorComm_TransmitComplete(void)
{
  s_tx_busy = 0U;
}
