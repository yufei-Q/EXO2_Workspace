#ifndef USB_MOTOR_COMM_H
#define USB_MOTOR_COMM_H

#include <main.h>
#include <dm_motor.h>

#define USB_MOTOR_FLAG_ENABLE       0x01U
#define USB_MOTOR_FLAG_CLEAR_ERROR  0x02U
#define USB_MOTOR_FLAG_SET_ZERO     0x04U

typedef struct
{
  uint8_t flags;
  float position;
  float velocity;
  float kp;
  float kd;
  float torque;
} USB_MotorTarget_t;

typedef struct
{
  uint16_t sequence;
  USB_MotorTarget_t motor[DM_MOTOR_COUNT];
} USB_MotorCommand_t;

void USB_MotorComm_Receive(const uint8_t *data, uint32_t length);
uint8_t USB_MotorComm_GetCommand(USB_MotorCommand_t *command);
uint8_t USB_MotorComm_SendFeedback(void);
void USB_MotorComm_TransmitComplete(void);

#endif /* USB_MOTOR_COMM_H */
