#include "can.h"
#include "damiao_motor.h"
#include "motor_can_rx.h"
#include "rs02_motor.h"

HAL_StatusTypeDef Motor_CAN_Start(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef filter;

  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_CAN_Start(hcan) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t data[8];
  uint8_t ext_mode;
  uint8_t ext_host_id;

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data) != HAL_OK)
  {
    return;
  }

  if (rx_header.DLC < 8)
  {
    return;
  }

  if (rx_header.IDE == CAN_ID_STD)
  {
    if (rx_header.StdId == RS02_DEFAULT_MASTER_ID)
    {
      (void)RS02_HandleFrame(&rx_header, data, RS02_DEFAULT_MASTER_ID);
      return;
    }
    else if (rx_header.StdId == DAMIAO_MST_ID)
    {
      (void)Damiao_HandleFrame(&rx_header, data);
      return;
    }
    return;
  }
  else if (rx_header.IDE == CAN_ID_EXT)
  {
    ext_mode = (uint8_t)((rx_header.ExtId >> 24) & 0x1FU);
    ext_host_id = (uint8_t)(rx_header.ExtId & 0xFFU);

    if ((ext_mode == RS02_PRIVATE_MODE_FEEDBACK) &&
        (ext_host_id == (uint8_t)RS02_DEFAULT_MASTER_ID))
    {
      (void)RS02_Private_HandleFrame(&rx_header, data, RS02_DEFAULT_MASTER_ID);
    }
  }
}
