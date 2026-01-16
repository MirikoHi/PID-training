/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#define CAN_FILTER(x) ((x) << 3)
#define CAN_FIFO_0 0b000
#define CAN_FIFO_1 0b100
#define CAN_STDID 0b000
#define CAN_EXTID 0b010
#define CAN_DATA_TYPE 0b000
#define CAN_REMOTE_TYPE 0b001
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
 * @brief 初始化CAN总线
 *
 * @param hcan CAN编号
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_Start(hcan);
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}


/**
 * @brief 配置CAN的滤波器
 *
 * @param hcan CAN编号
 * @param Object_Para 编号[3:] | FIFOx[2:2] | ID类型[1:1] | 帧类型[0:0]
 * @param ID ID
 * @param Mask_ID 屏蔽位(0x7ff, 0x1fffffff)
 */
void CAN_Filter_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
  CAN_FilterTypeDef can_filter_init_structure;

  // ID配置, 标准帧的ID是11bit, 按规定放在高16bit中的[15:5]位
  // 掩码后ID的高16bit
  can_filter_init_structure.FilterIdHigh = (ID & 0x7FF) << 5;
  // 掩码后ID的低16bit
  can_filter_init_structure.FilterIdLow = 0x0000;
  // 掩码后屏蔽位的高16bit
  can_filter_init_structure.FilterMaskIdHigh = (Mask_ID & 0x7FF) << 5;
  // 掩码后屏蔽位的低16bit
  can_filter_init_structure.FilterMaskIdLow = 0x0000;

  // 滤波器配置
  // 滤波器序号, 0-27, 共28个滤波器, can1是0~13, can2是14~27
  can_filter_init_structure.FilterBank = (Object_Para >> 3) & 0x1F;
  // 滤波器模式, 设置ID掩码模式
  can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
  // 32位滤波
  can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
  // 使能滤波器
  can_filter_init_structure.FilterActivation = ENABLE;
  
  // 从机模式配置
  // 从机模式选择开始单元, 一般均分14个单元给CAN1和CAN2
  can_filter_init_structure.SlaveStartFilterBank = 14;

  // 滤波器绑定FIFOx, 只能绑定一个
  can_filter_init_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;

  HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
}

void CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
  CAN_TxHeaderTypeDef TX_header;
  uint32_t used_mailbox;

  // 检测关键传参
  assert_param(hcan != NULL);

  TX_header.StdId = ID;
  TX_header.ExtId = 0;
  TX_header.IDE = 0;
  TX_header.RTR = 0;
  TX_header.DLC = Length;

  if(HAL_CAN_AddTxMessage(hcan, &TX_header, Data, &used_mailbox) != HAL_OK){
    Error_Handler();
  }
}


/* USER CODE END 1 */
