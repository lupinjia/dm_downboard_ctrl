/**
 * @file    can_bsp.c
 * @author  Shengcong Sang
 * @brief   board support package for can(fdcan) driver
 * @version 1.0
 * @date    2024-12-08
 * 
 * @copyright HYGroup@BIT 2024
 * 
 */
#include "can_bsp.h"

FDCAN_TxFrame_TypeDef FDCAN1_TxFrame = {
	.hcan = &hfdcan1,
	.Header.Identifier = 0x7FF,
	.Header.IdType = FDCAN_STANDARD_ID, 
	.Header.TxFrameType = FDCAN_DATA_FRAME,
	.Header.DataLength = FDCAN_DLC_BYTES_8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
	.Header.BitRateSwitch = FDCAN_BRS_OFF,
	.Header.FDFormat =  FDCAN_CLASSIC_CAN,           
	.Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
	.Header.MessageMarker = 0,
};

FDCAN_TxFrame_TypeDef FDCAN2_TxFrame = {
	.hcan = &hfdcan2,
	.Header.Identifier = 0x7FF,
	.Header.IdType = FDCAN_STANDARD_ID, 
	.Header.TxFrameType = FDCAN_DATA_FRAME,
	.Header.DataLength = FDCAN_DLC_BYTES_8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_PASSIVE,
	.Header.BitRateSwitch = FDCAN_BRS_OFF,
	.Header.FDFormat =  FDCAN_CLASSIC_CAN,           
	.Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
	.Header.MessageMarker = 0,
};

FDCAN_RxFrame_TypeDef FDCAN12_RxFrame;

void FDCAN_Init(void) {
	// Configure filter
	FDCAN_FilterTypeDef FDCAN_FilterConfig;
	FDCAN_FilterConfig.IdType = FDCAN_STANDARD_ID;
	FDCAN_FilterConfig.FilterIndex = 0;
	FDCAN_FilterConfig.FilterType = FDCAN_FILTER_MASK;
	FDCAN_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	FDCAN_FilterConfig.FilterID1 = 0x00;    // 接收所有ID
	FDCAN_FilterConfig.FilterID2 = 0x00;    // 掩码为0，不过滤
	HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_FilterConfig);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
																	FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	FDCAN_FilterConfig.IdType = FDCAN_STANDARD_ID;
	FDCAN_FilterConfig.FilterIndex = 0;
	FDCAN_FilterConfig.FilterType = FDCAN_FILTER_MASK;
	FDCAN_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	FDCAN_FilterConfig.FilterID1 = 0x00;    // 接收所有ID
	FDCAN_FilterConfig.FilterID2 = 0x00;    // 掩码为0，不过滤
	HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN_FilterConfig);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_ACCEPT_IN_RX_FIFO1,
																	FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

	// Start FDCAN1 and FDCAN2
	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_Start(&hfdcan2);

	// Activate FDCAN1 and FDCAN2 RX new message notification
	// FDCAN1->RX FIFO0
	// FDCAN2->RX FIFO1
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
}