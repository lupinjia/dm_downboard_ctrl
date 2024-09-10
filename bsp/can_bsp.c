#include "can_bsp.h"
/**
************************************************************************
* @brief:      	can_bsp_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN ʹ��
************************************************************************
**/
void can_bsp_init()
{
	can_filter_init();
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_Start(&hfdcan1);
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN�˲�����ʼ��
************************************************************************
**/
void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID;
	fdcan_filter.FilterIndex = 0;                
	fdcan_filter.FilterType = FDCAN_FILTER_RANGE;                   
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  
	fdcan_filter.FilterID1 = 0x00;                               
	fdcan_filter.FilterID2 = 0x7FF;
	HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter);
	// fdcan_filter.IdType = FDCAN_EXTENDED_ID;             // can configure std and ext filter separately, not used here
	// fdcan_filter.FilterIndex = 0;
	// fdcan_filter.FilterType = FDCAN_FILTER_RANGE;
	// fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	// fdcan_filter.FilterID1 = 0x00;
	// fdcan_filter.FilterID2 = 0x1FFFFFFF;
	// HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
}
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan: handle of FDCAN
* @param:       id: id of message
* @param:       data: uint8_t array
* @param:       len: data field length(Bytes)
* @retval:     	void
* @details:    	
************************************************************************
**/
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
	// uint16_t wait = 0xFFFF;
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier=id;
    pTxHeader.IdType=FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;
	pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_ON;
    pTxHeader.FDFormat=FDCAN_FD_CAN;
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;
	// while(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan)==0)
	// {
	// 	wait--;
	// 	if(wait == 0)
	// 		break;
	// }
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data)!=HAL_OK) 
		return 1;

	return 0;	
}
