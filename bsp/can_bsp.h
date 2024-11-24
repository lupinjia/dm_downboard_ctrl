#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__
#include "main.h"
#include "fdcan.h"
#include "tim.h"
#include "string.h"

#define hcan_t FDCAN_HandleTypeDef

void can_bsp_init();
void can_filter_init(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len, uint16_t wait);


#endif /* __CAN_BSP_H_ */

