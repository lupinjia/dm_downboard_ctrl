#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32h7xx.h"
#include "fdcan.h"

typedef struct {
    FDCAN_HandleTypeDef *hcan;
    FDCAN_TxHeaderTypeDef Header;
    uint8_t				Data[8];
}FDCAN_TxFrame_TypeDef;

typedef struct {
    FDCAN_HandleTypeDef *hcan;
    FDCAN_RxHeaderTypeDef Header;
    uint8_t 			Data[8];
} FDCAN_RxFrame_TypeDef;

extern FDCAN_TxFrame_TypeDef FDCAN1_TxFrame;
extern FDCAN_TxFrame_TypeDef FDCAN2_TxFrame;
extern FDCAN_RxFrame_TypeDef FDCAN12_RxFrame;

void FDCAN_Init(void);

#endif /* BSP_CAN_H */