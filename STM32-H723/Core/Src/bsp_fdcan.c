#include "bsp_fdcan.h"

extern void can_parse_feedback(FDCAN_HandleTypeDef *hcan, uint16_t rx_id, uint8_t *d, uint8_t len);
/**
************************************************************************
* @brief:       bsp_can_init(void)
* @param:       void
* @retval:      void
* @details:     Initialize CAN peripherals and filters
************************************************************************
**/
void bsp_can_init(void)
{
	can_filter_init();
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan2);
	//HAL_FDCAN_Start(&hfdcan3);

	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	//HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
/**
************************************************************************
* @brief:       can_filter_init(void)
* @param:       void
* @retval:      void
* @details:     Configure CAN acceptance filters and FIFO watermarks
************************************************************************
**/
void can_filter_init(void)
{
    FDCAN_FilterTypeDef fdcan_filter;

    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x00;
    fdcan_filter.FilterID2 = 0x00;

    // FDCAN1 filter
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                 FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);

    // === Added: same filter configuration for FDCAN2 ===
    HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                 FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
}
/**
************************************************************************
* @brief:       fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan        FDCAN handle
* @param:       id            CAN identifier
* @param:       data          Pointer to transmit data
* @param:       len           Length of data in bytes
* @retval:      0 on success, non-zero on error
* @details:     Build a CAN TX header and enqueue message to TX FIFO
************************************************************************
**/
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
    if (len > 8) len = 8;

    // 等待 TX FIFO 有空位（最多等 5ms）
    uint32_t t0 = HAL_GetTick();
    while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0) {
        if (HAL_GetTick() - t0 > 5) {
            return 2; // TX FIFO full timeout
        }
    }

    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier = id;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0;

    // DLC
    switch (len) {
        case 0: tx.DataLength = FDCAN_DLC_BYTES_0; break;
        case 1: tx.DataLength = FDCAN_DLC_BYTES_1; break;
        case 2: tx.DataLength = FDCAN_DLC_BYTES_2; break;
        case 3: tx.DataLength = FDCAN_DLC_BYTES_3; break;
        case 4: tx.DataLength = FDCAN_DLC_BYTES_4; break;
        case 5: tx.DataLength = FDCAN_DLC_BYTES_5; break;
        case 6: tx.DataLength = FDCAN_DLC_BYTES_6; break;
        case 7: tx.DataLength = FDCAN_DLC_BYTES_7; break;
        default: tx.DataLength = FDCAN_DLC_BYTES_8; break;
    }

    uint8_t payload[8] = {0};
    if (data && len) memcpy(payload, data, len);

    return (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx, payload) == HAL_OK) ? 0 : 1;
}



/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan��FDCAN���
* @param:       buf���������ݻ���
* @retval:     	���յ����ݳ���
* @details:    	��������
************************************************************************
**/
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
    if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) == 0)
        return 0;

    FDCAN_RxHeaderTypeDef rx = {0};
    uint8_t payload[8];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, payload) != HAL_OK)
        return 0;

    if (rx.IdType != FDCAN_STANDARD_ID)
        return 0;

    if (rec_id) *rec_id = (uint16_t)rx.Identifier;

    uint8_t len = 8;
    switch (rx.DataLength) {
        case FDCAN_DLC_BYTES_0: len = 0; break;
        case FDCAN_DLC_BYTES_1: len = 1; break;
        case FDCAN_DLC_BYTES_2: len = 2; break;
        case FDCAN_DLC_BYTES_3: len = 3; break;
        case FDCAN_DLC_BYTES_4: len = 4; break;
        case FDCAN_DLC_BYTES_5: len = 5; break;
        case FDCAN_DLC_BYTES_6: len = 6; break;
        case FDCAN_DLC_BYTES_7: len = 7; break;
        default: len = 8; break;
    }

    if (buf && len) memcpy(buf, payload, len);
    return len;
}




uint8_t rx_data1[8] = {0};
uint16_t rec_id1;

void fdcan1_rx_callback(void)
{
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
    {
        uint8_t len = fdcanx_receive(&hfdcan1, &rec_id1, rx_data1);
        if (len > 0)
        {
            can_parse_feedback(&hfdcan1, rec_id1, rx_data1, len);
        }
    }
}

uint8_t rx_data2[8] = {0};
uint16_t rec_id2;

void fdcan2_rx_callback(void)
{
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0)
    {
        uint8_t len = fdcanx_receive(&hfdcan2, &rec_id2, rx_data2);
        if (len > 0)
        {
            can_parse_feedback(&hfdcan2, rec_id2, rx_data2, len);
        }
    }
}
//uint8_t rx_data3[8] = {0};
//uint16_t rec_id3;
//void fdcan3_rx_callback(void)
//{
	//fdcanx_receive(&hfdcan3, &rec_id3, rx_data3);
//}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rxh;
    uint8_t rxdata[8];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;

    if (hfdcan->Instance == FDCAN1) {
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxh, rxdata);
        can_parse_feedback(&hfdcan1, rxh.Identifier, rxdata, 8);
    }
    else if (hfdcan->Instance == FDCAN2) {
        HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &rxh, rxdata);
        can_parse_feedback(&hfdcan2, rxh.Identifier, rxdata, 8);
    }

    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}











