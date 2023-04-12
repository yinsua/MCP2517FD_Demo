//
// Created by yinsua on 2023/3/20.
//

#include "CAN/canfdspi/drv_canfdspi_api.h"
#include "CAN/spi/drv_spi.h"
#include <stdlib.h>
#include <stdio.h>

// Transmit objects
CAN_TX_FIFO_CONFIG txConfig;
CAN_TX_FIFO_EVENT txFlags;
CAN_TX_MSGOBJ txObj;
uint8_t txd[MAX_DATA_BYTES];

// Receive objects
CAN_RX_FIFO_CONFIG rxConfig;
REG_CiFLTOBJ fObj;
REG_CiMASK mObj;
CAN_RX_FIFO_EVENT rxFlags;
CAN_RX_MSGOBJ rxObj;
uint8_t rxd[MAX_DATA_BYTES];
uint8_t can_send_flag = 0;

void can_rx_config() {
  // Setup RX FIFO
  DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(&rxConfig);
  rxConfig.FifoSize = 15;
  rxConfig.PayLoadSize = CAN_PLSIZE_8;
  rxConfig.RxTimeStampEnable = 0;

  DRV_CANFDSPI_ReceiveChannelConfigure(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, &rxConfig);
#if 1
  // Setup RX Filter
  fObj.word = 0;
  fObj.bF.SID = 0x00;
  fObj.bF.EXIDE = 0;
  fObj.bF.EID = 0x00;

  DRV_CANFDSPI_FilterObjectConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, &fObj.bF);

  // Setup RX Mask
  mObj.word = 0;
  mObj.bF.MSID = 0x0;
  mObj.bF.MIDE = 1; // Only allow standard IDs
  mObj.bF.MEID = 0x0;
  DRV_CANFDSPI_FilterMaskConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, &mObj.bF);

  // Link FIFO and Filter
  DRV_CANFDSPI_FilterToFifoLink(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, APP_RX_FIFO, true);
#endif
}

int8_t can_tx_config() {
#if 0
  // TEF Config
  CAN_TEF_CONFIG tefConfig;
  DRV_CANFDSPI_TefConfigureObjectReset(&tefConfig);
  tefConfig.FifoSize = 11;
  tefConfig.TimeStampEnable = 0;
  DRV_CANFDSPI_TefConfigure(DRV_CANFDSPI_INDEX_0, &tefConfig);
#endif
#ifdef USE_TX_QUEUE
  //TX Queen Config
  DRV_CANFDSPI_TransmitQueueReset(DRV_CANFDSPI_INDEX_0);
  CAN_TX_QUEUE_CONFIG queueConfig;
  DRV_CANFDSPI_TransmitQueueConfigureObjectReset(&queueConfig);
  queueConfig.FifoSize = 7;
  queueConfig.TxPriority = 0;
  queueConfig.PayLoadSize = CAN_PLSIZE_8;
  queueConfig.TxAttempts = 0;
  DRV_CANFDSPI_TransmitQueueConfigure(DRV_CANFDSPI_INDEX_0, &queueConfig);

#endif
#ifdef USE_TX_FIFO
  // Setup TX FIFO
  DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txConfig);
  txConfig.FifoSize = 12;
  txConfig.PayLoadSize = CAN_PLSIZE_8;
  txConfig.TxPriority = 0;
//  txConfig.TxAttempts = 0;//resending frame
  txConfig.RTREnable = 1;

  return DRV_CANFDSPI_TransmitChannelConfigure(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &txConfig);
#endif
}

void can_event_config() {
  // Setup Transmit and Receive Interrupts
  DRV_CANFDSPI_GpioModeConfigure(DRV_CANFDSPI_INDEX_0, GPIO_MODE_INT, GPIO_MODE_INT);
  DRV_CANFDSPI_GpioTransmitPinOpenDrainConfigure(DRV_CANFDSPI_INDEX_0, GPIO_PUSH_PULL);

  DRV_CANFDSPI_ModuleEventClear(DRV_CANFDSPI_INDEX_0, CAN_ALL_EVENTS);
#ifdef USE_TX_QUEUE
  DRV_CANFDSPI_TransmitQueueEventEnable(DRV_CANFDSPI_INDEX_0, CAN_TX_FIFO_NOT_FULL_EVENT);
#endif
#ifdef USE_TX_FIFO
  /// Maybe case NO_EVENT in DRV_CANFDSPI_TransmitChannelEventGet()
//  DRV_CANFDSPI_TransmitChannelEventEnable(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, CAN_TX_FIFO_ALL_EVENTS);
#endif
  DRV_CANFDSPI_ReceiveChannelEventEnable(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO,
                                         CAN_RX_FIFO_ALL_EVENTS);
  DRV_CANFDSPI_ModuleEventEnable(DRV_CANFDSPI_INDEX_0, CAN_TX_EVENT | CAN_RX_EVENT);
}

/**
CAN SPI MCP25XX芯片寄存器配置
*/
int8_t can_spi_software_init() {
  int8_t ret = 0;
  // Reset device
  DRV_CANFDSPI_Reset(DRV_CANFDSPI_INDEX_0);

  // Enable ECC and initialize RAM
  DRV_CANFDSPI_EccEnable(DRV_CANFDSPI_INDEX_0);

  DRV_CANFDSPI_RamInit(DRV_CANFDSPI_INDEX_0, 0xFF);

  // Configure device
  CAN_CONFIG config;
  DRV_CANFDSPI_ConfigureObjectReset(&config);
  config.IsoCrcEnable = 0;
  config.StoreInTEF = 0;
  config.TXQEnable = 1;

  DRV_CANFDSPI_Configure(DRV_CANFDSPI_INDEX_0, &config);

  can_rx_config();
  ret = can_tx_config();
  if (ret) return ret;

  can_event_config();

  // Setup Bit Time
  // 仲裁位与数据位的位波特率；晶振频率
  DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, CAN_1000K_8M,
                                CAN_SSP_MODE_AUTO, CAN_SYSCLK_20M);

  DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_CLASSIC_MODE);
  return 0;
}
/**
CAN SPI硬件初始化
*/
void can_spi_hardware_init() {
  //初始化已在main.c中全部完成，无需再次执行
//  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_SET);
}

/**
	CAN设置
*/
int can_init(void) {
  can_spi_hardware_init();
  return can_spi_software_init();
}

int can_test_reg(void) {
  // Variables
  uint8_t length;
  int good = 0;
  uint8_t i;

  Nop();

  // Verify read/write with different access length
  // Note: registers can be accessed in multiples of bytes
  for (length = 4; length <= MAX_DATA_BYTES; length++) {
    for (i = 0; i < length; i++) {
      txd[i] = rand() & 0x7f; // Bit 31 of Filter objects is not implemented
      rxd[i] = 0xff;
    }

    Nop();

    // Write data to registers
    DRV_CANFDSPI_WriteByteArray(DRV_CANFDSPI_INDEX_0, cRAMADDR_START, txd, length);

    // Read data back from registers
    DRV_CANFDSPI_ReadByteArray(DRV_CANFDSPI_INDEX_0, cRAMADDR_START, rxd, length);

    int size = length / 4;
    // Verify
    for (i = 0; i < size * 4; i++) {
      uint8_t a = txd[i], b = rxd[i];

      Nop();

      if (a != b) {

        Nop();
        Nop();

        // Data mismatch
        return 1;
      }
      good++;
    }
  }

  Nop();
  Nop();

  return 0;
}

void can_tx_obj_init() {
  txObj.word[0] = 0;// clean
  txObj.word[1] = 0;// clean

  txObj.bF.id.SID = 0;// standard id
  txObj.bF.id.EID = 0;//extend id

  //	CAN2.0 Frame please write "1", CANFD Frame "0" whole frame using NBR.
  txObj.bF.ctrl.BRS = 0;//ONLY USE FOR CANFD

  /// see <<MCP25xxFD Family Reference Manual>>
  //   do not replace "CAN_DLC_16" by Arabic numerals
  txObj.bF.ctrl.DLC = CAN_DLC_8;
  txObj.bF.ctrl.FDF = 0;//CAN:0, CANFD:1
  txObj.bF.ctrl.IDE = 0;//STD:0,EXT:4
  txObj.bF.ctrl.RTR = 0;//DATA:0, REMOTE:2
  txObj.bF.ctrl.ESI = 0;//错误状态指示符

  txObj.bF.ctrl.SEQ = 0;//用于跟踪发送事件FIFO中已发送报文的序列
}

int8_t can_send_queue_msg(uint16_t id, uint8_t *data, uint16_t size) {
  txObj.bF.id.SID = id;
  DRV_CANFDSPI_TransmitQueueEventGet(DRV_CANFDSPI_INDEX_0, &txFlags);
  if (txFlags & CAN_TX_FIFO_NOT_FULL_EVENT) {
    return DRV_CANFDSPI_TransmitQueueLoad(DRV_CANFDSPI_INDEX_0, &txObj, data, CAN_DLC_8, true);
  } else return 0;
}

int8_t can_send_msg(uint16_t id, uint8_t *data, uint16_t size) {
  can_tx_obj_init();
  txObj.bF.id.SID = id;

  CAN_TX_FIFO_STATUS status;
  DRV_CANFDSPI_TransmitChannelStatusGet(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &status);
  if (status & CAN_TX_FIFO_NOT_FULL) {
    int8_t ret = DRV_CANFDSPI_TransmitChannelLoad(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &txObj,
                                                  data, CAN_DLC_8, true);
    return ret;
  }
  else return status & 0xFF;
}

int32_t can_recv_msg(uint8_t *data) {
  DRV_CANFDSPI_ReceiveMessageGet(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, &rxObj,
                                 data, MAX_DATA_BYTES);
  return DRV_CANFDSPI_DlcToDataBytes(rxObj.bF.ctrl.DLC);
}

uint8_t can_recv_status() {
  CAN_RX_FIFO_STATUS sts;
  DRV_CANFDSPI_ReceiveChannelStatusGet(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, &sts);
  return sts & CAN_RX_FIFO_STATUS_MASK;
}

uint8_t can_recv_event() {
  CAN_RX_FIFO_EVENT event;
  DRV_CANFDSPI_ReceiveChannelEventGet(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, &event);
  return event;
}

uint16_t can_send_status() {
  CAN_TX_FIFO_STATUS sts;
  DRV_CANFDSPI_TransmitChannelStatusGet(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &sts);
  return sts & CAN_TX_FIFO_STATUS_MASK;
}

uint8_t can_send_event() {
  CAN_TX_FIFO_EVENT event;
  DRV_CANFDSPI_TransmitChannelEventGet(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &event);
  return event & CAN_TX_FIFO_ALL_EVENTS;
}

uint16_t can_send_queue_status() {
  CAN_TX_FIFO_STATUS sts;
  DRV_CANFDSPI_TransmitQueueStatusGet(DRV_CANFDSPI_INDEX_0, &sts);
  return sts & CAN_TX_FIFO_STATUS_MASK;
}

uint8_t can_send_queue_event() {
  CAN_TX_FIFO_EVENT event;
  DRV_CANFDSPI_TransmitChannelEventGet(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &event);
  DRV_CANFDSPI_TransmitQueueEventGet(DRV_CANFDSPI_INDEX_0, &event);
  return event & CAN_TX_FIFO_ALL_EVENTS;
}

void can_tx_int_callback() {
  CAN_MODULE_EVENT flags;
  DRV_CANFDSPI_ModuleEventGet(DRV_CANFDSPI_INDEX_0, &flags);
  if (((flags & CAN_TX_EVENT) == CAN_TX_EVENT) && can_send_flag) {
    can_send_flag = 0;
    DRV_CANFDSPI_TransmitChannelEventDisable(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, CAN_TX_FIFO_EMPTY_EVENT);
    DRV_CANFDSPI_ModuleEventDisable(DRV_CANFDSPI_INDEX_0, CAN_TX_EVENT);
    DRV_CANFDSPI_ModuleEventClear(DRV_CANFDSPI_INDEX_0, CAN_TX_EVENT);
  }
}

void can_tx_update() {
  DRV_CANFDSPI_TransmitChannelUpdate(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, true);
}