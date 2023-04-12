#include "stm32f4xx.h"
#include "core_cm4.h"
#include "stm32f4xx_hal.h"
// Include files
#include "drv_spi.h"

extern SPI_HandleTypeDef hspi1;

int8_t DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex,
                            uint8_t *SpiTxData,
                            uint8_t *SpiRxData,
                            uint16_t spiTransferSize) {
  HAL_GPIO_WritePin(CAN_SPI_nCS_GPIO, CAN_SPI_nCS_PIN, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, SpiTxData, SpiRxData, spiTransferSize, 500);
  HAL_GPIO_WritePin(CAN_SPI_nCS_GPIO, CAN_SPI_nCS_PIN, GPIO_PIN_SET);
  return 0;
}
