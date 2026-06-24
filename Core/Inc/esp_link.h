#ifndef ESP_LINK_H
#define ESP_LINK_H

#include "esp_protocol.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "stream_buffer.h"
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ESP_UART_RX_STREAM_SIZE  512U
#define ESP_UART_DMA_RX_SIZE     256U

void ESPLink_Init(UART_HandleTypeDef *uart,
                  StreamBufferHandle_t rx_stream,
                  QueueHandle_t tx_queue);
HAL_StatusTypeDef ESPLink_StartReceive(void);
void ESPLink_ServiceReceive(void);
size_t ESPLink_Read(uint8_t *data, size_t length, TickType_t wait_ticks);
bool ESPLink_QueueTx(const EspTxMessage_t *message, uint32_t timeout_ms);
HAL_StatusTypeDef ESPLink_TransmitMessage(const EspTxMessage_t *message);
uint32_t ESPLink_TakeUartErrorCount(void);
uint32_t ESPLink_TakeOverflowCount(void);

#endif /* ESP_LINK_H */
