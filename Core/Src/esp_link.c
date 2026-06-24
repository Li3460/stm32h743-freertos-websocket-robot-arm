#include "esp_link.h"

#include "task.h"

#define ESP_DMA_BUFFER __attribute__((section(".dma_buffer"), aligned(32)))

static UART_HandleTypeDef *s_uart;
static StreamBufferHandle_t s_rx_stream;
static QueueHandle_t s_tx_queue;
static TaskHandle_t s_tx_task;
static volatile uint32_t s_uart_errors;
static volatile uint32_t s_rx_overflows;
static volatile bool s_tx_failed;
static volatile bool s_rx_active;
static ESP_DMA_BUFFER uint8_t s_rx_dma[ESP_UART_DMA_RX_SIZE];
static ESP_DMA_BUFFER uint8_t s_tx_dma[ESP_PROTOCOL_MAX_FRAME + 32U];

static void ESPLink_CacheClean(void *address, uint32_t size)
{
  if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
  {
    SCB_CleanDCache_by_Addr((uint32_t *)address, (int32_t)size);
  }
}

static void ESPLink_CacheInvalidate(void *address, uint32_t size)
{
  if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
  {
    SCB_InvalidateDCache_by_Addr((uint32_t *)address, (int32_t)size);
  }
}

static HAL_StatusTypeDef ESPLink_ArmReceive(void)
{
  HAL_StatusTypeDef status;

  if ((s_uart == NULL) || (s_rx_stream == NULL)) { return HAL_ERROR; }
  ESPLink_CacheInvalidate(s_rx_dma, sizeof(s_rx_dma));
  status = HAL_UARTEx_ReceiveToIdle_DMA(s_uart, s_rx_dma, sizeof(s_rx_dma));
  if ((status == HAL_OK) && (s_uart->hdmarx != NULL))
  {
    __HAL_DMA_DISABLE_IT(s_uart->hdmarx, DMA_IT_HT);
  }
  s_rx_active = (status == HAL_OK);
  return status;
}

void ESPLink_Init(UART_HandleTypeDef *uart,
                  StreamBufferHandle_t rx_stream,
                  QueueHandle_t tx_queue)
{
  s_uart = uart;
  s_rx_stream = rx_stream;
  s_tx_queue = tx_queue;
  s_tx_task = NULL;
  s_uart_errors = 0U;
  s_rx_overflows = 0U;
  s_tx_failed = false;
  s_rx_active = false;
}

HAL_StatusTypeDef ESPLink_StartReceive(void)
{
  return ESPLink_ArmReceive();
}

void ESPLink_ServiceReceive(void)
{
  if ((!s_rx_active) && (s_uart != NULL) &&
      (s_uart->RxState == HAL_UART_STATE_READY))
  {
    if (ESPLink_ArmReceive() != HAL_OK) { s_uart_errors++; }
  }
}

size_t ESPLink_Read(uint8_t *data, size_t length, TickType_t wait_ticks)
{
  if ((data == NULL) || (s_rx_stream == NULL)) { return 0U; }
  return xStreamBufferReceive(s_rx_stream, data, length, wait_ticks);
}

bool ESPLink_QueueTx(const EspTxMessage_t *message, uint32_t timeout_ms)
{
  if ((message == NULL) || (s_tx_queue == NULL)) { return false; }
  return xQueueSend(s_tx_queue, message, pdMS_TO_TICKS(timeout_ms)) == pdPASS;
}

HAL_StatusTypeDef ESPLink_TransmitMessage(const EspTxMessage_t *message)
{
  size_t length;
  HAL_StatusTypeDef status;

  if ((s_uart == NULL) || (message == NULL)) { return HAL_ERROR; }
  length = ESPProtocol_FormatTxFrame(message, (char *)s_tx_dma,
                                     sizeof(s_tx_dma));
  if (length == 0U) { return HAL_ERROR; }
  s_tx_task = xTaskGetCurrentTaskHandle();
  s_tx_failed = false;
  (void)ulTaskNotifyTake(pdTRUE, 0U);
  ESPLink_CacheClean(s_tx_dma, sizeof(s_tx_dma));
  status = HAL_UART_Transmit_DMA(s_uart, s_tx_dma, (uint16_t)length);
  if (status != HAL_OK) { return status; }
  if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250U)) == 0U)
  {
    (void)HAL_UART_AbortTransmit(s_uart);
    return HAL_TIMEOUT;
  }
  return s_tx_failed ? HAL_ERROR : HAL_OK;
}

uint32_t ESPLink_TakeUartErrorCount(void)
{
  uint32_t value;

  taskENTER_CRITICAL();
  value = s_uart_errors;
  s_uart_errors = 0U;
  taskEXIT_CRITICAL();
  return value;
}

uint32_t ESPLink_TakeOverflowCount(void)
{
  uint32_t value;

  taskENTER_CRITICAL();
  value = s_rx_overflows;
  s_rx_overflows = 0U;
  taskEXIT_CRITICAL();
  return value;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
  BaseType_t higher_priority_task_woken = pdFALSE;
  size_t sent;

  if ((uart != s_uart) || (s_rx_stream == NULL)) { return; }
  s_rx_active = false;
  if (size > sizeof(s_rx_dma)) { size = sizeof(s_rx_dma); }
  ESPLink_CacheInvalidate(s_rx_dma, sizeof(s_rx_dma));
  sent = xStreamBufferSendFromISR(s_rx_stream, s_rx_dma, size,
                                  &higher_priority_task_woken);
  if (sent < size) { s_rx_overflows += (uint32_t)(size - sent); }
  if (ESPLink_ArmReceive() != HAL_OK) { s_uart_errors++; }
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
  BaseType_t higher_priority_task_woken = pdFALSE;

  if ((uart == s_uart) && (s_tx_task != NULL))
  {
    vTaskNotifyGiveFromISR(s_tx_task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  BaseType_t higher_priority_task_woken = pdFALSE;

  if (uart != s_uart) { return; }
  s_uart_errors++;
  s_rx_active = false;
  if (s_tx_task != NULL)
  {
    s_tx_failed = true;
    vTaskNotifyGiveFromISR(s_tx_task, &higher_priority_task_woken);
  }
  (void)HAL_UART_AbortReceive(uart);
  if (ESPLink_ArmReceive() != HAL_OK) { s_uart_errors++; }
  portYIELD_FROM_ISR(higher_priority_task_woken);
}
