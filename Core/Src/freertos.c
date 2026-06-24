/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    freertos.c
  * @brief   CMSIS-RTOS V2 objects and the six application tasks.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "main.h"

/* USER CODE BEGIN Includes */
#include "esp_link.h"
#include "esp_protocol.h"
#include "i2c.h"
#include "oled_status.h"
#include "queue.h"
#include "semphr.h"
#include "servo_manager.h"
#include "stream_buffer.h"
#include "system_status.h"
#include "usart.h"
#include <string.h>
/* USER CODE END Includes */

#define SERVO_CMD_QUEUE_LENGTH  8U
#define ESP_TX_QUEUE_LENGTH     12U
#define SAFETY_PERIOD_MS        50U
#define SERVO_PERIOD_MS         20U
#define OLED_PERIOD_MS          200U
#define MONITOR_PERIOD_MS       500U
#define HEARTBEAT_TIMEOUT_MS    2500U
#define STACK_CHECK_PERIOD_MS   2000U
#define STACK_WARNING_WORDS     64U

QueueHandle_t ServoCmdQueueHandle;
QueueHandle_t EspTxQueueHandle;
SemaphoreHandle_t SystemStatusMutexHandle;
StreamBufferHandle_t UartRxStreamBufferHandle;

osThreadId_t SafetyTaskHandle;
osThreadId_t ServoTaskHandle;
osThreadId_t EspRxTaskHandle;
osThreadId_t EspTxTaskHandle;
osThreadId_t OledTaskHandle;
osThreadId_t MonitorTaskHandle;

const osThreadAttr_t SafetyTask_attributes =
{
  .name = "SafetyTask",
  .stack_size = 256U * 4U,
  .priority = (osPriority_t)osPriorityHigh
};
const osThreadAttr_t ServoTask_attributes =
{
  .name = "ServoTask",
  .stack_size = 384U * 4U,
  .priority = (osPriority_t)osPriorityAboveNormal
};
const osThreadAttr_t EspRxTask_attributes =
{
  .name = "EspRxTask",
  .stack_size = 512U * 4U,
  .priority = (osPriority_t)osPriorityNormal
};
const osThreadAttr_t EspTxTask_attributes =
{
  .name = "EspTxTask",
  .stack_size = 512U * 4U,
  .priority = (osPriority_t)osPriorityNormal
};
const osThreadAttr_t OledTask_attributes =
{
  .name = "OledTask",
  .stack_size = 512U * 4U,
  .priority = (osPriority_t)osPriorityBelowNormal
};
const osThreadAttr_t MonitorTask_attributes =
{
  .name = "MonitorTask",
  .stack_size = 256U * 4U,
  .priority = (osPriority_t)osPriorityLow
};

void StartSafetyTask(void *argument);
void StartServoTask(void *argument);
void StartEspRxTask(void *argument);
void StartEspTxTask(void *argument);
void StartOledTask(void *argument);
void StartMonitorTask(void *argument);

static void App_PostError(uint32_t seq, EspErrorCode_t error)
{
  EspTxMessage_t message;

  (void)memset(&message, 0, sizeof(message));
  message.type = ESP_TX_ACK_ERROR;
  message.seq = seq;
  message.error = error;
  if (!ESPLink_QueueTx(&message, 0U))
  {
    SystemStatus_Increment(STATUS_COUNTER_QUEUE_FULL, 1U);
  }
  SystemStatus_SetError(ESPProtocol_ErrorText(error));
}

static void App_PostState(void)
{
  EspTxMessage_t message;
  SystemStatus_t status;
  uint8_t index;

  SystemStatus_GetSnapshot(&status);
  (void)memset(&message, 0, sizeof(message));
  message.type = ESP_TX_STATE;
  for (index = 0U; index < SERVO_COUNT; index++)
  {
    message.angles[index] = status.servo_angles[index];
  }
  message.moving = status.servo_moving;
  if (!ESPLink_QueueTx(&message, 0U))
  {
    SystemStatus_Increment(STATUS_COUNTER_QUEUE_FULL, 1U);
  }
}

static EspErrorCode_t App_ErrorFromText(const char *text)
{
  if (text == NULL) return ESP_ERROR_FORMAT;
  if (strcmp(text, "INVALID_ID") == 0) return ESP_ERROR_INVALID_ID;
  if (strcmp(text, "ANGLE_OUT_OF_RANGE") == 0) return ESP_ERROR_ANGLE_RANGE;
  if (strcmp(text, "SPEED_OUT_OF_RANGE") == 0) return ESP_ERROR_SPEED_RANGE;
  return ESP_ERROR_FORMAT;
}

static void App_HandleRxPacket(const EspRxPacket_t *packet)
{
  const char *validation_error;

  if (packet == NULL) { return; }
  switch (packet->type)
  {
    case ESP_RX_SERVO_COMMAND:
      if (!ServoManager_ValidateCommand(&packet->servo, &validation_error))
      {
        App_PostError(packet->servo.seq, App_ErrorFromText(validation_error));
        break;
      }
      if (packet->servo.type == SERVO_CMD_STOP)
      {
        SystemStatus_RequestStop(packet->servo.seq);
      }
      else
      {
        SystemStatus_SetSafetyStop(false, NULL);
        if (xQueueSend(ServoCmdQueueHandle, &packet->servo, 0U) != pdPASS)
        {
          SystemStatus_Increment(STATUS_COUNTER_QUEUE_FULL, 1U);
          App_PostError(packet->servo.seq, ESP_ERROR_QUEUE_FULL);
        }
      }
      break;
    case ESP_RX_HEARTBEAT:
      SystemStatus_Heartbeat(osKernelGetTickCount());
      break;
    case ESP_RX_QUERY:
      App_PostState();
      break;
    case ESP_RX_NET_BOOT:
      SystemStatus_SetNetwork(NET_BOOT, NULL);
      break;
    case ESP_RX_NET_CONNECTING:
      SystemStatus_SetNetwork(NET_CONNECTING, NULL);
      break;
    case ESP_RX_NET_CONNECTED:
      SystemStatus_SetNetwork(NET_CONNECTED, packet->ip_address);
      break;
    case ESP_RX_NET_FAILED:
      SystemStatus_SetNetwork(NET_FAILED, NULL);
      break;
    case ESP_RX_NET_LOST:
      SystemStatus_SetNetwork(NET_LOST, NULL);
      break;
    case ESP_RX_HTTP_READY:
      SystemStatus_SetHttpReady(packet->port == 80U);
      break;
    case ESP_RX_WS_READY:
      SystemStatus_SetWebSocketReady(packet->port == 81U);
      break;
    case ESP_RX_WS_CLIENTS:
      SystemStatus_SetWebSocketClients(packet->clients);
      break;
    default:
      break;
  }
}

void MX_FREERTOS_Init(void)
{
  SystemStatusMutexHandle = xSemaphoreCreateMutex();
  ServoCmdQueueHandle = xQueueCreate(SERVO_CMD_QUEUE_LENGTH,
                                     sizeof(ServoCommand_t));
  EspTxQueueHandle = xQueueCreate(ESP_TX_QUEUE_LENGTH,
                                  sizeof(EspTxMessage_t));
  UartRxStreamBufferHandle = xStreamBufferCreate(ESP_UART_RX_STREAM_SIZE, 1U);
  if ((SystemStatusMutexHandle == NULL) || (ServoCmdQueueHandle == NULL) ||
      (EspTxQueueHandle == NULL) || (UartRxStreamBufferHandle == NULL))
  {
    Error_Handler();
  }
  SystemStatus_AttachMutex(SystemStatusMutexHandle);
  ESPLink_Init(&huart2, UartRxStreamBufferHandle, EspTxQueueHandle);

  SafetyTaskHandle = osThreadNew(StartSafetyTask, NULL, &SafetyTask_attributes);
  ServoTaskHandle = osThreadNew(StartServoTask, NULL, &ServoTask_attributes);
  EspRxTaskHandle = osThreadNew(StartEspRxTask, NULL, &EspRxTask_attributes);
  EspTxTaskHandle = osThreadNew(StartEspTxTask, NULL, &EspTxTask_attributes);
  OledTaskHandle = osThreadNew(StartOledTask, NULL, &OledTask_attributes);
  MonitorTaskHandle = osThreadNew(StartMonitorTask, NULL, &MonitorTask_attributes);
  if ((SafetyTaskHandle == NULL) || (ServoTaskHandle == NULL) ||
      (EspRxTaskHandle == NULL) || (EspTxTaskHandle == NULL) ||
      (OledTaskHandle == NULL) || (MonitorTaskHandle == NULL))
  {
    Error_Handler();
  }
}

void StartSafetyTask(void *argument)
{
  TickType_t last_wake = xTaskGetTickCount();
  SystemStatus_t status;
  ServoCommand_t stop_command;
  bool stop_latched = false;
  bool manual_stop;
  bool heartbeat_lost;
  bool network_lost;
  bool no_clients;
  bool unsafe;

  (void)argument;
  (void)memset(&stop_command, 0, sizeof(stop_command));
  stop_command.type = SERVO_CMD_STOP;
  for (;;)
  {
    SystemStatus_GetSnapshot(&status);
    manual_stop = SystemStatus_ConsumeStopRequest(&stop_command.seq);
    if (!manual_stop) { stop_command.seq = 0U; }
    heartbeat_lost = ((xTaskGetTickCount() - status.last_heartbeat_tick) >
                      pdMS_TO_TICKS(HEARTBEAT_TIMEOUT_MS));
    network_lost = (status.network_state == NET_LOST);
    no_clients = (status.websocket_clients == 0U);
    unsafe = manual_stop || heartbeat_lost || network_lost || no_clients;
    if (heartbeat_lost || network_lost) { SystemStatus_SetLinkAlive(false); }
    if (unsafe && (!stop_latched || manual_stop))
    {
      (void)xQueueReset(ServoCmdQueueHandle);
      if (xQueueSend(ServoCmdQueueHandle, &stop_command, 0U) != pdPASS)
      {
        SystemStatus_Increment(STATUS_COUNTER_QUEUE_FULL, 1U);
      }
      SystemStatus_SetSafetyStop(true, manual_stop ? "STOP" :
                                 ((heartbeat_lost || network_lost) ?
                                  "LINK LOST" : "NO CLIENT"));
      stop_latched = true;
    }
    else if (!unsafe)
    {
      stop_latched = false;
      SystemStatus_SetSafetyStop(false, NULL);
    }
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAFETY_PERIOD_MS));
  }
}

void StartServoTask(void *argument)
{
  TickType_t last_wake = xTaskGetTickCount();
  TickType_t last_retry = last_wake - pdMS_TO_TICKS(1000U);
  ServoCommand_t command;
  EspTxMessage_t ack;
  const ServoConfig_t *config;
  bool hardware_ready = false;
  bool i2c_error_latched = false;
  uint8_t first;
  uint8_t last;
  uint8_t servo_id;

  (void)argument;
  for (;;)
  {
    if (!hardware_ready &&
        ((xTaskGetTickCount() - last_retry) >= pdMS_TO_TICKS(1000U)))
    {
      last_retry = xTaskGetTickCount();
      hardware_ready = (ServoManager_InitHardware(&hi2c1) == HAL_OK);
      if (!hardware_ready)
      {
        SystemStatus_Increment(STATUS_COUNTER_I2C, 1U);
        SystemStatus_Increment(STATUS_COUNTER_PCA9685, 1U);
        App_PostError(0U, ESP_ERROR_PCA9685);
      }
    }

    while (xQueueReceive(ServoCmdQueueHandle, &command, 0U) == pdPASS)
    {
      if (!hardware_ready && (command.type != SERVO_CMD_STOP))
      {
        App_PostError(command.seq, ESP_ERROR_PCA9685);
        continue;
      }
      ServoManager_ApplyCommand(&command);
      first = (command.type == SERVO_CMD_SINGLE) ? command.servo_id :
              ((command.type == SERVO_CMD_ALL) ? 1U : 0U);
      last = (command.type == SERVO_CMD_ALL) ? SERVO_COUNT : first;
      for (servo_id = first; servo_id <= last; servo_id++)
      {
        (void)memset(&ack, 0, sizeof(ack));
        ack.type = ESP_TX_ACK_OK;
        ack.seq = command.seq;
        ack.servo_id = servo_id;
        if (servo_id > 0U)
        {
          config = ServoManager_GetConfig(servo_id);
          ack.angle = (command.type == SERVO_CMD_HOME) ?
                      (int16_t)config->home_angle :
                      command.angles[servo_id - 1U];
          ack.pulse_us = ServoManager_GetPulseUs(servo_id, (float)ack.angle);
        }
        if (!ESPLink_QueueTx(&ack, 0U))
        {
          SystemStatus_Increment(STATUS_COUNTER_QUEUE_FULL, 1U);
        }
      }
    }

    if (hardware_ready)
    {
      if (ServoManager_Update20ms() != HAL_OK)
      {
        if (!i2c_error_latched)
        {
          SystemStatus_Increment(STATUS_COUNTER_I2C, 1U);
          SystemStatus_Increment(STATUS_COUNTER_PCA9685, 1U);
          SystemStatus_SetPca9685Ready(false);
          App_PostError(0U, ESP_ERROR_I2C);
          i2c_error_latched = true;
          hardware_ready = false;
        }
      }
      else { i2c_error_latched = false; }
    }
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SERVO_PERIOD_MS));
  }
}

void StartEspRxTask(void *argument)
{
  uint8_t input[64];
  size_t received;
  size_t index;
  EspFrameCollector_t collector;
  EspCollectResult_t result;
  EspRxPacket_t packet;
  EspErrorCode_t error;

  (void)argument;
  ESPProtocol_CollectorInit(&collector);
  if (ESPLink_StartReceive() != HAL_OK)
  {
    SystemStatus_Increment(STATUS_COUNTER_UART, 1U);
    SystemStatus_SetError("UART RX START");
  }
  for (;;)
  {
    received = ESPLink_Read(input, sizeof(input), portMAX_DELAY);
    for (index = 0U; index < received; index++)
    {
      result = ESPProtocol_CollectByte(&collector, input[index]);
      if (result == ESP_COLLECT_OVERFLOW)
      {
        SystemStatus_Increment(STATUS_COUNTER_UART_OVERFLOW, 1U);
        App_PostError(0U, ESP_ERROR_BUFFER_OVERFLOW);
      }
      else if (result == ESP_COLLECT_FRAME)
      {
        error = ESPProtocol_ParseFrame(collector.data, collector.length, &packet);
        if (error == ESP_ERROR_NONE) { App_HandleRxPacket(&packet); }
        else
        {
          if (error == ESP_ERROR_CRC)
          {
            SystemStatus_Increment(STATUS_COUNTER_CRC, 1U);
          }
          App_PostError(0U, error);
        }
      }
    }
  }
}

void StartEspTxTask(void *argument)
{
  EspTxMessage_t message;

  (void)argument;
  (void)memset(&message, 0, sizeof(message));
  message.type = ESP_TX_READY;
  if (!ESPLink_QueueTx(&message, 0U))
  {
    SystemStatus_Increment(STATUS_COUNTER_QUEUE_FULL, 1U);
  }
  for (;;)
  {
    if (xQueueReceive(EspTxQueueHandle, &message, portMAX_DELAY) == pdPASS)
    {
      if (ESPLink_TransmitMessage(&message) != HAL_OK)
      {
        SystemStatus_Increment(STATUS_COUNTER_UART, 1U);
      }
    }
  }
}

void StartOledTask(void *argument)
{
  SystemStatus_t status;
  uint32_t page_counter = 0U;
  uint32_t retry_elapsed = 5000U;
  bool ready = false;

  (void)argument;
  for (;;)
  {
    if (!ready)
    {
      retry_elapsed += OLED_PERIOD_MS;
      if (retry_elapsed >= 5000U)
      {
        retry_elapsed = 0U;
        ready = (OLEDStatus_Init(&hi2c2) == HAL_OK);
        SystemStatus_SetOledReady(ready);
        if (ready)
        {
          ready = (OLEDStatus_ShowStartup() == HAL_OK);
        }
        else
        {
          SystemStatus_Increment(STATUS_COUNTER_I2C, 1U);
          SystemStatus_Increment(STATUS_COUNTER_OLED, 1U);
          SystemStatus_SetError("OLED ERROR");
        }
      }
    }
    else
    {
      SystemStatus_GetSnapshot(&status);
      if (OLEDStatus_Render(&status, page_counter++) != HAL_OK)
      {
        ready = false;
        SystemStatus_SetOledReady(false);
        SystemStatus_Increment(STATUS_COUNTER_I2C, 1U);
        SystemStatus_Increment(STATUS_COUNTER_OLED, 1U);
        SystemStatus_SetError("OLED ERROR");
      }
    }
    osDelay(OLED_PERIOD_MS);
  }
}

void StartMonitorTask(void *argument)
{
  static const char *const names[6] =
  {
    "SafetyTask", "ServoTask", "EspRxTask", "EspTxTask", "OledTask", "MonitorTask"
  };
  osThreadId_t handles[6];
  bool warned[6] = {false, false, false, false, false, false};
  uint32_t elapsed = 0U;
  uint32_t count;
  uint8_t index;
  UBaseType_t remaining;
  EspTxMessage_t warning;

  (void)argument;
  handles[0] = SafetyTaskHandle;
  handles[1] = ServoTaskHandle;
  handles[2] = EspRxTaskHandle;
  handles[3] = EspTxTaskHandle;
  handles[4] = OledTaskHandle;
  handles[5] = MonitorTaskHandle;
  for (;;)
  {
    App_PostState();
    count = ESPLink_TakeUartErrorCount();
    if (count > 0U) { SystemStatus_Increment(STATUS_COUNTER_UART, count); }
    count = ESPLink_TakeOverflowCount();
    if (count > 0U) { SystemStatus_Increment(STATUS_COUNTER_UART_OVERFLOW, count); }
    ESPLink_ServiceReceive();

    elapsed += MONITOR_PERIOD_MS;
    if (elapsed >= STACK_CHECK_PERIOD_MS)
    {
      elapsed = 0U;
      for (index = 0U; index < 6U; index++)
      {
        remaining = uxTaskGetStackHighWaterMark((TaskHandle_t)handles[index]);
        if ((remaining < STACK_WARNING_WORDS) && !warned[index])
        {
          warned[index] = true;
          SystemStatus_Increment(STATUS_COUNTER_LOW_STACK, 1U);
          (void)memset(&warning, 0, sizeof(warning));
          warning.type = ESP_TX_STACK_WARNING;
          (void)memcpy(warning.task_name, names[index], strlen(names[index]) + 1U);
          warning.stack_words = (uint16_t)remaining;
          if (!ESPLink_QueueTx(&warning, 0U))
          {
            SystemStatus_Increment(STATUS_COUNTER_QUEUE_FULL, 1U);
          }
        }
      }
    }
    osDelay(MONITOR_PERIOD_MS);
  }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
  (void)task;
  (void)task_name;
  taskDISABLE_INTERRUPTS();
  for (;;) { }
}

void vApplicationMallocFailedHook(void)
{
  taskDISABLE_INTERRUPTS();
  for (;;) { }
}
