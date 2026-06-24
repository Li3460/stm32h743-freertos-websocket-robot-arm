# STM32H743 ESP8266 WebSocket 机械臂调试端

## 1. 工程概况

本工程实现 STM32H743VIT6 侧的 ESP8266 WebSocket 机械臂调试系统：

- MCU：STM32H743VIT6，当前系统时钟 480 MHz，启用 I-Cache 和 D-Cache。
- ESP8266：USART2，PA2/PA3，115200-8-N-1，RX/TX 均使用 DMA。
- PCA9685：I2C1，PB8/PB9，7 位地址 0x40，PWM 50 Hz。
- SSD1306：I2C2，PB10/PB11，先探测 0x3C，再探测 0x3D。
- RTOS：CMSIS-RTOS V2 + FreeRTOS `heap_4`，应用对象全部动态创建。
- HAL 时基：TIM6；FreeRTOS 内核 tick：SysTick。

原工程只有 CubeMX 外设初始化、七个空任务和一个绑定到错误 I2C 总线的旧 OLED 驱动。旧 `bsp/driver_oled.c` 保留作历史参考，但已从 Keil/CMake 构建中移除。

## 2. 新增文件

- `Core/Inc|Src/esp_protocol.*`：有界帧收集、XOR 校验、命令解析和发送组帧。
- `Core/Inc|Src/esp_link.*`：USART2 ReceiveToIdle DMA、StreamBuffer 和 TX DMA。
- `Core/Inc|Src/servo_manager.*`：六轴参数、限位、缓动、HOME 和 STOP。
- `Core/Inc|Src/system_status.*`：共享状态和短时互斥访问。
- `Core/Inc|Src/pca9685.*`：PCA9685 初始化和通道 PWM 输出。
- `Core/Inc|Src/ssd1306.*`：SSD1306 地址探测、显存和刷新。
- `Core/Inc|Src/fonts.*`：紧凑 5x7 ASCII 字体。
- `Core/Inc|Src/oled_status.*`：状态页面渲染。
- `Core/Src/stm32h7xx_hal_timebase_tim.c`：TIM6 HAL 1 ms 时基。
- `MDK-ARM/websocket_custom.sct`：DMA D2 SRAM 布局。

同时修改了 `.ioc`、FreeRTOSConfig、main、I2C、USART、DMA、中断、Keil、CMake 和 GNU linker script。

## 3. RTOS 任务

CubeMX 的 Stack Size 单位为 Word；STM32H743 上 1 Word = 4 Bytes。CMSIS V2 `stack_size` 单位为 Byte。

| Task | Priority | CubeMX Words | CMSIS Bytes | 运行方式 |
|---|---:|---:|---:|---|
| SafetyTask | High | 256 | 1024 | `vTaskDelayUntil`，50 ms |
| ServoTask | AboveNormal | 384 | 1536 | `vTaskDelayUntil`，20 ms |
| EspRxTask | Normal | 512 | 2048 | 阻塞等待 StreamBuffer |
| EspTxTask | Normal | 512 | 2048 | 阻塞等待 EspTxQueue |
| OledTask | BelowNormal | 512 | 2048 | 200 ms 周期 |
| MonitorTask | Low | 256 | 1024 | 500 ms 周期 |

没有 `defaultTask`。所有高优先级任务都会快速进入阻塞或周期延时。

## 4. FreeRTOS 对象

- `ServoCmdQueue`：8 个 `ServoCommand_t`。
- `EspTxQueue`：12 个 `EspTxMessage_t`。
- `UartRxStreamBuffer`：512 字节，Trigger Level 1。
- `SystemStatusMutex`：保护 `SystemStatus_t`。

为满足 MDK-Lite 32 KiB 限制，线程仍由 CMSIS-RTOS V2 创建，两个队列、StreamBuffer 和互斥锁使用等价的 FreeRTOS 原生动态 API。没有 PCA9685 或 OLED 互斥锁：ServoTask 独占 hi2c1，OledTask 独占 hi2c2。

`FreeRTOSConfig.h` 的关键配置：

- `USE_FreeRTOS_HEAP_4`
- `configTOTAL_HEAP_SIZE = 32768`
- `configSUPPORT_STATIC_ALLOCATION = 0`
- `configCHECK_FOR_STACK_OVERFLOW = 2`
- `configUSE_MALLOC_FAILED_HOOK = 1`
- `INCLUDE_uxTaskGetStackHighWaterMark = 1`
- `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`

本应用不使用软件定时器，因此 `configUSE_TIMERS = 0`。MonitorTask 每 2 秒读取六个任务的历史最小剩余栈；低于 64 Words 时只发送一次 `$WARN,STACK,...`。

## 5. 初始化和外设所有权

启动顺序为：HAL/MPU、Cache、系统时钟、GPIO、DMA、I2C1、I2C2、USART2、系统状态、RTOS 对象、六个任务、调度器。

为保证外设单一所有者：

- PCA9685 初始化在 ServoTask 中完成，初始化不会写六路舵机 PWM。
- SSD1306 探测和初始化在 OledTask 中完成。
- ReceiveToIdle DMA 在 EspRxTask 启动后开启，避免调度器启动前发生 FromISR API 调用。
- READY 消息由 EspTxTask 先放入 EspTxQueue，再由同一任务取出发送。

## 6. USART2 DMA 流程

接收流程：

1. `HAL_UARTEx_ReceiveToIdle_DMA` 接收至 256 字节 DMA 缓冲区。
2. 禁用 Half Transfer 中断，避免同一批数据重复提交。
3. IDLE、DMA TC 或 UART 错误进入对应回调。
4. `HAL_UARTEx_RxEventCallback` 只做 Cache Invalidate、写入 512 字节 StreamBuffer、重新挂接 DMA 和任务唤醒。
5. EspRxTask 阻塞读取 StreamBuffer，在任务上下文完成协议解析。
6. UART 错误会停止异常接收、计数并重新挂接 ReceiveToIdle DMA。

发送流程：

1. 其他任务只投递结构化 `EspTxMessage_t`。
2. EspTxTask 统一生成 `$payload*XX\r\n` 并计算 XOR。
3. EspTxTask 启动 TX DMA，阻塞等待完成通知或 250 ms 超时。
4. 只有 EspTxTask 调用 `HAL_UART_Transmit_DMA`。

中断优先级：USART2=5、RX DMA Stream0=5、TX DMA Stream1=6、TIM6=15、SysTick/PendSV=15。

## 7. DMA 和 D-Cache

DMA1 无法访问默认 DTCM，所以仅做 32 字节对齐仍不够。本工程采用以下方案：

- RX 缓冲区 256 字节，TX 缓冲区 192 字节，均为 32 字节对齐。
- 两个缓冲区进入 `.dma_buffer` section。
- Keil scatter 和 GNU linker script 将该 section 放到 D2 SRAM `0x30000000`。
- Keil map 已验证：TX=`0x30000000`，RX=`0x300000C0`。
- RX DMA 前/后执行 Cache Invalidate，TX DMA 前执行 Cache Clean。

不要删除 `websocket_custom.sct`、`.dma_buffer` 属性或 Cache 维护调用。

## 8. 协议

帧格式：`$数据内容*XX\r\n`。XOR 范围为 `$` 后、`*` 前的每个字节，输出为两位大写十六进制。

解析器支持 SV、ALL、HOME、STOP、HB、QUERY 及全部约定 NET 状态；支持半帧、粘包和连续多帧。遇到新 `$` 会重新同步，超过 160 字节的帧会丢弃并上报 `BUFFER_OVERFLOW`。

解析代码未使用 `strcpy`、`sprintf`、`sscanf`；整数解析、字段拆分、IP 地址和输出缓冲区都有长度限制。

发送支持 READY、ACK、STATE，并扩展低栈告警 `$WARN,STACK,<task>,<words>*XX`。ALL 为六轴分别返回 ACK，HOME/STOP 使用 ID 0 表示整机命令。

## 9. 舵机控制

六轴临时参数位于 `servo_manager.c`，必须按真实机械结构重新标定。

ServoTask 每 20 ms 执行：

1. 消费待处理命令。
2. 计算 `step = min(command_speed, axis_max_speed) * 0.02`。
3. 逐步移动 `current_angle` 到 `target_angle`。
4. 仅在角度变化或首次激活通道时写 PCA9685。
5. 完成方向变换、500～2500 us 映射、12 位计数映射和 0～4095 限幅。

STOP 将所有 target 固定到 current，保持现有 PWM；HOME 按 30 deg/s 缓动到 home。

`SERVO_MOVE_HOME_ON_BOOT` 默认是 0。PCA9685 上电只配置模式和 50 Hz，不主动输出六路舵机脉宽。第一次有效命令只激活命令涉及的通道。

## 10. OLED

OledTask 先探测 `0x3C << 1`，失败后探测 `0x3D << 1`。初始化或刷新失败只更新状态和错误计数，每 5 秒重试；不会阻塞串口或舵机系统。

页面包括启动、ESP UART、Wi-Fi、IP、HTTP、WebSocket、客户端数量、舵机 target/current/PWM、STOP、LINK LOST、CRC、PCA 和 OLED 错误。

## 11. 心跳和安全停止

SafetyTask 每 50 ms 检查：

- 有效 HB 是否超过 2500 ms 未更新。
- 网络是否为 `NET_LOST`。
- WebSocket 客户端数量是否为 0。
- 是否收到显式 STOP。

首次进入不安全状态时，SafetyTask 清除未执行的运动命令并向 ServoCmdQueue 发送 STOP。目标固定在当前软件角度，PWM 保持，不保存或恢复旧目标。通信恢复后仍停在当前位置，直到收到新运动命令。

## 12. CubeMX 手动确认

再次打开 `.ioc` 时确认：

1. PA2/PA3 为 USART2 TX/RX，115200-8-N-1。
2. RX DMA=DMA1 Stream0/Normal/High，IRQ 5。
3. TX DMA=DMA1 Stream1/Normal/Low，IRQ 6。
4. USART2 global IRQ=5。
5. PB8/PB9=I2C1，PB10/PB11=I2C2，并有外部上拉。
6. SYS Timebase Source=TIM6，FreeRTOS 使用 SysTick。
7. FreeRTOS Interface=CMSIS_V2、heap_4、Total Heap=32768、只动态分配。
8. 六任务的 Stack Size 按表格以 Words 填写。
9. 不要把调用 FromISR API 的 USART2/RX DMA IRQ 改到 0～4。

StreamBuffer、两个原生队列和互斥锁在 `MX_FREERTOS_Init` 中创建。CubeMX 再生代码可能覆盖为 CMSIS 对象并使 MDK-Lite 超限，因此再生成后应对照本 README 和 Keil 工程复核。

## 13. Keil 工程

Keil 使用 ArmClang 6.23、MicroLIB 和 `-Oz`。原工程错误引用 RVDS port，现已改为同版本 CubeH7 官方 `portable/GCC/ARM_CM4F` port。

为满足 MDK-Lite 32 KiB 限制，USART2 固定参数初始化、USART2 IRQ 和 TIM6 基本时基只保留本项目实际路径；ReceiveToIdle、UART DMA、I2C 传输等运行接口仍使用 HAL。重新生成 CubeMX 代码后不要无检查覆盖 `usart.c`、`i2c.c`、`stm32h7xx_it.c` 和 `stm32h7xx_hal_timebase_tim.c`。

最终 Keil Rebuild：

- `0 Error(s), 0 Warning(s)`。
- Code=29930、RO-data=1574、RW-data=664、ZI-data=36960 Bytes。
- 已生成 `MDK-ARM/websocket/websocket.axf` 和 HEX。

## 14. 上电测试顺序

1. 先断开舵机动力电源，只给 MCU、ESP、PCA9685 逻辑和 OLED 供电，并确认共地。
2. 检查 PB8/PB9、PB10/PB11 外部上拉和总线无持续拉低。
3. 确认 USART2 为 115200，并收到 STM32 READY。
4. 发送正确 XOR 的 NET、HB、WS_CLIENTS=1，检查每 500 ms STATE。
5. 测试错误 XOR、超长帧、半帧、粘包和多帧恢复。
6. 不接舵机，用示波器确认 PCA9685 为 50 Hz，脉宽在标定范围内。
7. 单接 1 号舵机，以 5～10 deg/s 做小角度方向和限位测试。
8. 逐轴标定 min/max/home/direction 后再连接全部机构。
9. 运动中分别停止 HB、发送 WIFI_LOST、置客户端为 0、发送 STOP，确认 PWM 保持且角度停止变化。
10. 长时间运行并检查 STACK WARN、CRC、UART、I2C 和队列计数。

## 15. 风险

- 系统没有舵机位置反馈，上电时软件只能假定机械臂位于配置的 home。第一次命令前必须人工放到已知安全姿态。
- 500～2500 us、角度限位和方向只是测试值，未标定可能造成堵转或撞限位。
- ESP-01 和舵机需要可靠独立电源，舵机电源与 MCU 应分开但必须共地。
- I2C 软件重试无法修复外设永久拉低总线等硬件故障。
- 手工精简的 UART/TIM6 路径与当前 STM32Cube H7 HAL V1.13.0 匹配；升级 HAL 后必须复核寄存器和回调状态机。
