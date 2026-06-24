# ESP8266 烧录与连接

目标：ESP8266 连接 `iPhone14`，提供 HTTP 80 / WebSocket 81，并在 UART0 上使用 STM32 协议。

烧录接线：USB-TTL TX→RXD、RX→TXD、GND→GND；独立稳定3.3V→VCC；USB-TTL与电源共地。IO0接GND，RST短接GND后松开进入下载模式。

正常运行：断电，移除 IO0-GND，再上电或复位。随后断开 USB-TTL 的 TX/RX，将 ESP RXD 接 STM32 PA2/USART2_TX，ESP TXD 接 STM32 PA3/USART2_RX，保持共地。

热点连接成功后，从手机浏览器访问 ESP 获得的 IP 地址，例如 `http://172.20.10.2/`。
