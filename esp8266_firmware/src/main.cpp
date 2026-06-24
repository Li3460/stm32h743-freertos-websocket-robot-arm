#include <Arduino.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>

#include "wifi_secrets.h"

namespace
{
constexpr uint16_t kHttpPort = 80;
constexpr uint16_t kWebSocketPort = 81;
constexpr uint32_t kHeartbeatPeriodMs = 1000;
constexpr uint32_t kWifiAttemptTimeoutMs = 20000;
constexpr uint32_t kWifiRetryPeriodMs = 5000;
constexpr size_t kMaxProtocolFrame = 160;
constexpr size_t kSerialLineCapacity = 192;
constexpr uint8_t kTrackedClients = 8;

ESP8266WebServer httpServer(kHttpPort);
WebSocketsServer webSocket(kWebSocketPort);

bool webSocketClients[kTrackedClients] = {};
bool serversStarted = false;
bool wifiConnected = false;
bool wifiFailureReported = false;
uint32_t wifiAttemptStarted = 0;
uint32_t lastWifiRetry = 0;
uint32_t lastHeartbeat = 0;
uint32_t heartbeatSequence = 0;
char serialLine[kSerialLineCapacity];
size_t serialLineLength = 0;
bool serialLineOverflow = false;

const char kIndexHtml[] PROGMEM = R"HTML(
<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ARM SERVO DEBUG</title>
<style>
body{font-family:system-ui;margin:0;background:#101820;color:#e8f1f7}main{max-width:720px;margin:auto;padding:18px}
h1{font-size:22px}.card{background:#1b2a34;border-radius:12px;padding:14px;margin:12px 0}
.row{display:grid;grid-template-columns:80px 1fr 46px;gap:10px;align-items:center;margin:12px 0}
input{width:100%}button{padding:10px 14px;margin:5px;border:0;border-radius:8px;background:#24a1de;color:white}
button.stop{background:#d94242}#log{white-space:pre-wrap;font-family:monospace;font-size:12px;max-height:220px;overflow:auto}
.online{color:#52d273}.offline{color:#ff6b6b}
</style></head><body><main><h1>ESP8266 机械臂调试</h1>
<div class="card">WebSocket: <b id="state" class="offline">连接中</b><br>IP: <span id="ip"></span></div>
<div class="card" id="servos"></div>
<div class="card">速度 <input id="speed" type="range" min="1" max="100" value="30"><span id="speedVal">30</span> °/s<br>
<button onclick="sendAll()">发送全部</button><button onclick="sendCmd('HOME,'+nextSeq())">HOME</button>
<button class="stop" onclick="sendCmd('STOP,'+nextSeq())">STOP</button><button onclick="sendCmd('QUERY,'+nextSeq())">QUERY</button></div>
<div class="card"><div id="log"></div></div></main>
<script>
const limits=[[10,170,90],[20,160,90],[20,160,90],[10,170,90],[10,170,90],[10,100,30]];
let seq=1,ws;const box=document.getElementById('servos');
limits.forEach((v,i)=>{box.insertAdjacentHTML('beforeend',`<div class="row"><label>舵机 ${i+1}</label><input id="s${i}" type="range" min="${v[0]}" max="${v[1]}" value="${v[2]}" oninput="v${i}.textContent=this.value"><span id="v${i}">${v[2]}</span></div>`)});
speed.oninput=()=>speedVal.textContent=speed.value;function nextSeq(){return seq++}
function frame(p){let x=0;for(let i=0;i<p.length;i++)x^=p.charCodeAt(i);return '$'+p+'*'+x.toString(16).padStart(2,'0').toUpperCase()+'\r\n'}
function log(t){const e=document.getElementById('log');e.textContent=(new Date().toLocaleTimeString()+' '+t+'\n'+e.textContent).slice(0,5000)}
function sendCmd(p){if(ws&&ws.readyState===1){const f=frame(p);ws.send(f);log('TX '+f.trim())}}
function sendOne(i){sendCmd(`SV,${i+1},${document.getElementById('s'+i).value},${speed.value},${nextSeq()}`)}
box.addEventListener('change',e=>{if(e.target.type==='range')sendOne(Number(e.target.id.slice(1)))})
function sendAll(){let a=[];for(let i=0;i<6;i++)a.push(document.getElementById('s'+i).value);sendCmd(`ALL,${a.join(',')},${speed.value},${nextSeq()}`)}
function connect(){ws=new WebSocket(`ws://${location.hostname}:81/`);ws.onopen=()=>{state.textContent='在线';state.className='online';log('WS ONLINE')};
ws.onclose=()=>{state.textContent='离线';state.className='offline';setTimeout(connect,1500)};ws.onerror=()=>ws.close();ws.onmessage=e=>log('RX '+e.data.trim())}
ip.textContent=location.hostname;connect();
</script></body></html>
)HTML";

uint8_t protocolXor(const String &payload)
{
  uint8_t checksum = 0;
  for (size_t index = 0; index < payload.length(); ++index)
  {
    checksum ^= static_cast<uint8_t>(payload[index]);
  }
  return checksum;
}

void sendProtocol(const String &payload)
{
  static const char hex[] = "0123456789ABCDEF";
  const uint8_t checksum = protocolXor(payload);
  Serial.write('$');
  Serial.print(payload);
  Serial.write('*');
  Serial.write(hex[(checksum >> 4U) & 0x0FU]);
  Serial.write(hex[checksum & 0x0FU]);
  Serial.print("\r\n");
}

uint8_t connectedClientCount()
{
  uint8_t count = 0;
  for (bool connected : webSocketClients)
  {
    if (connected) { ++count; }
  }
  return count;
}

void reportClients()
{
  sendProtocol(String(F("NET,WS_CLIENTS,")) + connectedClientCount());
}

void relayWebSocketFrame(const uint8_t *payload, size_t length)
{
  if ((payload == nullptr) || (length == 0) ||
      (length > kMaxProtocolFrame) || (payload[0] != '$'))
  {
    return;
  }
  Serial.write(payload, length);
  if (payload[length - 1] != '\n')
  {
    Serial.print("\r\n");
  }
}

void webSocketEvent(uint8_t client, WStype_t type, uint8_t *payload,
                    size_t length)
{
  switch (type)
  {
    case WStype_CONNECTED:
      if (client < kTrackedClients) { webSocketClients[client] = true; }
      reportClients();
      break;
    case WStype_DISCONNECTED:
      if (client < kTrackedClients) { webSocketClients[client] = false; }
      reportClients();
      break;
    case WStype_TEXT:
      relayWebSocketFrame(payload, length);
      break;
    default:
      break;
  }
}

void startServers()
{
  if (serversStarted) { return; }
  httpServer.on("/", HTTP_GET, []() { httpServer.send_P(200, "text/html; charset=utf-8", kIndexHtml); });
  httpServer.on("/health", HTTP_GET, []() { httpServer.send(200, "text/plain", "OK"); });
  httpServer.onNotFound([]() { httpServer.send(404, "text/plain", "Not found"); });
  httpServer.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  serversStarted = true;
}

void startWifiAttempt()
{
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  wifiAttemptStarted = millis();
  lastWifiRetry = wifiAttemptStarted;
  wifiFailureReported = false;
  sendProtocol(F("NET,WIFI_CONNECTING"));
}

void serviceWifi()
{
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED)
  {
    if (!wifiConnected)
    {
      wifiConnected = true;
      wifiFailureReported = false;
      sendProtocol(String(F("NET,WIFI_CONNECTED,")) + WiFi.localIP().toString());
      startServers();
      sendProtocol(F("NET,HTTP_READY,80"));
      sendProtocol(String(F("NET,WS_READY,")) + kWebSocketPort);
      reportClients();
    }
    return;
  }

  if (wifiConnected)
  {
    wifiConnected = false;
    for (bool &client : webSocketClients) { client = false; }
    sendProtocol(F("NET,WIFI_LOST"));
    reportClients();
    lastWifiRetry = millis();
  }
  if (!wifiFailureReported &&
      (millis() - wifiAttemptStarted >= kWifiAttemptTimeoutMs))
  {
    wifiFailureReported = true;
    sendProtocol(F("NET,WIFI_FAILED"));
  }
  if (millis() - lastWifiRetry >= kWifiRetryPeriodMs)
  {
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWifiRetry = millis();
    wifiAttemptStarted = lastWifiRetry;
    wifiFailureReported = false;
    sendProtocol(F("NET,WIFI_CONNECTING"));
  }
}

void serviceSerialBridge()
{
  while (Serial.available() > 0)
  {
    const char byte = static_cast<char>(Serial.read());
    if (!serialLineOverflow && (serialLineLength + 1U < sizeof(serialLine)))
    {
      serialLine[serialLineLength++] = byte;
    }
    else
    {
      serialLineOverflow = true;
    }
    if (byte == '\n')
    {
      if (!serialLineOverflow && (serialLineLength > 0U))
      {
        serialLine[serialLineLength] = '\0';
        if (connectedClientCount() > 0U)
        {
          webSocket.broadcastTXT(reinterpret_cast<uint8_t *>(serialLine),
                                 serialLineLength);
        }
      }
      serialLineLength = 0U;
      serialLineOverflow = false;
    }
  }
}
}  // namespace

void setup()
{
  Serial.begin(115200, SERIAL_8N1);
  Serial.setDebugOutput(false);
  delay(100);
  sendProtocol(F("NET,BOOT"));
  startWifiAttempt();
}

void loop()
{
  serviceWifi();
  if (wifiConnected)
  {
    httpServer.handleClient();
    webSocket.loop();
  }
  serviceSerialBridge();
  if (millis() - lastHeartbeat >= kHeartbeatPeriodMs)
  {
    lastHeartbeat = millis();
    sendProtocol(String(F("HB,")) + heartbeatSequence++);
  }
  yield();
}
