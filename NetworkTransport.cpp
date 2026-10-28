#include "NetworkTransport.h"
#include "bench_config.h"
#include "WebAssets.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <esp_system.h>
#include <cstring>

namespace {
WebServer httpServer(80);
WebSocketsServer socketServer(81);
char networkToken[33] = "";
char networkApName[24] = "";
bool networkReady = false;
constexpr char LOCAL_ORIGIN[] = "http://192.168.4.1";
constexpr char LOCAL_WS_URL[] = "ws://192.168.4.1:81/ws";

bool validOrigin(const String &value) {
  return value == LOCAL_ORIGIN || value == "http://192.168.4.1:80";
}

bool allowHttpRequest() {
  const String host = httpServer.hostHeader();
  const String origin = httpServer.header("Origin");
  if ((host != "192.168.4.1" && host != "192.168.4.1:80") ||
      (origin.length() && !validOrigin(origin))) {
    httpServer.send(403, "text/plain; charset=utf-8", "Local bench origin only.");
    return false;
  }
  return true;
}

void serveGzip(const char *mime, const uint8_t *data, size_t length) {
  if (!allowHttpRequest()) return;
  httpServer.sendHeader("Content-Encoding", "gzip");
  httpServer.sendHeader("Cache-Control", "no-store");
  httpServer.sendHeader("X-Content-Type-Options", "nosniff");
  httpServer.send_P(200, mime, reinterpret_cast<const char *>(data), length);
}

void serveDeviceInfo() {
  if (!allowHttpRequest()) return;
  char response[320];
  snprintf(response, sizeof(response),
    "{\"token\":\"%s\",\"ssid\":\"%s\",\"ws_url\":\"%s\",\"protocol\":%u,\"firmware\":\"%s\"}",
    networkToken, networkApName, LOCAL_WS_URL, unsigned(PROTOCOL_VERSION), FW_VERSION);
  httpServer.sendHeader("Cache-Control", "no-store");
  httpServer.sendHeader("X-Content-Type-Options", "nosniff");
  httpServer.send(200, "application/json; charset=utf-8", response);
}

bool validateSocketHeader(String name, String value) {
  if (name.equalsIgnoreCase("Origin")) return validOrigin(value);
  if (name.equalsIgnoreCase("Host")) return value == "192.168.4.1:81";
  return true;
}

void socketEvent(uint8_t client, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_DISCONNECTED) {
    onNetworkDisconnect(client);
    return;
  }
  if (type == WStype_CONNECTED) {
    if (length != 3 || !payload || memcmp(payload, "/ws", 3)) socketServer.disconnect(client);
    return;
  }
  if (type == WStype_TEXT) {
    // 完整保留 JSON 与 CRC 的原始字节，只去除通信帧末尾的换行符。
    if (!payload || !length || length > NETWORK_MAX_COMMAND_BYTES) {
      socketServer.disconnect(client);
      return;
    }
    while (length && (payload[length - 1] == '\n' || payload[length - 1] == '\r')) --length;
    if (!length) { socketServer.disconnect(client); return; }
    for (size_t i = 0; i < length; ++i) {
      if (!payload[i] || payload[i] == '\n' || payload[i] == '\r') {
        socketServer.disconnect(client);
        return;
      }
    }
    char frame[NETWORK_MAX_COMMAND_BYTES + 1];
    memcpy(frame, payload, length);
    frame[length] = 0;
    onNetworkFrame(client, frame);
    return;
  }
  if (type == WStype_BIN || type == WStype_FRAGMENT_TEXT_START ||
      type == WStype_FRAGMENT_BIN_START || type == WStype_FRAGMENT ||
      type == WStype_FRAGMENT_FIN || type == WStype_ERROR) {
    socketServer.disconnect(client);
  }
  // 浏览器 WebSocket 的 ping/pong 只检测连接健康，不代替执行器控制心跳。
}
} // 匿名命名空间结束。

bool beginNetwork() {
  if (networkReady) return true;
  snprintf(networkApName, sizeof(networkApName), "TVC-Bench-%04X",
           unsigned(ESP.getEfuseMac() & 0xffff));
  if (!WiFi.mode(WIFI_AP) ||
      !WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0)) ||
      !WiFi.softAP(networkApName, NETWORK_AP_PASSWORD, 1, 0, 2)) return false;
  // Wi-Fi 无线模块工作时，为 esp_random() 提供硬件随机熵。
  snprintf(networkToken, sizeof(networkToken), "%08lx%08lx%08lx%08lx",
           (unsigned long)esp_random(), (unsigned long)esp_random(),
           (unsigned long)esp_random(), (unsigned long)esp_random());

  static const char *httpHeaders[] = {"Origin"};
  httpServer.collectHeaders(httpHeaders, 1);
  httpServer.on("/", HTTP_GET, [] { serveGzip("text/html; charset=utf-8", WEB_INDEX_GZ, sizeof(WEB_INDEX_GZ)); });
  httpServer.on("/index.html", HTTP_GET, [] { serveGzip("text/html; charset=utf-8", WEB_INDEX_GZ, sizeof(WEB_INDEX_GZ)); });
  httpServer.on("/app.bundle.js", HTTP_GET, [] { serveGzip("application/javascript; charset=utf-8", WEB_BUNDLE_GZ, sizeof(WEB_BUNDLE_GZ)); });
  httpServer.on("/app.mjs", HTTP_GET, [] { serveGzip("application/javascript; charset=utf-8", WEB_APP_GZ, sizeof(WEB_APP_GZ)); });
  httpServer.on("/protocol.mjs", HTTP_GET, [] { serveGzip("application/javascript; charset=utf-8", WEB_PROTOCOL_GZ, sizeof(WEB_PROTOCOL_GZ)); });
  httpServer.on("/preview.mjs", HTTP_GET, [] { serveGzip("application/javascript; charset=utf-8", WEB_PREVIEW_GZ, sizeof(WEB_PREVIEW_GZ)); });
  httpServer.on("/style.css", HTTP_GET, [] { serveGzip("text/css; charset=utf-8", WEB_STYLE_GZ, sizeof(WEB_STYLE_GZ)); });
  httpServer.on("/api/device", HTTP_GET, serveDeviceInfo);
  httpServer.onNotFound([] { httpServer.send(404, "text/plain; charset=utf-8", "Not found"); });
  httpServer.begin();

  // 不要只在主程序里减小 WEBSOCKETS_SERVER_CLIENT_MAX，库与主程序
  // 必须使用相同的类布局；单个控制端的所有权限制由主程序执行。
  static const char *socketHeaders[] = {"Origin", "Host"};
  socketServer.onValidateHttpHeader(validateSocketHeader, socketHeaders, 2);
  socketServer.onEvent(socketEvent);
  socketServer.enableHeartbeat(2000, 1000, 2);
  socketServer.begin();
  networkReady = true;
  return true;
}

void pollNetwork() {
  if (!networkReady) return;
  httpServer.handleClient();
  socketServer.loop();
}

bool sendNetworkFrame(uint8_t client, const String &frame) {
  return networkReady && socketServer.clientIsConnected(client) &&
         socketServer.sendTXT(client, frame.c_str(), frame.length());
}

void disconnectNetworkClient(uint8_t client) {
  if (networkReady) socketServer.disconnect(client);
}

const char *getNetworkToken() { return networkToken; }
const char *getNetworkApName() { return networkApName; }
