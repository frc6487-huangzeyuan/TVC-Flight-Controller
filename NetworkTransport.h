#pragma once
#include <Arduino.h>

// 仅供本地实验平台联网，硬件交给他人使用前应修改此密码。
#include "NetworkSecrets.h"
constexpr char NETWORK_AP_PASSWORD[] = TVC_WIFI_PASSWORD;
static_assert(sizeof(NETWORK_AP_PASSWORD) >= 9 && sizeof(NETWORK_AP_PASSWORD) <= 64, "Wi-Fi password must be 8 to 63 bytes");
constexpr size_t NETWORK_MAX_COMMAND_BYTES = 1023;

// 所有网络函数都在 Arduino 的 loop 任务中执行，不要在持有
// stateLock 时调用：网络发送可能等待，而保护任务必须及时运行。
bool beginNetwork();
void pollNetwork();
bool sendNetworkFrame(uint8_t client, const String &frame);
void disconnectNetworkClient(uint8_t client);
const char *getNetworkToken();
const char *getNetworkApName();

// 主程序统一验证令牌、会话、启动编号、序号，以及唯一控制端身份。
// frame 是可修改、以空字符结尾的单条命令，只在本次回调期间有效。
extern void onNetworkFrame(uint8_t client, char *frame);
extern void onNetworkDisconnect(uint8_t client);
