# TVC Electronics

ESP32-S3 双 IMU 云台控制固件及网页控制台。接线、校准、操作流程及实物验证边界见 [飞控说明](README_云台回中.md)。

编译主控前，将 `NetworkSecrets.example.h` 复制为 `NetworkSecrets.h`，设置自己的 Wi-Fi 热点密码；该文件被 Git 忽略，不要提交。该密码需要符合 ESP32 热点的 8–63 字节要求。使用已有设备时请沿用该设备的实际配置。

Arduino IDE 中打开 `TVC_Electronics.ino`，使用 Espressif ESP32 开发板包 3.3.12、ArduinoJson 7.4.3、WebSockets 2.7.2，并按飞控说明选择 ESP32-S3 配置。修改 `web/dist` 后运行 `node tools/embed_web.mjs`，更新固件内置网页资源。当前工程不含 SuperMini 采集板固件源码。

软件构建和仿真不等于实物验证；启动前预定位的舵机到位时间是估计值，没有位置反馈。操作电机前应拆桨确认舵机方向、电调停止脉宽和实际跟随时间。
