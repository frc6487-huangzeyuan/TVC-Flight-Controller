# TVC Electronics

ESP32-S3 双 IMU 云台控制固件及网页控制台。接线、校准、操作流程及实物验证边界见 [飞控说明](README_云台回中.md)。

`GimbalPid` 可通过 `GimbalConfig.model` 接受外部标定的局部模型（静态偏置、重力矩阵和控制效能逆矩阵），将角加速度目标换算为推力纠偏向量，再解算舵角。`StartupPreposition` 与运行共用向量换算，保持限位和 PWM 限速。模型模式的 PID 输出单位是 °/s²，积分项和输出限幅也使用该单位；普通模式输出仍为纠偏角度。

该功能默认禁用，当前主控启动配置和网页命令不会自动启用它。TVC Studio 通过可选的 SIL 接口注入从仿真模型求得的系数；这些系数不是实物标定。当前单一机械分支、小角度标定不能保证任意方位或大角度可用。测试见 `tests/model-control.test.cpp`；主控固件仍需使用实际重心、推力和执行器标定后才能采用模型配置。设计参考 [PX4 控制分配说明](https://github.com/PX4/PX4-user_guide/blob/main/en/concept/control_allocation.md)，未直接复制其实现。

编译主控前，将 `NetworkSecrets.example.h` 复制为 `NetworkSecrets.h`，设置自己的 Wi-Fi 热点密码；该文件被 Git 忽略，不要提交。该密码需要符合 ESP32 热点的 8–63 字节要求。使用已有设备时请沿用该设备的实际配置。

Arduino IDE 中打开 `TVC_Electronics.ino`，使用 Espressif ESP32 开发板包 3.3.12、ArduinoJson 7.4.3、WebSockets 2.7.2，并按飞控说明选择 ESP32-S3 配置。修改 `web/dist` 后运行 `node tools/embed_web.mjs`，更新固件内置网页资源。当前工程不含 SuperMini 采集板固件源码。

软件构建和仿真不等于实物验证；启动前预定位的舵机到位时间是估计值，没有位置反馈。操作电机前应拆桨确认舵机方向、电调停止脉宽和实际跟随时间。
