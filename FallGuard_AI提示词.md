# ESP32-S3 智能跌倒检测网关开发提示词

## 角色定义
你是一名资深嵌入式系统工程师，精通 ESP-IDF（v5.x）、FreeRTOS、
网络协议栈与 IoT 云平台对接。代码风格严谨，注重健壮性、可维护性
与低功耗设计，所有功能模块均需完整实现，不留占位符。

---

## 项目背景
基于 ESP32-S3 + LD6002C 毫米波雷达模组，开发一套跌倒检测物联网
网关固件。雷达驱动已完成（ld6002c.c / ld6002c.h），通过 UART 以
TinyFrame 协议通信，所有雷达数据通过回调函数上报。

---

## 硬件资源
- SoC      : ESP32-S3
- 雷达     : LD6002C（UART1，TX=GPIO17，RX=GPIO18）
- 按键     : GPIO0（低电平触发，内部上拉）
- 状态 LED : GPIO2（可选，用于指示运行状态）
- Flash    : 4MB（默认分区表）

---

## 功能需求

### 1. 网络配置模块（wifi_manager）

#### 1.1 首次上电 / 无配置时 —— AP 配网模式
- 上电后检查 NVS 中是否存有 WiFi 凭据
- 若无凭据，立即开启 SoftAP：
  - SSID     : FallGuard-XXXX（后四位为 MAC 末两字节十六进制）
  - Password : 12345678
  - 信道     : 6
  - 最大连接 : 1（单客户端）
- 同时启动 HTTP 配网服务器（端口 80），提供以下接口：
  - GET  /         → 返回配网页面 HTML（内嵌 CSS，无需外部资源）
  - POST /connect  → 接收 JSON {"ssid":"xxx","password":"xxx"}
  - GET  /status   → 返回当前连接状态 JSON
- 配网页面需美观，包含 SSID 输入、密码输入、连接按钮、状态显示
- 收到配置后将凭据写入 NVS，尝试连接 STA，成功后关闭 AP 和 HTTP 服务

#### 1.2 已有凭据时 —— STA 模式自动连接
- 读取 NVS 凭据，启动 STA 连接
- 连接超时设定：10 秒
- 断线重连策略：
  - 指数退避：第1次等5s，第2次等10s，第3次等20s
  - 三次均失败后：清除 NVS 凭据，重启进入 AP 配网模式
  - 重连期间持续上报 LED 快闪（250ms 周期）

#### 1.3 网络状态机
使用枚举定义以下状态，并通过 FreeRTOS EventGroup 驱动：
  WIFI_STATE_IDLE / AP_MODE / STA_CONNECTING / STA_CONNECTED /
  STA_DISCONNECTED / STA_RECONNECTING / STA_FAILED

---

### 2. MQTT 模块（mqtt_client_manager）

#### 2.1 连接配置（占位符，后期填写）
```c
#define MQTT_BROKER_URI      "mqtt://YOUR_SERVER_IP:1883"
#define MQTT_USERNAME        "YOUR_USERNAME"
#define MQTT_PASSWORD        "YOUR_PASSWORD"
#define MQTT_CLIENT_ID_FMT   "fallguard_%s"   // %s = MAC后6位
#define MQTT_KEEPALIVE_SEC   60
#define MQTT_QOS_DATA        1
#define MQTT_QOS_EVENT       1
```

#### 2.2 Topic 规划（占位符，后期填写）
```c
#define TOPIC_FALL_STATUS    "fallguard/{device_id}/fall"
#define TOPIC_RADAR_PARAMS   "fallguard/{device_id}/params"
#define TOPIC_HUMAN_STATUS   "fallguard/{device_id}/human"
#define TOPIC_HEARTBEAT      "fallguard/{device_id}/heartbeat"
#define TOPIC_3D_CLOUD       "fallguard/{device_id}/cloud"
#define TOPIC_CMD_SUB        "fallguard/{device_id}/cmd"  // 下行命令订阅
```

#### 2.3 上报数据（JSON 格式占位，后期由用户确认）
所有数据包均包含公共字段：
```json
{
  "device_id": "fallguard_AABBCC",
  "timestamp": 1710000000,
  "data": { ... }
}
```
具体 data 字段结构留作占位，等待用户后续定义，
但需封装好 `mqtt_publish_fall()` / `mqtt_publish_human()`
/ `mqtt_publish_heartbeat()` / `mqtt_publish_3d_cloud()` 等接口。

#### 2.4 断线重连
- MQTT 断线后自动重连，最大重试间隔 60 秒
- WiFi 未连接时不尝试 MQTT 连接，等待 WiFi 就绪事件

#### 2.5 下行命令处理
订阅 TOPIC_CMD_SUB，解析以下命令并调用对应雷达 API：
```json
{"cmd": "set_height",      "value": 2.4}
{"cmd": "set_threshold",   "value": 0.6}
{"cmd": "set_sensitivity", "value": 10}
{"cmd": "set_alarm_zone",  "XL":1.5,"XR":1.5,"ZF":1.5,"ZB":1.5}
{"cmd": "get_params"}
{"cmd": "reboot"}
```

---

### 3. 按键模块（button_manager）

- GPIO0，内部上拉，低电平有效
- 采用 FreeRTOS 软件定时器去抖（20ms）
- 事件定义：
  - 短按（< 2s）   : 打印当前设备状态到串口日志
  - 长按（≥ 5s）   : 清除 NVS WiFi 凭据，重启进入 AP 配网
  - 超长按（≥ 10s）: 恢复出厂设置（清除全部 NVS），重启

---

### 4. LED 状态指示（led_indicator）

| 状态                | 闪烁模式            |
|---------------------|---------------------|
| AP 配网模式          | 慢闪 1Hz            |
| STA 连接中           | 快闪 4Hz            |
| STA 已连接/MQTT正常  | 每3秒呼吸一次        |
| MQTT 断线            | 双闪（100ms间隔）    |
| 跌倒报警             | 急闪 10Hz，持续3秒   |
| 系统错误             | 常亮                |

使用 FreeRTOS 任务驱动，LED 模式可通过队列动态切换。

---

### 5. 心跳与看门狗

- 每 30 秒向 MQTT 发送一次心跳包，包含：
  设备在线时长、WiFi RSSI、空闲堆大小、雷达连接状态
- 启用 ESP-IDF 硬件看门狗（TWDT），超时 10 秒
- 各关键任务定期喂狗

---

### 6. 系统日志与调试

- 使用 ESP_LOG 分模块输出（TAG 区分各模块）
- 编译时可通过 menuconfig 控制日志等级
- 串口日志波特率：115200

---

## 工程结构要求

```
project/
├── main/
│   ├── main.c                  # 入口，初始化各模块
│   ├── CMakeLists.txt
├── components/
│   ├── ld6002c/                # 已有雷达驱动
│   │   ├── ld6002c.c
│   │   ├── ld6002c.h
│   │   └── CMakeLists.txt
│   ├── wifi_manager/           # WiFi + AP配网 + HTTP服务
│   │   ├── wifi_manager.c
│   │   ├── wifi_manager.h
│   │   └── CMakeLists.txt
│   ├── mqtt_client_manager/    # MQTT封装
│   │   ├── mqtt_client_manager.c
│   │   ├── mqtt_client_manager.h
│   │   └── CMakeLists.txt
│   ├── button_manager/         # 按键检测
│   │   ├── button_manager.c
│   │   ├── button_manager.h
│   │   └── CMakeLists.txt
│   └── led_indicator/          # LED指示
│       ├── led_indicator.c
│       ├── led_indicator.h
│       └── CMakeLists.txt
└── CMakeLists.txt
```

---

## 编码规范

1. 所有模块提供 `xxx_init()` / `xxx_deinit()` 接口
2. 模块间通过 FreeRTOS EventGroup / Queue 通信，禁止全局变量跨模块共享
3. 所有 `esp_err_t` 返回值必须处理，关键路径使用 `ESP_ERROR_CHECK`
4. NVS 操作封装到独立函数，key 名称使用宏定义
5. 字符串拼接使用 `snprintf`，禁止 `sprintf`
6. 所有动态内存分配后检查 NULL，失败时打印错误并安全退出
7. 头文件使用 `#ifndef` 守卫，`extern "C"` 包裹

---

## 输出要求

按以下顺序逐文件输出，每个文件完整可编译，不省略任何函数体：

1. `components/wifi_manager/wifi_manager.h`
2. `components/wifi_manager/wifi_manager.c`
3. `components/mqtt_client_manager/mqtt_client_manager.h`
4. `components/mqtt_client_manager/mqtt_client_manager.c`
5. `components/button_manager/button_manager.h`
6. `components/button_manager/button_manager.c`
7. `components/led_indicator/led_indicator.h`
8. `components/led_indicator/led_indicator.c`
9. `main/main.c`
10. 各组件 `CMakeLists.txt`
11. 根目录 `CMakeLists.txt`

每个文件开头注明文件路径、功能说明、依赖的 IDF 组件。
