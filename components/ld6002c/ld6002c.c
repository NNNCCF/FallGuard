/**
 * @file ld6002c.c
 * @brief LD6002C 跌倒检测雷达模组驱动实现 (TinyFrame 协议)
 *
 * 适用平台: ESP32-S3 / ESP-IDF
 *
 * 协议要点:
 *  - 帧头(SOF) ~ HEAD_CKSUM 采用大端序
 *  - DATA 采用小端序
 *  - HEAD_CKSUM: SOF^ID[0]^ID[1]^LEN[0]^LEN[1]^TYPE[0]^TYPE[1]，再取反
 *  - DATA_CKSUM: DATA 所有字节异或，再取反
 */

#include "ld6002c.h"

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* ============================================================
 * 内部宏
 * ============================================================ */
static const char *TAG = "LD6002C";

/** 帧头固定部分字节数: SOF(1)+ID(2)+LEN(2)+TYPE(2)+HEAD_CKSUM(1) = 8 */
#define TF_HEADER_LEN  (8U)
/** DATA_CKSUM 字节数 */
#define TF_TAIL_LEN    (1U)

/* ============================================================
 * 内部状态机枚举
 * ============================================================ */
typedef enum {
    PARSE_SOF = 0,
    PARSE_ID_0,
    PARSE_ID_1,
    PARSE_LEN_0,
    PARSE_LEN_1,
    PARSE_TYPE_0,
    PARSE_TYPE_1,
    PARSE_HEAD_CKSUM,
    PARSE_DATA,
    PARSE_DATA_CKSUM,
} tf_parse_state_t;

/* ============================================================
 * 内部帧结构
 * ============================================================ */
typedef struct {
    uint16_t id;
    uint16_t len;          /*!< DATA 区字节数 */
    uint16_t type;
    uint8_t  head_cksum;
    uint8_t  data[LD6002C_MAX_DATA_LEN];
    uint8_t  data_cksum;
} tf_frame_t;

/* ============================================================
 * 驱动上下文
 * ============================================================ */
typedef struct {
    uart_port_t         uart_port;
    ld6002c_callbacks_t cb;
    TaskHandle_t        rx_task;
    uint16_t            tx_id;          /*!< 递增帧 ID */
    SemaphoreHandle_t   tx_mutex;       /*!< 发送互斥量 */
    bool                initialized;
} ld6002c_ctx_t;

static ld6002c_ctx_t s_ctx = {0};

/* ============================================================
 * 内部工具函数
 * ============================================================ */

/**
 * @brief 计算 CKSUM: 所有字节异或后取反
 */
static uint8_t calc_cksum(const uint8_t *data, uint16_t len)
{
    uint8_t ret = 0;
    for (uint16_t i = 0; i < len; i++) {
        ret ^= data[i];
    }
    return ~ret;
}

/**
 * @brief 从小端序字节流读取 float
 */
static float read_float_le(const uint8_t *p)
{
    uint32_t u;
    memcpy(&u, p, 4);
    float f;
    memcpy(&f, &u, 4);
    return f;
}

/**
 * @brief 从小端序字节流读取 uint32
 */
static uint32_t read_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

/**
 * @brief 从小端序字节流读取 int32
 */
static int32_t read_i32_le(const uint8_t *p)
{
    return (int32_t)read_u32_le(p);
}

/**
 * @brief 将 float 以小端序写入缓冲区
 */
static void write_float_le(uint8_t *p, float val)
{
    uint32_t u;
    memcpy(&u, &val, 4);
    p[0] = (uint8_t)(u & 0xFF);
    p[1] = (uint8_t)((u >> 8) & 0xFF);
    p[2] = (uint8_t)((u >> 16) & 0xFF);
    p[3] = (uint8_t)((u >> 24) & 0xFF);
}

/**
 * @brief 将 uint32 以小端序写入缓冲区
 */
static void write_u32_le(uint8_t *p, uint32_t val)
{
    p[0] = (uint8_t)(val & 0xFF);
    p[1] = (uint8_t)((val >> 8) & 0xFF);
    p[2] = (uint8_t)((val >> 16) & 0xFF);
    p[3] = (uint8_t)((val >> 24) & 0xFF);
}

/* ============================================================
 * 发送函数
 * ============================================================ */

/**
 * @brief 构建并发送一帧 TF 数据
 *
 * @param type   消息类型
 * @param data   DATA 区指针（小端序，可为 NULL 若 len=0）
 * @param len    DATA 区字节数
 * @return ESP_OK / 错误码
 */
static esp_err_t tf_send(uint16_t type, const uint8_t *data, uint16_t len)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (len > LD6002C_MAX_DATA_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 总帧长: 8(头) + len(DATA) + 1(DATA_CKSUM)，若 len=0 则无 DATA 和 DATA_CKSUM */
    uint16_t frame_len = TF_HEADER_LEN + (len > 0 ? (len + TF_TAIL_LEN) : 0);
    uint8_t buf[TF_HEADER_LEN + LD6002C_MAX_DATA_LEN + TF_TAIL_LEN];

    xSemaphoreTake(s_ctx.tx_mutex, portMAX_DELAY);

    uint16_t id = s_ctx.tx_id++;

    /* --- 填充帧头（大端序） --- */
    buf[0] = LD6002C_SOF;
    buf[1] = (uint8_t)((id >> 8) & 0xFF);
    buf[2] = (uint8_t)(id & 0xFF);
    buf[3] = (uint8_t)((len >> 8) & 0xFF);
    buf[4] = (uint8_t)(len & 0xFF);
    buf[5] = (uint8_t)((type >> 8) & 0xFF);
    buf[6] = (uint8_t)(type & 0xFF);
    /* HEAD_CKSUM = ~(SOF ^ ID0 ^ ID1 ^ LEN0 ^ LEN1 ^ TYPE0 ^ TYPE1) */
    buf[7] = calc_cksum(buf, 7);

    uint16_t offset = TF_HEADER_LEN;

    if (len > 0 && data != NULL) {
        memcpy(&buf[offset], data, len);
        offset += len;
        buf[offset] = calc_cksum(data, len);
        offset += TF_TAIL_LEN;
    }

    int written = uart_write_bytes(s_ctx.uart_port, (const char *)buf, frame_len);
    xSemaphoreGive(s_ctx.tx_mutex);

    if (written != (int)frame_len) {
        ESP_LOGE(TAG, "uart_write_bytes failed: written=%d expected=%d", written, frame_len);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "TX type=0x%04X id=%04X len=%d", type, id, len);
    return ESP_OK;
}

/* ============================================================
 * 帧解析与分发
 * ============================================================ */

/**
 * @brief 解析并分发一帧完整数据
 */
static void tf_dispatch(const tf_frame_t *frame)
{
    ESP_LOGD(TAG, "RX type=0x%04X id=%04X len=%d", frame->type, frame->id, frame->len);

    switch (frame->type) {

    /* ---------- 0xFFFF 固件状态 ---------- */
    case MSG_TYPE_QUERY_FW_STATUS: {
        if (frame->len < 4) {
            ESP_LOGW(TAG, "FW status frame too short: %d", frame->len);
            break;
        }
        if (s_ctx.cb.on_fw_status) {
            ld6002c_fw_status_t st = {
                .project  = (ld6002c_project_t)frame->data[0],
                .major    = frame->data[1],
                .sub      = frame->data[2],
                .modified = frame->data[3],
            };
            s_ctx.cb.on_fw_status(&st);
        }
        break;
    }

    /* ---------- 0x0E02 跌倒状态 ---------- */
    case MSG_TYPE_FALL_STATUS: {
        if (frame->len < 1) break;
        if (s_ctx.cb.on_fall_status) {
            ld6002c_fall_status_t st = {
                .is_fall = (frame->data[0] == 0x01),
            };
            s_ctx.cb.on_fall_status(&st);
        }
        break;
    }

    /* ---------- 0x0E04 设置安装高度 回传 ---------- */
    case MSG_TYPE_SET_HEIGHT: {
        if (frame->len < 1) break;
        /* 回传: result 0=失败 1=成功 */
        if (s_ctx.cb.on_set_height) {
            s_ctx.cb.on_set_height(frame->data[0]);
        }
        break;
    }

    /* ---------- 0x0E06 获取参数 回传 ---------- */
    case MSG_TYPE_GET_PARAMS: {
        /* 7 个字段: high(float)+threshold(float)+sensitivity(u32)+4*float = 28 字节 */
        if (frame->len < 28) {
            /* 若 len=0 表示获取失败 */
            if (s_ctx.cb.on_params) {
                s_ctx.cb.on_params(NULL);
            }
            break;
        }
        if (s_ctx.cb.on_params) {
            ld6002c_params_t p;
            const uint8_t *d = frame->data;
            p.high        = read_float_le(d);       d += 4;
            p.threshold   = read_float_le(d);       d += 4;
            p.sensitivity = read_u32_le(d);         d += 4;
            p.rect_XL     = read_float_le(d);       d += 4;
            p.rect_XR     = read_float_le(d);       d += 4;
            p.rect_ZF     = read_float_le(d);       d += 4;
            p.rect_ZB     = read_float_le(d);
            s_ctx.cb.on_params(&p);
        }
        break;
    }

    /* ---------- 0x0E08 设置阈值 回传 ---------- */
    case MSG_TYPE_SET_THRESHOLD: {
        if (frame->len < 1) break;
        if (s_ctx.cb.on_set_threshold) {
            s_ctx.cb.on_set_threshold(frame->data[0]);
        }
        break;
    }

    /* ---------- 0x0E0A 设置灵敏度 回传 ---------- */
    case MSG_TYPE_SET_SENSITIVITY: {
        if (frame->len < 1) break;
        if (s_ctx.cb.on_set_sensitivity) {
            s_ctx.cb.on_set_sensitivity(frame->data[0]);
        }
        break;
    }

    /* ---------- 0x0E0C 设置报警区域 回传 ---------- */
    case MSG_TYPE_SET_ALARM_ZONE: {
        if (frame->len < 1) break;
        if (s_ctx.cb.on_set_alarm_zone) {
            s_ctx.cb.on_set_alarm_zone(frame->data[0]);
        }
        break;
    }

    /* ---------- 0x0E0E 高度上传 ---------- */
    case MSG_TYPE_HEIGHT_UPLOAD: {
        if (frame->len < 4) break;
        if (s_ctx.cb.on_height_upload) {
            ld6002c_height_upload_t h = {
                .value = read_u32_le(frame->data),
            };
            s_ctx.cb.on_height_upload(&h);
        }
        break;
    }

    /* ---------- 0x0A08 3D 点云 ---------- */
    case MSG_TYPE_3D_CLOUD: {
        if (frame->len < 4) break;
        if (s_ctx.cb.on_3d_cloud) {
            ld6002c_3d_cloud_t cloud;
            const uint8_t *d = frame->data;
            cloud.target_num = read_i32_le(d); d += 4;
            int32_t n = cloud.target_num;
            if (n < 0) n = 0;
            if (n > 32) n = 32;  /* 安全截断 */
            /* 每个目标: cluster_index(4) + x(4) + y(4) + z(4) + speed(4) = 20 字节 */
            if (frame->len < (uint16_t)(4 + n * 20)) {
                ESP_LOGW(TAG, "3D cloud frame too short for %d points", n);
                n = 0;
                cloud.target_num = 0;
            }
            for (int i = 0; i < n; i++) {
                cloud.points[i].cluster_index = read_i32_le(d); d += 4;
                cloud.points[i].x             = read_float_le(d); d += 4;
                cloud.points[i].y             = read_float_le(d); d += 4;
                cloud.points[i].z             = read_float_le(d); d += 4;
                cloud.points[i].speed         = read_float_le(d); d += 4;
            }
            s_ctx.cb.on_3d_cloud(&cloud);
        }
        break;
    }

    /* ---------- 0x0F09 有无人状态 ---------- */
    case MSG_TYPE_HUMAN_STATUS: {
        if (frame->len < 1) break;
        if (s_ctx.cb.on_human_status) {
            ld6002c_human_status_t h = {
                .is_human = (frame->data[0] == 0x01),
            };
            s_ctx.cb.on_human_status(&h);
        }
        break;
    }

    /* ---------- 0x3000 OTA 状态 ---------- */
    case MSG_TYPE_OTA_STATUS: {
        ESP_LOGI(TAG, "OTA status code=0x%02X", frame->len > 0 ? frame->data[0] : 0);
        break;
    }

    default:
        ESP_LOGW(TAG, "Unknown frame type=0x%04X", frame->type);
        break;
    }
}

/* ============================================================
 * UART 接收任务
 * ============================================================ */

/**
 * @brief UART 字节级状态机接收任务
 *
 * 严格按照文档流程图实现，每字节进入状态机处理。
 * HEAD_CKSUM 计算：XOR(SOF, ID0, ID1, LEN0, LEN1, TYPE0, TYPE1) 再取反。
 * DATA_CKSUM 计算：XOR(DATA 所有字节) 再取反。
 */
static void ld6002c_rx_task(void *arg)
{
    tf_parse_state_t state = PARSE_SOF;
    tf_frame_t frame;
    memset(&frame, 0, sizeof(frame));

    /* HEAD_CKSUM 的滚动 XOR 累加器（SOF~TYPE 段） */
    uint8_t head_xor = 0;
    /* DATA_CKSUM 的滚动 XOR 累加器 */
    uint8_t data_xor = 0;
    /* 已接收 DATA 字节数 */
    uint16_t data_received = 0;

    uint8_t byte;

    while (1) {
        int ret = uart_read_bytes(s_ctx.uart_port, &byte, 1,
                                  pdMS_TO_TICKS(50));
        if (ret <= 0) {
            /* 超时，无数据，继续等待 */
            continue;
        }

        switch (state) {

        case PARSE_SOF:
            if (byte == LD6002C_SOF) {
                memset(&frame, 0, sizeof(frame));
                head_xor = byte;   /* XOR 初始化 */
                data_xor = 0;
                data_received = 0;
                state = PARSE_ID_0;
            }
            /* 否则丢弃，继续寻找 SOF */
            break;

        case PARSE_ID_0:
            frame.id = (uint16_t)(byte << 8);
            head_xor ^= byte;
            state = PARSE_ID_1;
            break;

        case PARSE_ID_1:
            frame.id |= byte;
            head_xor ^= byte;
            state = PARSE_LEN_0;
            break;

        case PARSE_LEN_0:
            frame.len = (uint16_t)(byte << 8);
            head_xor ^= byte;
            state = PARSE_LEN_1;
            break;

        case PARSE_LEN_1:
            frame.len |= byte;
            head_xor ^= byte;
            if (frame.len > LD6002C_MAX_DATA_LEN) {
                ESP_LOGW(TAG, "LEN too large (%d), reset parser", frame.len);
                state = PARSE_SOF;
                break;
            }
            state = PARSE_TYPE_0;
            break;

        case PARSE_TYPE_0:
            frame.type = (uint16_t)(byte << 8);
            head_xor ^= byte;
            state = PARSE_TYPE_1;
            break;

        case PARSE_TYPE_1:
            frame.type |= byte;
            head_xor ^= byte;
            state = PARSE_HEAD_CKSUM;
            break;

        case PARSE_HEAD_CKSUM: {
            /* 计算期望值：~head_xor */
            uint8_t expected = ~head_xor;
            if (byte != expected) {
                ESP_LOGW(TAG, "HEAD_CKSUM mismatch: got=0x%02X expect=0x%02X", byte, expected);
                state = PARSE_SOF;
                break;
            }
            frame.head_cksum = byte;
            if (frame.len == 0) {
                /* 无 DATA 区，帧完整 */
                tf_dispatch(&frame);
                state = PARSE_SOF;
            } else {
                data_received = 0;
                state = PARSE_DATA;
            }
            break;
        }

        case PARSE_DATA:
            frame.data[data_received++] = byte;
            data_xor ^= byte;
            if (data_received >= frame.len) {
                state = PARSE_DATA_CKSUM;
            }
            break;

        case PARSE_DATA_CKSUM: {
            uint8_t expected = ~data_xor;
            if (byte != expected) {
                ESP_LOGW(TAG, "DATA_CKSUM mismatch: got=0x%02X expect=0x%02X", byte, expected);
                state = PARSE_SOF;
                break;
            }
            frame.data_cksum = byte;
            tf_dispatch(&frame);
            state = PARSE_SOF;
            break;
        }

        default:
            state = PARSE_SOF;
            break;
        }
    }
}

/* ============================================================
 * 公共 API 实现
 * ============================================================ */

esp_err_t ld6002c_init(const ld6002c_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    /* 保存配置 */
    s_ctx.uart_port = cfg->uart_port;
    memcpy(&s_ctx.cb, &cfg->callbacks, sizeof(ld6002c_callbacks_t));
    s_ctx.tx_id = 0;

    /* 互斥量 */
    s_ctx.tx_mutex = xSemaphoreCreateMutex();
    if (s_ctx.tx_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* 配置 UART */
    uart_config_t uart_cfg = {
        .baud_rate  = LD6002C_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err;

    err = uart_param_config(cfg->uart_port, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        vSemaphoreDelete(s_ctx.tx_mutex);
        return err;
    }

    err = uart_set_pin(cfg->uart_port, cfg->tx_pin, cfg->rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        vSemaphoreDelete(s_ctx.tx_mutex);
        return err;
    }

    err = uart_driver_install(cfg->uart_port,
                              LD6002C_UART_BUF_SIZE,  /* RX buffer */
                              0,                       /* TX buffer (0=直接发) */
                              0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        vSemaphoreDelete(s_ctx.tx_mutex);
        return err;
    }

    s_ctx.initialized = true;

    /* 启动接收任务 */
    BaseType_t xret = xTaskCreate(ld6002c_rx_task, "ld6002c_rx",
                                  LD6002C_TASK_STACK_SIZE,
                                  NULL,
                                  LD6002C_TASK_PRIORITY,
                                  &s_ctx.rx_task);
    if (xret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create RX task");
        uart_driver_delete(cfg->uart_port);
        vSemaphoreDelete(s_ctx.tx_mutex);
        s_ctx.initialized = false;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "LD6002C initialized on UART%d (TX=%d, RX=%d)",
             cfg->uart_port, cfg->tx_pin, cfg->rx_pin);
    return ESP_OK;
}

void ld6002c_deinit(void)
{
    if (!s_ctx.initialized) return;

    if (s_ctx.rx_task) {
        vTaskDelete(s_ctx.rx_task);
        s_ctx.rx_task = NULL;
    }
    uart_driver_delete(s_ctx.uart_port);

    if (s_ctx.tx_mutex) {
        vSemaphoreDelete(s_ctx.tx_mutex);
        s_ctx.tx_mutex = NULL;
    }

    memset(&s_ctx, 0, sizeof(s_ctx));
    ESP_LOGI(TAG, "LD6002C deinitialized");
}

/* ---------- 通用消息 ---------- */

esp_err_t ld6002c_query_fw_status(void)
{
    /* TYPE=0xFFFF, 无 DATA */
    return tf_send(MSG_TYPE_QUERY_FW_STATUS, NULL, 0);
}

esp_err_t ld6002c_enter_ota(void)
{
    /* TYPE=0x3000, 无 DATA */
    return tf_send(MSG_TYPE_ENTER_OTA, NULL, 0);
}

/* ---------- 跌倒检测命令 ---------- */

esp_err_t ld6002c_set_height(float height_m)
{
    if (height_m < 1.0f || height_m > 5.0f) {
        ESP_LOGW(TAG, "set_height: out of range [1, 5], got %.2f", height_m);
        /* 仍发送，由雷达决定是否接受 */
    }
    uint8_t data[4];
    write_float_le(data, height_m);
    return tf_send(MSG_TYPE_SET_HEIGHT, data, 4);
}

esp_err_t ld6002c_get_params(void)
{
    /* 上位机请求：无 DATA */
    return tf_send(MSG_TYPE_GET_PARAMS, NULL, 0);
}

esp_err_t ld6002c_set_threshold(float threshold_m)
{
    uint8_t data[4];
    write_float_le(data, threshold_m);
    return tf_send(MSG_TYPE_SET_THRESHOLD, data, 4);
}

esp_err_t ld6002c_set_sensitivity(uint32_t sensitivity)
{
    if (sensitivity < 3 || sensitivity > 30) {
        ESP_LOGW(TAG, "set_sensitivity: recommended range [3, 30], got %lu",
                 (unsigned long)sensitivity);
    }
    uint8_t data[4];
    write_u32_le(data, sensitivity);
    return tf_send(MSG_TYPE_SET_SENSITIVITY, data, 4);
}

esp_err_t ld6002c_set_alarm_zone(float rect_XL, float rect_XR,
                                  float rect_ZF, float rect_ZB)
{
    uint8_t data[16];
    write_float_le(data + 0,  rect_XL);
    write_float_le(data + 4,  rect_XR);
    write_float_le(data + 8,  rect_ZF);
    write_float_le(data + 12, rect_ZB);
    return tf_send(MSG_TYPE_SET_ALARM_ZONE, data, 16);
}

esp_err_t ld6002c_set_user_log(bool enable)
{
    uint8_t data[4];
    write_u32_le(data, enable ? 0x00000001U : 0x00000000U);
    return tf_send(MSG_TYPE_USER_LOG, data, 4);
}

esp_err_t ld6002c_radar_init_params(void)
{
    /* TYPE=0x2110, 无 DATA，协议要求无数据体 */
    return tf_send(MSG_TYPE_RADAR_INIT, NULL, 0);
}
