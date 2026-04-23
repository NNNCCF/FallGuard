/**
 * @file ld6002c.h
 * @brief LD6002C 跌倒检测雷达模组驱动 (TinyFrame 协议)
 *
 * 适用平台: ESP32-S3 / ESP-IDF
 * 串口参数: 115200, 8N1, 无硬件流控
 *
 * 通信角色:
 *   客户端 (上位机) = ESP32S3
 *   服务端 (模组)   = LD6002C 雷达
 */

#ifndef LD6002C_H
#define LD6002C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/* ============================================================
 * 宏定义
 * ============================================================ */

/** TF 帧起始字节，固定为 0x01 */
#define LD6002C_SOF              (0x01U)

/** 默认 UART 端口 */
#define LD6002C_UART_PORT        (UART_NUM_1)

/** 默认 TX/RX 引脚（用户可在 ld6002c_config_t 中覆盖） */
#define LD6002C_DEFAULT_TX_PIN   (17)
#define LD6002C_DEFAULT_RX_PIN   (18)

/** UART 波特率 */
#define LD6002C_BAUD_RATE        (115200)

/** UART RX 环形缓冲大小 */
#define LD6002C_UART_BUF_SIZE    (1024)

/** 接收任务栈大小 */
#define LD6002C_TASK_STACK_SIZE  (4096)

/** 接收任务优先级 */
#define LD6002C_TASK_PRIORITY    (10)

/** 同步等待超时 (ms)，用于带返回值的命令 */
#define LD6002C_SYNC_TIMEOUT_MS  (500)

/** 最大 DATA 区长度（协议最大帧 1024 字节，头 8 字节，尾 1 字节） */
#define LD6002C_MAX_DATA_LEN     (1015)

/* ============================================================
 * 消息类型定义（TYPE 字段，大端序存储，但比较时用 uint16_t）
 * ============================================================ */
typedef enum {
    /* 通用 */
    MSG_TYPE_QUERY_FW_STATUS    = 0xFFFF,  /*!< 查询固件状态 (主动下发) */
    MSG_TYPE_REPORT_FW_STATUS   = 0xFFFF,  /*!< 返回固件状态 (被动上传) */
    MSG_TYPE_ENTER_OTA          = 0x3000,  /*!< 进入 OTA 升级 (主动下发) */
    MSG_TYPE_OTA_STATUS         = 0x3000,  /*!< OTA 状态 (被动上传) */

    /* 跌倒检测项目 */
    MSG_TYPE_FALL_STATUS        = 0x0E02,  /*!< 报告跌倒状态 (主动上传) */
    MSG_TYPE_SET_HEIGHT         = 0x0E04,  /*!< 设置安装高度 (双向) */
    MSG_TYPE_GET_PARAMS         = 0x0E06,  /*!< 获取/返回雷达参数 (双向) */
    MSG_TYPE_SET_THRESHOLD      = 0x0E08,  /*!< 设置跌倒阈值 (双向) */
    MSG_TYPE_SET_SENSITIVITY    = 0x0E0A,  /*!< 设置跌倒灵敏度 (双向) */
    MSG_TYPE_SET_ALARM_ZONE     = 0x0E0C,  /*!< 设置报警区域参数 (双向) */
    MSG_TYPE_HEIGHT_UPLOAD      = 0x0E0E,  /*!< 高度上传结果 (主动上传) */
    MSG_TYPE_USER_LOG           = 0x010E,  /*!< 打开/关闭 User log (单向) */
    MSG_TYPE_RADAR_INIT         = 0x2110,  /*!< 雷达初始化设置参数 (单向) */
    MSG_TYPE_3D_CLOUD           = 0x0A08,  /*!< 报告 3D 点云数据 (主动上传) */
    MSG_TYPE_HUMAN_STATUS       = 0x0F09,  /*!< 报告有无人检测结果 (主动上传) */
} ld6002c_msg_type_t;

/* ============================================================
 * 数据结构
 * ============================================================ */

/**
 * @brief 固件项目类型
 */
typedef enum {
    PROJECT_PRESENCE    = 0, /*!< 存在感知 */
    PROJECT_BREATH      = 1, /*!< 呼吸检测 */
    PROJECT_GESTURE     = 2, /*!< 手势检测 */
    PROJECT_RANGING     = 3, /*!< 测距 */
    PROJECT_PEOPLE_CNT  = 4, /*!< 人员计数 */
    PROJECT_3D_CLOUD    = 5, /*!< 3D 点云检测 */
} ld6002c_project_t;

/**
 * @brief 固件状态
 */
typedef struct {
    ld6002c_project_t project;  /*!< 当前运行项目 */
    uint8_t major;              /*!< 主版本 */
    uint8_t sub;                /*!< 子版本 */
    uint8_t modified;           /*!< 修订版本 */
} ld6002c_fw_status_t;

/**
 * @brief 跌倒状态
 */
typedef struct {
    bool is_fall;               /*!< true = 跌倒，false = 正常 */
} ld6002c_fall_status_t;

/**
 * @brief 雷达参数（来自 0x0E06 返回）
 */
typedef struct {
    float    high;         /*!< 安装高度 (m) */
    float    threshold;    /*!< 跌倒阈值 (m) */
    uint32_t sensitivity;  /*!< 灵敏度 (3~30) */
    float    rect_XL;      /*!< 报警区域左边界 (m) */
    float    rect_XR;      /*!< 报警区域右边界 (m) */
    float    rect_ZF;      /*!< 报警区域前边界 (m) */
    float    rect_ZB;      /*!< 报警区域后边界 (m) */
} ld6002c_params_t;

/**
 * @brief 高度上传结果
 */
typedef struct {
    uint32_t value; /*!< 当前目标点高度（雷达坐标系，0 = 雷达位置） */
} ld6002c_height_upload_t;

/**
 * @brief 3D 点云单点数据
 */
typedef struct {
    int32_t cluster_index; /*!< 聚类目标 ID */
    float   x;             /*!< x 坐标 (m) */
    float   y;             /*!< y 坐标 (m) */
    float   z;             /*!< z 坐标 (m) */
    float   speed;         /*!< 速度 (m/s) */
} ld6002c_point_t;

/**
 * @brief 3D 点云帧
 */
typedef struct {
    int32_t         target_num;                /*!< 目标个数 */
    ld6002c_point_t points[32];                /*!< 最多 32 个点（可按需扩展） */
} ld6002c_3d_cloud_t;

/**
 * @brief 有无人状态
 */
typedef struct {
    bool is_human; /*!< true = 有人，false = 无人 */
} ld6002c_human_status_t;

/**
 * @brief OTA 状态
 */
typedef struct {
    uint8_t code; /*!< 0xFF = 待升级 */
} ld6002c_ota_status_t;

/* ============================================================
 * 回调函数类型
 * ============================================================ */

/** 跌倒状态回调 */
typedef void (*ld6002c_fall_cb_t)(const ld6002c_fall_status_t *status);

/** 固件状态回调 */
typedef void (*ld6002c_fw_cb_t)(const ld6002c_fw_status_t *status);

/** 参数获取回调（0x0E06 返回） */
typedef void (*ld6002c_params_cb_t)(const ld6002c_params_t *params);

/** 高度上传回调 */
typedef void (*ld6002c_height_cb_t)(const ld6002c_height_upload_t *result);

/** 3D 点云回调 */
typedef void (*ld6002c_3d_cloud_cb_t)(const ld6002c_3d_cloud_t *cloud);

/** 有无人回调 */
typedef void (*ld6002c_human_cb_t)(const ld6002c_human_status_t *status);

/** 通用操作结果回调（set/get 成功/失败） result: 0=失败,1=成功 */
typedef void (*ld6002c_result_cb_t)(uint8_t result);

/**
 * @brief 所有回调集合，不使用的设为 NULL
 */
typedef struct {
    ld6002c_fall_cb_t     on_fall_status;    /*!< 跌倒状态上传 */
    ld6002c_fw_cb_t       on_fw_status;      /*!< 固件状态返回 */
    ld6002c_params_cb_t   on_params;         /*!< 参数获取结果 */
    ld6002c_result_cb_t   on_set_height;     /*!< 设置安装高度结果 */
    ld6002c_result_cb_t   on_set_threshold;  /*!< 设置阈值结果 */
    ld6002c_result_cb_t   on_set_sensitivity;/*!< 设置灵敏度结果 */
    ld6002c_result_cb_t   on_set_alarm_zone; /*!< 设置报警区域结果 */
    ld6002c_height_cb_t   on_height_upload;  /*!< 高度上传结果 */
    ld6002c_3d_cloud_cb_t on_3d_cloud;       /*!< 3D 点云数据 */
    ld6002c_human_cb_t    on_human_status;   /*!< 有无人状态 */
} ld6002c_callbacks_t;

/* ============================================================
 * 初始化配置
 * ============================================================ */

/**
 * @brief 驱动初始化配置
 */
typedef struct {
    uart_port_t          uart_port;  /*!< UART 端口号 */
    int                  tx_pin;     /*!< TX GPIO */
    int                  rx_pin;     /*!< RX GPIO */
    ld6002c_callbacks_t  callbacks;  /*!< 回调函数集 */
} ld6002c_config_t;

/* ============================================================
 * 公共 API
 * ============================================================ */

/**
 * @brief 初始化驱动（配置 UART，启动接收任务）
 * @param cfg 配置结构体指针
 * @return ESP_OK / 错误码
 */
esp_err_t ld6002c_init(const ld6002c_config_t *cfg);

/**
 * @brief 反初始化驱动（停止任务，释放资源）
 */
void ld6002c_deinit(void);

/**
 * @brief 查询固件状态 (TYPE:0xFFFF)
 * @return ESP_OK / 错误码
 */
esp_err_t ld6002c_query_fw_status(void);

/**
 * @brief 进入 OTA 升级 (TYPE:0x3000)
 * @return ESP_OK / 错误码
 */
esp_err_t ld6002c_enter_ota(void);

/**
 * @brief 设置雷达安装高度 (TYPE:0x0E04)
 * @param height_m 安装高度，单位米，范围 1~5
 * @return ESP_OK / 错误码
 */
esp_err_t ld6002c_set_height(float height_m);

/**
 * @brief 获取雷达所有参数 (TYPE:0x0E06)
 * @return ESP_OK / 错误码，结果通过 on_params 回调返回
 */
esp_err_t ld6002c_get_params(void);

/**
 * @brief 设置跌倒阈值 (TYPE:0x0E08)
 * @param threshold_m 跌倒阈值（米），默认 0.6，地面高度为 0m
 * @return ESP_OK / 错误码
 */
esp_err_t ld6002c_set_threshold(float threshold_m);

/**
 * @brief 设置跌倒灵敏度 (TYPE:0x0E0A)
 * @param sensitivity 灵敏度，范围 3~30，初始值 10
 * @return ESP_OK / 错误码
 */
esp_err_t ld6002c_set_sensitivity(uint32_t sensitivity);

/**
 * @brief 设置报警区域参数 (TYPE:0x0E0C)
 * @param rect_XL 左边界 (m)，范围 0.3~1.5
 * @param rect_XR 右边界 (m)，范围 0.3~1.5
 * @param rect_ZF 前边界 (m)，范围 0.3~1.5
 * @param rect_ZB 后边界 (m)，范围 0.3~1.5
 * @return ESP_OK / 错误码
 */
esp_err_t ld6002c_set_alarm_zone(float rect_XL, float rect_XR,
                                  float rect_ZF, float rect_ZB);

/**
 * @brief 打开或关闭 User log (TYPE:0x010E)
 * @param enable true=打开，false=关闭
 * @return ESP_OK / 错误码
 */
esp_err_t ld6002c_set_user_log(bool enable);

/**
 * @brief 雷达初始化设置参数 (TYPE:0x2110)
 *        发送后雷达以内置默认值初始化（High:2.4, Threshold:0.6 等）
 * @return ESP_OK / 错误码
 */
esp_err_t ld6002c_radar_init_params(void);

#ifdef __cplusplus
}
#endif

#endif /* LD6002C_H */
