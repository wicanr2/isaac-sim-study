/*
 * 上位協定(USART1)與 CAN 訊框的版面。橋接程式(bridge-rs/src/proto.rs)
 * 與受控體(plant/)照同一份版面實作;改這裡要同步改那兩邊。
 *
 * USART1 框包:
 *   A5 5A | len(u8,= payload 長度) | type(u8) | payload | crc16 lo hi
 *   crc16 = CRC-16/MODBUS,算 type + payload。全部 little-endian。
 */
#ifndef PROTO_H
#define PROTO_H

#include <stdint.h>

#define PROTO_SYNC0        0xA5
#define PROTO_SYNC1        0x5A
#define PROTO_MAX_PAYLOAD  32

/* host -> MCU */
#define MSG_CMD_VEL        0x01  /* i16 v_mm_s, i16 w_mrad_s */
#define MSG_PING           0x03  /* 空 payload */
/* MCU -> host */
#define MSG_ODOM           0x02  /* 見 odom_payload_t */
#define MSG_PONG           0x83  /* u8 fw_major, u8 fw_minor */

typedef struct __attribute__((packed)) {
    uint16_t seq;
    uint32_t t_ms;        /* 韌體自己的毫秒計數(SysTick) */
    int32_t  x_mm;
    int32_t  y_mm;
    int32_t  th_mrad;
    int16_t  vl_mm_s;     /* 量到的左輪速 */
    int16_t  vr_mm_s;
    uint8_t  flags;       /* 見 ODOM_FLAG_* 低 8 位;同一個 byte 也放在 CAN 0x201 的第 5 byte */
    uint8_t  flags_hi;    /* ODOM_FLAG_* 的第 8 位之後(SLIP) */
} odom_payload_t;

#define ODOM_FLAG_ENABLED   (1u << 0)   /* 馬達致能 */
#define ODOM_FLAG_ESTOP     (1u << 1)   /* PC13 急停 */
#define ODOM_FLAG_CMD_STALE (1u << 2)   /* 命令逾時(CMD_TIMEOUT_MS 沒 cmd_vel) */
#define ODOM_FLAG_DRV_FAULT (1u << 3)   /* PC14/PC15 驅動器故障(低有效)→ 停 */
#define ODOM_FLAG_BUMPER    (1u << 4)   /* PC0 保險桿 → 拒絕前進、允許後退 */
#define ODOM_FLAG_STALL     (1u << 5)   /* 堵轉:duty 高而輪不動 → 停,直到上位命令歸零 */
#define ODOM_FLAG_HB_LOST   (1u << 6)   /* 心跳逾時(HB_TIMEOUT_MS 沒 PING)→ 降速到 0 */
#define ODOM_FLAG_WDT_RESET (1u << 7)   /* 這次開機是暖重置(IWDG 或其他 reset,不是上電) */
#define ODOM_FLAG_SLIP      (1u << 8)   /* 打滑:陀螺儀的 yaw rate 與輪差推出的 yaw rate 對不上 → 只回報(上位決定),命令歸零才解 */

/* 安全功能遮罩(g_cfg.safety_mask;calib SAFETY_MASK 全開)。只給負對照用:關掉一項,對應的驗收必須紅。 */
#define SAFETY_IWDG      (1u << 0)
#define SAFETY_DRV_FAULT (1u << 1)
#define SAFETY_BUMPER    (1u << 2)
#define SAFETY_STALL     (1u << 3)
#define SAFETY_HB        (1u << 4)
#define SAFETY_SLIP      (1u << 5)   /* 打滑偵測(要有 IMU);slip-off 負對照關它 */

/* CAN 0x181 編碼器(受控體 -> MCU):i32 左累計 tick, i32 右累計 tick
 * CAN 0x201 馬達狀態(MCU -> 受控體/橋接):i16 duty_l, i16 duty_r, u8 flags, u8 seq, u16 保留 */

#define FW_VERSION_MAJOR 0
#define FW_VERSION_MINOR 1

#endif
