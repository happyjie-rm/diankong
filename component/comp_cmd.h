#pragma once

#include <stdbool.h>
#include <stdint.h>


typedef enum
{
  PENDING = 1,        ///< 等待中 | Pending
  OK = 0,             ///< 操作成功 | Operation successful
  FAILED = -1,        ///< 操作失败 | Operation failed
  INIT_ERR = -2,      ///< 初始化错误 | Initialization error
  ARG_ERR = -3,       ///< 参数错误 | Argument error
  STATE_ERR = -4,     ///< 状态错误 | State error
  SIZE_ERR = -5,      ///< 尺寸错误 | Size error
  CHECK_ERR = -6,     ///< 校验错误 | Check error
  NOT_SUPPORT = -7,   ///< 不支持 | Not supported
  NOT_FOUND = -8,     ///< 未找到 | Not found
  NO_RESPONSE = -9,   ///< 无响应 | No response
  NO_MEM = -10,       ///< 内存不足 | Insufficient memory
  NO_BUFF = -11,      ///< 缓冲区不足 | Insufficient buffer
  TIMEOUT = -12,      ///< 超时 | Timeout
  EMPTY = -13,        ///< 为空 | Empty
  FULL = -14,         ///< 已满 | Full
  BUSY = -15,         ///< 忙碌 | Busy
  PTR_NULL = -16,     ///< 空指针 | Null pointer
  OUT_OF_RANGE = -17  ///< 超出范围 | Out of range
} err_t;

typedef struct {
  float x;
  float y;
} vector2_t;

typedef struct {
  float x;
  float y;
  float z;
} vector3_t;

/* 拨杆位置 */
typedef enum {
  CMD_SW_ERR = 0,
  CMD_SW_UP = 1,
  CMD_SW_MID = 3,
  CMD_SW_DOWN = 2,
} cmd_switch_pos_t;

typedef enum {

  vt13_CMD_SW_UP = 1,
  vt13_CMD_SW_MID = 2,
  vt13_CMD_SW_DOWN = 3,
} vt13_cmd_switch_pos_t;
/* 键盘按键值 */
typedef enum {
  CMD_KEY_W = 0,
  CMD_KEY_S,
  CMD_KEY_A,
  CMD_KEY_D,
  CMD_KEY_SHIFT,
  CMD_KEY_CTRL,
  CMD_KEY_Q,
  CMD_KEY_E,
  CMD_KEY_R,
  CMD_KEY_F,
  CMD_KEY_G,
  CMD_KEY_Z,
  CMD_KEY_X,
  CMD_KEY_C,
  CMD_KEY_V,
  CMD_KEY_B,
  CMD_KEY_L_CLICK,
  CMD_KEY_R_CLICK,
  CMD_KEY_NUM,
} cmd_key_t;

typedef struct {
  uint8_t l;
  uint8_t r;
} cmd_mouse_click_t;

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
  cmd_mouse_click_t click;
} cmd_mouse_t;

typedef struct {
  struct {
    vector2_t l;
    vector2_t r;
  } ch;

  float ch_res;
  cmd_switch_pos_t sw_l;
  cmd_switch_pos_t sw_r;
  cmd_mouse_t mouse;
  uint16_t key;
  uint16_t res;
} cmd_rc_t;
