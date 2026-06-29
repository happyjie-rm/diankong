#ifndef _BSP_DWT_H
#define _BSP_DWT_H

#include "main.h"
#include "stdint.h"
// #include "bsp_log.h"

// ==================== DWT 时间类型 ====================

//! DWT 软件时间轴拆分结果。
//! s/ms/us 分别保存初始化以来的秒、毫秒余数和微秒余数。
typedef struct {
  uint32_t s;   //! 秒计数
  uint16_t ms;  //! 当前秒内的毫秒余数
  uint16_t us;  //! 当前毫秒内的微秒余数
} DWT_Time_t;

// ==================== 代码段计时辅助 ====================

//! 计算一段代码的执行耗时，单位为秒。
//! dt 由调用方提供，用于保存耗时；code 为待测代码块。
#define TIME_ELAPSE(dt, code)                                                  \
  do {                                                                         \
    float tstart = DWT_GetTimeline_s();                                        \
    code;                                                                      \
    dt = DWT_GetTimeline_s() - tstart;                                         \
    LOGINFO("[DWT] " #dt " = %f s\r\n", dt);                                   \
  } while (0)

// ==================== DWT 公开接口 ====================

//! 初始化 DWT CYCCNT，CPU_Freq_mHz 单位为 MHz。
//! 示例：C 板 168MHz，A 板 180MHz。
void DWT_Init(uint32_t CPU_Freq_mHz);

//! 获取两次调用之间的时间间隔，单位为秒。
//! cnt_last 保存上一次 CYCCNT 时间戳，函数返回前会更新该值。
float DWT_GetDeltaT(uint32_t *cnt_last);

//! 获取两次调用之间的高精度时间间隔，单位为秒。
//! cnt_last 保存上一次 CYCCNT 时间戳，函数返回前会更新该值。
double DWT_GetDeltaT64(uint32_t *cnt_last);

//! 获取初始化以来的时间轴，单位为秒。
float DWT_GetTimeline_s(void);

//! 获取初始化以来的时间轴，单位为毫秒。
float DWT_GetTimeline_ms(void);

//! 获取初始化以来的时间轴，单位为微秒。
uint64_t DWT_GetTimeline_us(void);

//! DWT 忙等待延时，Delay 单位为秒。
//! 不依赖 SysTick，可在临界区或关闭中断期间替代 HAL_Delay()。
void DWT_Delay(float Delay);

//! 更新软件时间轴，三个 timeline 读取函数都会调用它。
//! 若长时间不读取 timeline，应手动调用以避免 CYCCNT 溢出后时间轴滞后。
void DWT_SysTimeUpdate(void);

#endif /* BSP_DWT_H_ */
