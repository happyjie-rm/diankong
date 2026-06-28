#include "bsp_dwt.h"

#include "cmsis_os.h"

// ==================== DWT 时间基准状态 ====================

//! 当前拆分后的系统时间，单位为秒/毫秒/微秒。
static DWT_Time_t SysTime;
//! CPU 主频及常用毫秒、微秒换算因子。
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
//! CYCCNT 32 位计数器溢出次数。
static uint32_t CYCCNT_RountCount;
//! 上一次读取到的 CYCCNT 值，用于判断是否发生回绕。
static uint32_t CYCCNT_LAST;
//! 64 位软件扩展计数器，结合溢出次数计算总周期数。
static uint64_t CYCCNT64;

// ==================== CYCCNT 溢出维护 ====================

//! 检查 DWT CYCCNT 是否回绕，并在回绕时更新软件溢出计数。
//! 约束：两次调用间隔不能超过一次 32 位 CYCCNT 溢出周期。
static void DWT_CNT_Update(void)
{
    //! 简单位锁：避免任务和中断同时更新溢出计数导致重复累计。
    static volatile uint8_t bit_locker = 0;
    if (!bit_locker) {
        bit_locker                = 1;
        volatile uint32_t cnt_now = DWT->CYCCNT;
        if (cnt_now < CYCCNT_LAST)
            CYCCNT_RountCount++;

        CYCCNT_LAST = DWT->CYCCNT;
        bit_locker  = 0;
    }
}

// ==================== 初始化与时间差接口 ====================

//! 初始化 DWT CYCCNT 计数器，并保存 CPU 主频换算因子。
void DWT_Init(uint32_t CPU_Freq_mHz)
{
    //! 使能 DWT/ITM 调试跟踪外设访问。
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    //! 清零并启动 Cortex-M DWT CYCCNT 周期计数器。
    DWT->CYCCNT = (uint32_t)0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz       = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms    = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us    = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;

    DWT_CNT_Update();
}

//! 获取两次调用之间的秒级时间差，并更新调用方保存的上次计数值。
float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt                  = ((float)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last                 = cnt_now;

    DWT_CNT_Update();

    return dt;
}

//! 获取两次调用之间的秒级时间差，使用 double 提升返回精度。
double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt                 = ((double)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last                 = cnt_now;

    DWT_CNT_Update();

    return dt;
}

// ==================== 时间轴读取接口 ====================

//! 刷新软件时间轴，把 CYCCNT64 拆分为秒、毫秒和微秒字段。
void DWT_SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    DWT_CNT_Update();

    CYCCNT64   = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    SysTime.s = (uint32_t)CNT_TEMP1;
    SysTime.ms = (uint16_t)(CNT_TEMP2 / CPU_FREQ_Hz_ms);
    CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = (uint16_t)(CNT_TEMP3 / CPU_FREQ_Hz_us);
}

//! 获取初始化以来的秒级时间轴。
float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = (float)SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

    return DWT_Timelinef32;
}

//! 获取初始化以来的毫秒级时间轴。
float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = (float)SysTime.s * 1000.0f + SysTime.ms + SysTime.us * 0.001f;

    return DWT_Timelinef32;
}

//! 获取初始化以来的微秒级时间轴。
uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();

    uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

    return DWT_Timelinef32;
}

// ==================== 忙等待延时 ====================

//! 使用 CYCCNT 做秒级忙等待延时，可在关闭中断或临界区中使用。
void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait         = Delay;

    while ((float)(DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
        ;
}
