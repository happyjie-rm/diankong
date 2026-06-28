#ifndef CRC_REF_H
#define CRC_REF_H

#include <stdint.h>

//* 裁判系统 CRC 工具接口。
//? CRC8 通常放在帧头末尾，CRC16 放在整帧末尾；这些接口直接在调用方提供的
//? 字节缓冲区上计算、校验或回填校验值，不申请额外内存。
//! 保留 TRUE/FALSE 无符号宏是为了兼容裁判系统旧代码中直接比较宏返回值的写法。
#define TRUE 1u
#define FALSE 0u

//* 将 CRC8 写入消息/帧头缓冲区的最后 1 字节。
//? dwLength 必须把目标 CRC8 字节也计算在内。
//! pchMessage 为空或长度不足以容纳有效数据和 CRC8 时，函数会直接返回。
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);

//* 校验消息/帧头最后 1 字节是否匹配 CRC8。
//? 对前 dwLength - 1 字节重新计算 CRC8，和末尾保存的校验字节一致时返回 TRUE。
uint32_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);

//* 使用调用方给定的初值计算 dwLength 字节的 CRC8。
//? 传入上一次返回的 CRC 值可以继续分段计算。
//! dwLength 非 0 时，调用方必须保证 pchMessage 指向有效缓冲区。
uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength,
                           uint8_t ucCRC8);

//* 将 CRC16 写入完整帧缓冲区的最后 2 字节。
//? 裁判系统帧格式按小端保存 CRC16：低字节在前，高字节在后。
//! dwLength 必须把 2 个 CRC16 目标字节也计算在内。
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

//* 校验完整帧末尾 2 字节是否匹配 CRC16。
//? 对前 dwLength - 2 字节重新计算 CRC16，再和末尾的小端 CRC16 尾部比较。
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

//* 使用调用方给定的初值计算 dwLength 字节的 CRC16。
//? 既可用于整帧计算，也可用于分段累积计算。
//! pchMessage 为空时返回 0xffffu，以保持原有防御性行为。
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength,
                             uint16_t wCRC);

#endif
