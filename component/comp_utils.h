/*
  自定义的小工具
*/

#ifndef COMP_UTILS_H
#define COMP_UTILS_H

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>




#ifndef M_2PI
#define M_2PI 6.28318530717958647692f
#endif

/* MCU_DEBUG_BUILD 由 CMake Debug 配置注入，禁止在头文件里手工定义。 */


/**
 * @brief 返回两个值中的最大值
 *
 */
#define MAX(a, b)                                                              \
  ({                                                                           \
    __typeof__(a) _a = (a);                                                    \
    __typeof__(b) _b = (b);                                                    \
    _a > _b ? _a : _b;                                                         \
  })

/**
 * @brief 返回两个值中的最小值
 *
 */
#define MIN(a, b)                                                              \
  ({                                                                           \
    __typeof__(a) _a = (a);                                                    \
    __typeof__(b) _b = (b);                                                    \
    _a < _b ? _a : _b;                                                         \
  })

#ifdef MCU_DEBUG_BUILD

/**
 * @brief 如果表达式的值为假则运行处理函数
 *
 */
#define ASSERT(expr)                                                           \
  do {                                                                         \
    if (!(expr)) {                                                             \
      verify_failed(__FILE__, __LINE__);                                       \
    }                                                                          \
  } while (0)
#else

/**
 * @brief 未定DEBUG，表达式不会运行，断言被忽略
 *
 */
#define ASSERT(expr) ((void)(0))
#endif

#ifdef MCU_DEBUG_BUILD

/**
 * @brief 如果表达式的值为假则运行处理函数
 *
 */
#define VERIFY(expr)                                                           \
  do {                                                                         \
    if (!(expr)) {                                                             \
      verify_failed(__FILE__, __LINE__);                                       \
    }                                                                          \
  } while (0)
#else

/**
 * @brief 表达式会运行，忽略表达式结果
 *
 */
#define VERIFY(expr) ((void)(expr))
#endif

#ifndef RM_UNUSED

/**
 * @brief 标记未使用的参数，防止编译器警告
 *
 */
#define RM_UNUSED(X) ((void)X)
#endif

/**
 * @brief 获取结构体或者联合成员的容器
 *
 */
#define CONTAINER_OF(ptr, type, member)                                        \
  ({                                                                           \
    const typeof(((type *)0)->member) *__mptr = (ptr);                         \
    (type *)((char *)__mptr - offsetof(type, member));                         \
  })

/**
 * @brief 获取数组长度
 *
 */
#define ARRAY_LEN(array) (sizeof((array)) / sizeof(*(array)))

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 计算平方根倒数
 *
 * @param x 输入
 * @return float 计算结果
 */
float inv_sqrtf(float x);

/**
 * @brief 将值限制在-limit和limit之间。
 *
 * @param x 输入
 * @param limit 上下界的绝对值
 * @return float 操作后的值
 */
float abs_clampf(float x, float limit);

/**
 * @brief 将值限制在下限和上限之间。
 *
 * @param origin 被操作的值
 * @param lo 下限
 * @param hi 上限
 */
void clampf(float *origin, float lo, float hi);

/**
 * @brief 符号函数
 *
 * @param in 输入
 * @return float 运算结果
 */
float signf(float x);

/**
 * @brief 计算循环值的误差，用于没有负数值，并在一定范围内变化的值
 * 例如编码器：相差1.5PI其实等于相差-0.5PI
 *
 * @param sp 被操作的值
 * @param fb 变化量
 * @param range 被操作的值变化范围，正数时起效
 *
 * @return 函数运行结果
 */
float circle_error(float sp, float fb, float range);

/**
 * @brief 循环加法，用于没有负数值，并在一定范围内变化的值
 * 例如编码器，在0-2PI内变化，1.5PI + 1.5PI = 1PI
 *
 * @param origin 被操作的值
 * @param delta 变化量
 * @param range 被操作的值变化范围，正数时起效
 */
void circle_add(float *origin, float delta, float range);

/**
 * @brief 循环值取反
 *
 * @param origin 被操作的值
 */
void circle_reverse(float *origin);



/**
 * @brief 断言失败处理
 *
 * @param file 文件名
 * @param line 行号
 */
void verify_failed(const char *file, uint32_t line);

#ifdef __cplusplus
}
#endif
#endif // COMP_UTILS_H
