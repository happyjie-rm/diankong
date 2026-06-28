/*
  自定义的小工具
*/

#include "comp_utils.h"

#include <string.h>




/**
 * @brief 计算平方根倒数
 *
 * @param x 输入
 * @return float 计算结果
 */
inline float inv_sqrtf(float x) {
#if 0
  /* Fast inverse square-root */
  /* See: http://en.wikipedia.org/wiki/Fast_inverse_square_root */
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
#else
  return 1.0f / sqrtf(x);
#endif
}

/**
 * @brief 将值限制在-limit和limit之间。
 *
 * @param x 输入
 * @param limit 上下界的绝对值
 * @return float 操作后的值
 */
float abs_clampf(float x, float limit) {
  const float abs_limit = fabsf(limit);
  if (x > abs_limit) {
    return abs_limit;
  }

  if (x < -abs_limit) {
    return -abs_limit;
  }

  return x;
}

/**
 * @brief 将值限制在下限和上限之间。
 *
 * @param origin 被操作的值
 * @param lo 下限
 * @param hi 上限
 */
inline void clampf(float *origin, float lo, float hi) {
  ASSERT(origin);
  ASSERT(hi > lo);
  if (*origin < lo)
    *origin = lo;
  else if (*origin > hi)
    *origin = hi;
  //*origin = MIN(hi, MAX(*origin, lo));
}

/**
 * @brief 符号函数
 *
 * @param in 输入
 * @return float 运算结果
 */
inline float signf(float x) {
  if (x == 0.0f)
    return x;
  else
    return (x > 0) ? 1.0f : 0.0f;
}

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
inline float circle_error(float sp, float fb, float range) {
  float error = sp - fb;
  if (range > 0.0f) {
    float half_range = range / 2.0f;

    if (error > half_range)
      error -= range;
    else if (error < -half_range)
      error += range;
  }
  return error;
}

/**
 * @brief 循环加法，用于没有负数值，并在一定范围内变化的值
 * 例如编码器，在0-2PI内变化，1.5PI + 1.5PI = 1PI
 *
 * @param origin 被操作的值
 * @param delta 变化量
 * @param range 被操作的值变化范围，正数时起效
 */
inline void circle_add(float *origin, float delta, float range) {
  float out = *origin + delta;
  if (range > 0.0f) {
    while (out >= range) out -= range;
    while (out < 0.0f) out += range;
  }
  *origin = out;
}

/**
 * @brief 循环值取反
 *
 * @param origin 被操作的值
 */
inline void circle_reverse(float *origin) { *origin = -(*origin) + M_2PI; }


/**
 * @brief 断言失败处理
 *
 * @param file 文件名
 * @param line 行号
 */
void verify_failed(const char *file, uint32_t line) {
  RM_UNUSED(file);
  RM_UNUSED(line);
  while (1) {
  };
}
