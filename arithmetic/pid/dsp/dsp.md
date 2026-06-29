三角函数
#define PI 3.141592f
float32_t Angle = 60.0f;
float32_t Angle_Arc = Angle / 180 * PI;
float32_t Cos = 0.0f;
float32_t Sin = 0.0f;
Cos = arm_cos_f32(Angle_Arc); // 计算余弦值
Sin = arm_sin_f32(Angle_Arc); // 计算正弦值
平方根
float32_t Input = 9.0f;
float32_t Output = 0.0f;
arm_sqrt_f32(Input, &Output); // 计算平方根
最大值、最小值、平均值
float32_t Array[3] = {0f, 1.2f, 1.3f};
float32_t Min = 0.0f;
uint32_t Min_index = 0;
float32_t Max = 0.0f;
uint32_t Max_index = 0;
arm_max_f32(Array, 3, &Max, &Max_index); // 计算最大值
arm_min_f32(Array, 3, &Min, &Min_index); // 计算最小值
标准差、均方根、方差
float32_t Array[3] = {0f, 1.2f, 1.3f};
float32_t Std = 0.0f;
float32_t Rms = 0.0f;
float32_t Var = 0.0f;
arm_std_f32(Array, 3, &Std); // 计算标准差
arm_rms_f32(Array, 3, &Rms); // 计算均方根
arm_var_f32(Array, 3, &Var); // 计算方差
数据拷贝、填充
float32_t Array_A[3] = {0f, 1.2f, 1.3f};
float32_t Array_B[3];
arm_copy_f32(Array_A, Array_B, 3); // 拷贝数据
arm_fill_f32(5.0f, Array_B, 3); // 填充数据
基本运算
float32_t Array_A[3] = {0f, 1.2f, 1.3f};
float32_t Array_B[3] = {3.5f, -5.0f, 2.5f};
float32_t Result_Array[3];
float32_t Dot_result;
arm_add_f32(Array_A, Array_B, Result_Array, 3); // 求和
arm_sub_f32(Array_A, Array_B, Result_Array, 3); // 求差
arm_mult_f32(Array_A, Array_B, Result_Array, 3); // 求乘法
arm_dot_prod_f32(Array_A, Array_B, 3, &Dot_result); // 求点乘