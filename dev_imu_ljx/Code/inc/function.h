#ifndef __FUNCTION_H
#define __FUNCTION_H

#include "main.h"
#include "math.h"

//====================================================宏定义函数区====================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     绝对值函数 数据范围是 [-32767,32767]
// 参数说明     dat             需要求绝对值的数
// 返回参数     int             返回绝对值
// 使用示例     dat = func_abs(dat);                            // 将dat变成正数
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
#define     func_abs(x)             ((x) >= 0 ? (x): -(x))

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     限幅 数据范围是 [-32768,32767]
// 参数说明     x               被限幅的数据
// 参数说明     y               限幅范围(数据会被限制在-y至+y之间)
// 返回参数     int             限幅之后的数据         
// 使用示例     int dat = func_limit(500, 300);                 // 数据被限制在-300至+300之间  因此返回的结果是300
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
#define     func_limit(x, y)        ((x) > (y) ? (y) : ((x) < -(y) ? -(y) : (x)))

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     双边限幅 数据范围是 [-32768,32767]
// 参数说明     x               被限幅的数据
// 参数说明     a               限幅范围左边界
// 参数说明     b               限幅范围右边界
// 返回参数     int             限幅之后的数据         
// 使用示例     int dat = func_limit_ab(500, -300, 400);        //数据被限制在-300至+400之间  因此返回的结果是400
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
#define     func_limit_ab(x, a, b)  ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))

//====================================================宏定义函数区====================================================

#define     rad_to_deg(x)	(x / PI * 180)
#define     deg_to_rad(x)	(x * PI / 180)

//=====================================================常规函数区=====================================================
void        func_get_sin_amplitude_table        (uint32_t *data_buffer, uint32_t sample_max, uint32_t amplitude_max, uint32_t offset_degree);

uint32_t      func_get_greatest_common_divisor    (uint32_t num1, uint32_t num2);

void        func_soft_delay                     (volatile long t);

int32_t       func_str_to_int                     (char *str);
void        func_int_to_str                     (char *str, int32_t number);
uint32_t      func_str_to_uint                    (char *str);
void        func_uint_to_str                    (char *str, uint32_t number);
float       func_str_to_float                   (char *str);
void        func_float_to_str                   (char *str, float number, uint8_t point_bit);
double      func_str_to_double                  (char *str);
void        func_double_to_str                  (char *str, double number, uint8_t point_bit);
uint32_t      func_str_to_hex                     (char *str);
void        func_hex_to_str                     (char *str, uint32_t number);

uint32_t      zf_sprintf                          (int8_t *buff, const int8_t *format, ...);
//=====================================================常规函数区=====================================================
double 		mySqrt								(double x);
float 		invSqrt								(float x);

#endif
