#include "function.h"
#include <stdarg.h>
#include <string.h>

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     正弦波转换函数 获取指定采样点
// 参数说明     *data_buffer    数据缓冲区
// 参数说明     sample_max      采样点数
// 参数说明     amplitude_max   最大幅值
// 参数说明     offset_degree   偏移相位角
// 返回参数     void
// 使用示例     func_get_sin_amplitude_table(sound_amplitude, 1024, 512, 270);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void func_get_sin_amplitude_table (uint32_t *data_buffer, uint32_t sample_max, uint32_t amplitude_max, uint32_t offset_degree)
{
    uint32_t i = 0, j = 0;
    double rad = 0.0;
    double amplitude = 0.0;

    offset_degree = offset_degree % 360;
    offset_degree = (uint32_t)((float)offset_degree / 360.0f * sample_max);

    j = amplitude_max / 2;                                                      // 偏置幅值 默认为一半最大幅值
    rad = 3.1415926535898 * 2 / sample_max;                                     // 换算每个采样点的弧度差
    for(i = offset_degree; i < sample_max + offset_degree; i ++)
    {
        amplitude = j * sin(rad * i) + j;
        *data_buffer = (uint32_t)amplitude;
        data_buffer ++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取整型数的最大公约数 九章算术之更相减损术
// 参数说明     num1            数字1
// 参数说明     num2            数字2
// 返回参数     uint32_t          最大公约数
// 使用示例     return func_get_greatest_common_divisor(144, 36);               // 获取 144 与 36 的最大公约数
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint32_t func_get_greatest_common_divisor (uint32_t num1, uint32_t num2)
{
    while(num1 != num2)
    {
        if(num1 > num2)
        {
            num1 = num1 - num2;
        }
        if(num1 < num2)
        {
            num2 = num2 - num1;
        }
    }
    return num1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     软件延时
// 参数说明     t               延时时间
// 返回参数     void
// 使用示例     func_soft_delay(100);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void func_soft_delay (volatile long t)
{
    while(t --);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     字符串转整形数字 数据范围是 [-32768,32767]
// 参数说明     *str            传入字符串 可带符号
// 返回参数     int32_t           转换后的数据          
// 使用示例     int32_t dat = func_str_to_int("-100");
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
int32_t func_str_to_int (char *str)
{
    assert_param(NULL != str);
    uint8_t sign = 0;                                                             // 标记符号 0-正数 1-负数
    int32_t temp = 0;                                                             // 临时计算变量
    do
    {
        if(NULL == str)
        {
            break;
        }

        if('-' == *str)                                                         // 如果第一个字符是负号
        {
            sign = 1;                                                           // 标记负数
            str ++;
        }
        else if('+' == *str)                                                    // 如果第一个字符是正号
        {
            str ++;
        }

        while(('0' <= *str) && ('9' >= *str))                                   // 确定这是个数字
        {
            temp = temp * 10 + ((uint8_t)(*str) - 0x30);                          // 计算数值
            str ++;
        }

        if(sign)
        {
            temp = -temp;
        }
    }while(0);
    return temp;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     整形数字转字符串 数据范围是 [-32768,32767]
// 参数说明     *str            字符串指针
// 参数说明     number          传入的数据
// 返回参数     void
// 使用示例     func_int_to_str(data_buffer, -300);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void func_int_to_str (char *str, int32_t number)
{
    assert_param(NULL != str);
    uint8_t data_temp[16];                                                        // 缓冲区
    uint8_t bit = 0;                                                              // 数字位数
    int32_t number_temp = 0;

    do
    {
        if(NULL == str)
        {
            break;
        }

        if(0 > number)                                                          // 负数
        {
            *str ++ = '-';
            number = -number;
        }
        else if(0 == number)                                                    // 或者这是个 0
        {
            *str = '0';
            break;
        }

        while(0 != number)                                                      // 循环直到数值归零
        {
            number_temp = number % 10;
            data_temp[bit ++] = func_abs(number_temp);                          // 倒序将数值提取出来
            number /= 10;                                                       // 削减被提取的个位数
        }
        while(0 != bit)                                                         // 提取的数字个数递减处理
        {
            *str ++ = (data_temp[bit - 1] + 0x30);                              // 将数字从倒序数组中倒序取出 变成正序放入字符串
            bit --;
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     字符串转整形数字 数据范围是 [0,65535]
// 参数说明     *str            传入字符串 无符号
// 返回参数     uint32_t          转换后的数据          
// 使用示例     uint32_t dat = func_str_to_uint("100");
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint32_t func_str_to_uint (char *str)
{
    assert_param(NULL != str);
    uint32_t temp = 0;                                                            // 临时计算变量

    do
    {
        if(NULL == str)
        {
            break;
        }

        while(('0' <= *str) && ('9' >= *str))                                  // 确定这是个数字
        {
            temp = temp * 10 + ((uint8_t)(*str) - 0x30);                         // 计算数值
            str ++;
        }
    }while(0);

    return temp;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     整形数字转字符串 数据范围是 [0,65535]
// 参数说明     *str            字符串指针
// 参数说明     number          传入的数据
// 返回参数     void
// 使用示例     func_uint_to_str(data_buffer, 300);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void func_uint_to_str (char *str, uint32_t number)
{
    assert_param(NULL != str);
    int8_t data_temp[16];                                                         // 缓冲区
    uint8_t bit = 0;                                                              // 数字位数

    do
    {
        if(NULL == str)
        {
            break;
        }

        if(0 == number)                                                         // 这是个 0
        {
            *str = '0';
            break;
        }

        while(0 != number)                                                      // 循环直到数值归零
        {
            data_temp[bit ++] = (number % 10);                                  // 倒序将数值提取出来
            number /= 10;                                                       // 削减被提取的个位数
        }
        while(0 != bit)                                                         // 提取的数字个数递减处理
        {
            *str ++ = (data_temp[bit - 1] + 0x30);                              // 将数字从倒序数组中倒序取出 变成正序放入字符串
            bit --;
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     字符串转浮点数 有效累计精度为小数点后六位
// 参数说明     *str            传入字符串 可带符号
// 返回参数     float           转换后的数据          
// 使用示例     float dat = func_str_to_float("-100.2");
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
float func_str_to_float (char *str)
{
    assert_param(NULL != str);
    uint8_t sign = 0;                                                             // 标记符号 0-正数 1-负数
    float temp = 0.0;                                                           // 临时计算变量 整数部分
    float temp_point = 0.0;                                                     // 临时计算变量 小数部分
    float point_bit = 1;                                                        // 小数累计除数

    do
    {
        if(NULL == str)
        {
            break;
        }

        if('-' == *str)                                                         // 负数
        {
            sign = 1;                                                           // 标记负数
            str ++;
        }
        else if('+' == *str)                                                    // 如果第一个字符是正号
        {
            str ++;
        }

        // 提取整数部分
        while(('0' <= *str) && ('9' >= *str))                                   // 确定这是个数字
        {
            temp = temp * 10 + ((uint8_t)(*str) - 0x30);                          // 将数值提取出来
            str ++;
        }
        if('.' == *str)
        {
            str ++;
            while(('0' <= *str) && ('9' >= *str) && 1000000.0f > point_bit)      // 确认这是个数字 并且精度控制还没到六位
            {
                temp_point = temp_point * 10 + ((uint8_t)(*str) - 0x30);          // 提取小数部分数值
                point_bit *= 10;                                                // 计算这部分小数的除数
                str ++;
            }
            temp_point /= point_bit;                                            // 计算小数
        }
        temp += temp_point;                                                     // 将数值拼合

        if(sign)
        {
            temp = -temp;
        }
    }while(0);
    return temp;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     浮点数字转字符串
// 参数说明     *str            字符串指针
// 参数说明     number          传入的数据
// 参数说明     point_bit       小数点精度
// 返回参数     void
// 使用示例     func_float_to_str(data_buffer, 3.1415, 2);                      // 结果输出 data_buffer = "3.14"
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void func_float_to_str (char *str, float number, uint8_t point_bit)
{
    assert_param(NULL != str);
    int data_int = 0;                                                           // 整数部分
    int data_float = 0.0;                                                       // 小数部分
    int data_temp[8];                                                           // 整数字符缓冲
    int data_temp_point[6];                                                     // 小数字符缓冲
    uint8_t bit = point_bit;                                                      // 转换精度位数

    do
    {
        if(NULL == str)
        {
            break;
        }

        // 提取整数部分
        data_int = (int)number;                                                 // 直接强制转换为 int
        if(0 > number)                                                          // 判断源数据是正数还是负数
        {
            *str ++ = '-';
        }
        else if(0.0f == number)                                                  // 如果是个 0
        {
            *str ++ = '0';
            *str ++ = '.';
            *str = '0';
            break;
        }

        // 提取小数部分
        number = number - data_int;                                             // 减去整数部分即可
        while(bit --)
        {
            number = number * 10;                                               // 将需要的小数位数提取到整数部分
        }
        data_float = (int)number;                                               // 获取这部分数值

        // 整数部分转为字符串
        bit = 0;
        do
        {
            data_temp[bit ++] = data_int % 10;                                  // 将整数部分倒序写入字符缓冲区
            data_int /= 10;
        }while(0 != data_int);
        while(0 != bit)
        {
            *str ++ = (func_abs(data_temp[bit - 1]) + 0x30);                    // 再倒序将倒序的数值写入字符串 得到正序数值
            bit --;
        }

        // 小数部分转为字符串
        if(0 != point_bit)
        {
            bit = 0;
            *str ++ = '.';
            if(0 == data_float)
            {
                *str = '0';
            }
            else
            {
                while(0 != point_bit)                                           // 判断有效位数
                {
                    data_temp_point[bit ++] = data_float % 10;                  // 倒序写入字符缓冲区
                    data_float /= 10;
                    point_bit --;                                                
                }
                while(0 != bit)
                {
                    *str ++ = (func_abs(data_temp_point[bit - 1]) + 0x30);      // 再倒序将倒序的数值写入字符串 得到正序数值
                    bit --;
                }
            }
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     字符串转浮点数 有效累计精度为小数点后九位
// 参数说明     str             传入字符串 可带符号
// 返回参数     double          转换后的数据          
// 使用示例     double dat = func_str_to_double("-100.2");
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
double func_str_to_double (char *str)
{
    assert_param(NULL != str);
    uint8_t sign = 0;                                                             // 标记符号 0-正数 1-负数
    double temp = 0.0;                                                          // 临时计算变量 整数部分
    double temp_point = 0.0;                                                    // 临时计算变量 小数部分
    double point_bit = 1;                                                       // 小数累计除数

    do
    {
        if(NULL == str)
        {
            break;
        }

        if('-' == *str)                                                         // 负数
        {
            sign = 1;                                                           // 标记负数
            str ++;
        }
        else if('+' == *str)                                                    // 如果第一个字符是正号
        {
            str ++;
        }

        // 提取整数部分
        while(('0' <= *str) && ('9' >= *str))                                   // 确定这是个数字
        {
            temp = temp * 10 + ((uint8_t)(*str) - 0x30);                          // 将数值提取出来
            str ++;
        }
        if('.' == *str)
        {
            str ++;
            while(('0' <= *str) && ('9' >= *str) && 1000000000.0 > point_bit)   // 确认这是个数字 并且精度控制还没到九位
            {
                temp_point = temp_point * 10 + ((uint8_t)(*str) - 0x30);          // 提取小数部分数值
                point_bit *= 10;                                                // 计算这部分小数的除数
                str ++;
            }
            temp_point /= point_bit;                                            // 计算小数
        }
        temp += temp_point;                                                     // 将数值拼合

        if(sign)
        {
            temp = -temp;
        }
    }while(0);
    return temp;

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     浮点数字转字符串
// 参数说明     *str            字符串指针
// 参数说明     number          传入的数据
// 参数说明     point_bit       小数点精度
// 返回参数     void
// 使用示例     func_double_to_str(data_buffer, 3.1415, 2);                     // 结果输出 data_buffer = "3.14"
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void func_double_to_str (char *str, double number, uint8_t point_bit)
{
    assert_param(NULL != str);
    int data_int = 0;                                                           // 整数部分
    int data_float = 0.0;                                                       // 小数部分
    int data_temp[12];                                                          // 整数字符缓冲
    int data_temp_point[9];                                                     // 小数字符缓冲
    uint8_t bit = point_bit;                                                      // 转换精度位数

    do
    {
        if(NULL == str)
        {
            break;
        }

        // 提取整数部分
        data_int = (int)number;                                                 // 直接强制转换为 int
        if(0 > number)                                                          // 判断源数据是正数还是负数
        {
            *str ++ = '-';
        }
        else if(0.0 == number)                                                  // 如果是个 0
        {
            *str ++ = '0';
            *str ++ = '.';
            *str = '0';
            break;
        }

        // 提取小数部分
        number = number - data_int;                                             // 减去整数部分即可
        while(bit --)
        {
            number = number * 10;                                               // 将需要的小数位数提取到整数部分
        }
        data_float = (int)number;                                               // 获取这部分数值

        // 整数部分转为字符串
        bit = 0;
        do
        {
            data_temp[bit ++] = data_int % 10;                                  // 将整数部分倒序写入字符缓冲区
            data_int /= 10;
        }while(0 != data_int);
        while(0 != bit)
        {
            *str ++ = (func_abs(data_temp[bit - 1]) + 0x30);                    // 再倒序将倒序的数值写入字符串 得到正序数值
            bit --;
        }

        // 小数部分转为字符串
        if(0 != point_bit)
        {
            bit = 0;
            *str ++ = '.';
            if(0 == data_float)
            {
                *str = '0';
            }
            else
            {
                while(0 != point_bit)                                           // 判断有效位数
                {
                    data_temp_point[bit ++] = data_float % 10;                  // 倒序写入字符缓冲区
                    data_float /= 10;
                    point_bit --;                                                
                }
                while(0 != bit)
                {
                    *str ++ = (func_abs(data_temp_point[bit - 1]) + 0x30);      // 再倒序将倒序的数值写入字符串 得到正序数值
                    bit --;
                }
            }
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     字符串转 Hex
// 参数说明     str             传入字符串 无符号
// 返回参数     uint32_t          转换后的数据
// 使用示例     uint32_t dat = func_str_to_hex("0x11");
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint32_t func_str_to_hex (char *str)
{
    assert_param(NULL != str);
    uint32_t str_len = strlen(str);                                               // 字符串长
    uint32_t result_data = 0;                                                     // 结果缓存
    uint8_t temp = 0;                                                             // 计算变量
    uint8_t flag = 0;                                                             // 标志位

    do
    {
        if(NULL == str)
        {
            break;
        }

        if(flag)
        {
            if(('a' <= *str) && ('f' >= *str))
            {
                temp = (*str - 87);
            }
            else if(('A' <= *str) && ('F' >= *str))
            {
                temp = (*str - 55);
            }
            else if(('0' <= *str) && ('9' >= *str))
            {
                temp = (*str - 48);
            }
            else
            {
                break;
            }
            result_data = ((result_data << 4) | (temp & 0x0F));
        }
        else
        {
//            if(strncmp("0x", str, 2))
            if(('0' == *str) && ('x' == *(str + 1)))
            {
                str ++;
                flag = 1;
            }
        }
        str ++;
    }while(str_len --);

    return result_data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Hex 转字符串
// 参数说明     *str            字符串指针
// 参数说明     number          传入的数据
// 返回参数     void
// 使用示例     func_hex_to_str(data_buffer, 0x11);                             // 结果输出 data_buffer = "0x11"
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void func_hex_to_str (char *str, uint32_t number)
{
    assert_param(NULL != str);
    const char hex_index[16] = {
        '0', '1', '2', '3',
        '4', '5', '6', '7',
        '8', '9', 'A', 'B',
        'C', 'D', 'E', 'F'};
    int8_t data_temp[12];                                                         // 缓冲区
    uint8_t bit = 0;                                                              // 数字位数

//    *str ++ = '0';
//    *str ++ = 'x';
    do
    {
        if(NULL == str)
        {
            break;
        }

        if(0 == number)                                                         // 这是个 0
        {
            *str = '0';
            break;
        }

        while(0 != number)                                                      // 循环直到数值归零
        {
            data_temp[bit ++] = (number & 0xF);                                 // 倒序将数值提取出来
            number >>= 4;                                                       // 削减被提取的个位数
        }
        while(0 != bit)                                                         // 提取的数字个数递减处理
        {
            *str ++ = hex_index[data_temp[bit - 1]];                            // 将数字从倒序数组中倒序取出 变成正序放入字符串
            bit --;
        }
    }while(0);
}

//带正负开根号
double mySqrt(double x)
{
	int minus = 1;
    if (x < 0) 
	{
        x = func_abs(x);
		minus = -1;
    }
    if (x == 0 || x == 1) 
	{
        return x;
    }
    double guess = x / 2.0; // 初始猜测
    double epsilon = 1e-3; // 定义一个很小的误差值作为递减条件

    while (func_abs(guess * guess - x) > epsilon) 
	{
        guess = (guess + x / guess) / 2.0; // 牛顿迭代公式
    }

    return (guess * minus);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     数字转换为 ASCII 值
// 参数说明     dat             传入的数据
// 参数说明     *p              数据缓冲
// 参数说明     neg_type        数据类型
// 参数说明     radix           进制
// 返回参数     uint8_t           数据
// 使用示例     number_conversion_ascii((uint32_t)ival, vstr, 1, 10);
// 备注信息     本函数在文件内部调用 用户不用关注 也不可修改
//-------------------------------------------------------------------------------------------------------------------
static uint8_t number_conversion_ascii (uint32_t dat, int8_t *p, uint8_t neg_type, uint8_t radix)
{
    int32_t   neg_dat = 0;
    uint32_t  pos_dat = 0;
    uint8_t   temp_data = 0;
    uint8_t   valid_num = 0;

    if(neg_type)
    {
        neg_dat = (int32_t)dat;
        if(0 > neg_dat)
        {
            neg_dat = -neg_dat;
        }
        while(1)
        {
            *p = neg_dat % radix + '0';
            neg_dat = neg_dat / radix;
            valid_num ++;

            if(!neg_dat)
            {
                break;
            }
            p ++;
        }
    }
    else
    {
        pos_dat = dat;
        while(1)
        {
            temp_data = pos_dat % radix;
            if(10 <= temp_data)
            {
                temp_data += 'A' - 10;
            }
            else
            {
                temp_data += '0';
            }

            *p = temp_data;

            pos_dat = pos_dat / radix;
            valid_num ++;

            if(!pos_dat)
            {
                break;
            }
            p ++;
        }
    }
    return valid_num;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     printf 显示转换
// 参数说明     *d_buff         缓冲区
// 参数说明     len             长度
// 返回参数     void
// 使用示例     printf_reverse_order(vstr, vlen);
// 备注信息     本函数在文件内部调用 用户不用关注 也不可修改
//-------------------------------------------------------------------------------------------------------------------
static void printf_reverse_order (int8_t *d_buff, uint32_t len)
{
    uint32_t i = 0;
    int8_t  temp_data = 0;
    for(i = 0; len / 2 > i; i ++)
    {
        temp_data = d_buff[len - 1 - i];
        d_buff[len - 1 - i] = d_buff[i];
        d_buff[i] = temp_data; 
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     sprintf 函数实现
// 参数说明     *buff           缓冲区
// 参数说明     *format         源字符串
// 参数说明     ...             可变参数列表
// 返回参数     uint32_t          处理后数据长
// 使用示例     zf_sprintf(buff, "Data : %d", 100);
// 备注信息     本函数在文件内部调用 用户不用关注 也不可修改
//-------------------------------------------------------------------------------------------------------------------
uint32_t zf_sprintf (int8_t *buff, const int8_t *format, ...)
{
    uint32_t buff_len = 0;
	va_list arg;
	va_start(arg, format);

    while (*format)
    {
        int8_t ret = *format;
        if ('%' == ret)
        {
            switch (*++ format)
            {
                case 'a':// 十六进制p计数法输出浮点数 暂未实现
                {
                }break;

                case 'c':// 一个字符
                {
                    int8_t ch = (int8_t)va_arg(arg, uint32_t);
                    *buff = ch;
                    buff ++;
                    buff_len ++;
                }break;

                case 'd':
                case 'i':// 有符号十进制整数
                {
                    int8_t vstr[33];
                    int32_t ival = (int32_t)va_arg(arg, int32_t);
                    uint8_t vlen = number_conversion_ascii((uint32_t)ival, vstr, 1, 10);

                    if(0 > ival)  
                    {
                        vstr[vlen] = '-';
                        vlen ++;
                    }
                    printf_reverse_order(vstr, vlen);
                    memcpy(buff, vstr, vlen);
                    buff += vlen;
                    buff_len += vlen;
                }break;

                case 'f':// 浮点数，输出小数点后六位  不能指定输出精度
                case 'F':// 浮点数，输出小数点后六位  不能指定输出精度
                {
                    int8_t vstr[33];
                    double ival = (double)va_arg(arg, double);
                    uint8_t vlen = number_conversion_ascii((uint32_t)(int32_t)ival, vstr, 1, 10);

                    if(0 > ival)  
                    {
                        vstr[vlen] = '-';
                        vlen ++;
                    }
                    printf_reverse_order(vstr, vlen);
                    memcpy(buff, vstr, vlen);
                    buff += vlen;
                    buff_len += vlen;

                    ival = ((double)ival - (int32_t)ival) * 1000000;
                    if(ival)
                    {
                        vlen = number_conversion_ascii((uint32_t)(int32_t)ival, vstr, 1, 10);
                    }
                    else
                    {
                        vstr[0] = vstr[1] = vstr[2] = vstr[3] = vstr[4] = vstr[5] = '0';
                        vlen = 6;
                    }

                    while(6 > vlen)
                    {
                        vstr[vlen] = '0';
                        vlen ++;
                    }

                    vstr[vlen] = '.';
                    vlen ++;

                    printf_reverse_order(vstr, vlen);
                    memcpy(buff, vstr, vlen);
                    buff_len += vlen;
                }break;

                case 'u':// 无符号十进制整数
                {
                    int8_t vstr[33];
                    uint32_t ival = (uint32_t)va_arg(arg, uint32_t);
                    uint8_t vlen = number_conversion_ascii(ival, vstr, 0, 10);

                    printf_reverse_order(vstr, vlen);
                    memcpy(buff, vstr, vlen);
                    buff += vlen;
                    buff_len += vlen;
                }break;

                case 'o':// 无符号八进制整数 
                {
                    int8_t vstr[33];
                    uint32_t ival = (uint32_t)va_arg(arg, uint32_t);
                    uint8_t vlen = number_conversion_ascii(ival, vstr, 0, 8);

                    printf_reverse_order(vstr, vlen);
                    memcpy(buff, vstr, vlen);
                    buff += vlen;
                    buff_len += vlen;

                }break;

                case 'x':// 无符号十六进制整数
                case 'X':// 无符号十六进制整数
                {
                    int8_t vstr[33];
                    uint32_t ival = (uint32_t)va_arg(arg, uint32_t);
                    uint8_t vlen = number_conversion_ascii(ival, vstr, 0, 16);

                    printf_reverse_order(vstr, vlen);
                    memcpy(buff, vstr, vlen);
                    buff += vlen;
                    buff_len += vlen;
                }break;

                case 's':// 字符串
                {
                    int8_t *pc = va_arg(arg, int8_t *);
                    while (*pc)
                    {
                        *buff = *pc;
                        buff ++;
                        buff_len ++;
                        pc ++;
                    }
                }break;

                case 'p':// 以16进制形式输出指针
                {
                    int8_t vstr[33];
                    uint32_t ival = (uint32_t)va_arg(arg, uint32_t);
                    uint8_t vlen = number_conversion_ascii(ival, vstr, 0, 16);

                    printf_reverse_order(vstr, 8);
                    memcpy(buff, vstr, 8);
                    buff += 8;
                    buff_len += 8;
                }break;

                case '%':// 输出字符% 
                {
                    *buff = '%';
                    buff ++;
                    buff_len ++;
                }break;

                default:
                {
                }break;
            }
        }
        else
        {
            *buff = (int8_t)(*format);
            buff ++;
            buff_len ++;
        }
        format ++;
    }
    va_end(arg);

    return buff_len;
}

float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
