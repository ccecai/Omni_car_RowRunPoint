//
// Created by 19734 on 2022/12/29.
//
#include "crc32.h"

//CRC校验函数为：
uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
    uint32_t i,bits;
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        //printf("ptr[%d]是：%x\r\n",i,ptr[i]);
        for (bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    //printf("crc32计算出来是是：%x\r\n",CRC32);
    return CRC32;
}
/**发送时调用函数：
*crc32_core((uint32_t*)(&(motor_s->motor_send_data)), 7);
*接收时调用函数：
*crc32_core((uint32_t*)(&(motor_r->motor_recv_data)), 18);
*/
