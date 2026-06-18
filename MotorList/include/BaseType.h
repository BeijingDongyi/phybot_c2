/**
 * @file BaseType.h
 * @author BoCi486
 * @brief
 * @version 0.1
 * @date 2023-07-28
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef _BASE_TYPE_H_
#define _BASE_TYPE_H_
// #include "stdarg.h"
#include "stdint.h"

// /* 常用数学值 */
#define MATH_15Deg2Rad (0.26179938779914943f)
#define MATH_30Deg2Rad (0.523598775598298873f)
#define MATH_ONE_OVER_PI (0.3183098861837f)
#define MATH_PI_OVER_2 (1.5707963267f)
#define MATH_PI (3.1415926535897f)
#define MATH_2PI (6.2831853071795f)
#define MATH_TWO_PI MATH_2PI
#define MATH_ONE_VOER_SQRT_THREE (0.57735026918962576450914878f) // 1 / 根号3
#define MATH_SQRT_THREE_VOER_TWO (0.86602540378443864676372317f) // 根号3 / 2
#define MATH_SQRT_THREE (1.7320508075688772935274f) // 根号3
#define MATH_Hz2RadS_ (6.28318530717958647f)
#define MATH_RadS2Hz (0.1591549430918953357f)
#define MATH_Rad2Deg (57.29577951308232087f)
#define MATH_Deg2Rad (0.0174532925199432959f)
#define MATH_RPM2RadS (0.1047197551196597746f)
#define MATH_RadS2RPM  (9.5492965855137201461330258023509f)

#define true 1
#define false 0

#ifdef PAC
#define FOC_RAMFUNC __attribute__((section(".ramfunc")))
#else
#define FOC_RAMFUNC 
#endif

typedef struct
{
    float V1;
    float V2;
} Vct2_f32;

typedef struct
{
    float V1;
    float V2;
    float V3;
} Vct3_f32;

typedef struct
{
    uint32_t V1;
    uint32_t V2;
} Vct2_u32;

typedef struct
{
    uint32_t V1;
    uint32_t V2;
    uint32_t V3;
} Vct3_u32;

/*空指针处理*/
// 编译阶段检查空指针
#ifndef DEBUG
#define assert_non_null(ptr)
#else
void PtrIsNULL(void);
#define assert_non_null(ptr) \
    if (ptr == NULL)         \
    {                        \
        PtrIsNULL();         \
    }
#endif

// 啥都不干的指针，有自己需要用的类型在这添加
//  Demo:  uint32_t NullPrt_RetU32_InFloadt(float In,...);
void NullPtr_RetVoid_InVoid(void);
float NullPtr_RetFloaat_InVoid(void);
uint32_t NullPtr_RetU32_InVoid(void);

#endif