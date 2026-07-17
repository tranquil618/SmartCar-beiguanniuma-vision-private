#ifndef DEFINE_H_
#define DEFINE_H_
#ifndef PC_DEBUG
    #include "inc.h"  // 只有在非 PC 模式（即龙芯模式）下才包含硬件头文件
#else
    // 在 PC 上调试时，手动定义算法需要的类型，不要引用 inc.h
    #include <cstdint>
    typedef uint8_t  uint8;
    typedef uint16_t uint16;
    typedef uint32_t uint32;
    typedef uint16_t uint16_t; // 确保兼容性
#endif

#define USE_Copy_IMG    (1)    //宏定义一个标识符，判断使用图像副本还是使用摄像头原始灰度图
#if USE_Copy_IMG!=0&&USE_Copy_IMG!=1   //如果开关既非0又非1
#error "USE_Copy_IMG错误"
#endif

#define clip(x,min,max)   (((x)>(max))?(max):(((x<(min))?(min):(x))))
#define clip2(x,num)      (clip((x),(-num),(num)))
//#define MAX(a,b)          (((a)>(b))?(a):(b))
//#define MIN(a,b)          (((a)<(b))?(a):(b))
#ifdef MAX
#undef MAX
#endif
#define MAX(a,b) (((a)>(b))?(a):(b))

#ifdef MIN
#undef MIN
#endif
#define MIN(a,b) (((a)<(b))?(a):(b))

#if USE_Copy_IMG
//#define AT_IMG(x,y)       (Image_Used[(y)*IMG_W+(x)])
#define AT_IMG(x,y)       (Image_Used[(y)][(x)])
#else

#endif
#define AT_CLIPING(x,y)   (AT_IMG((clip((x),(0),(IMG_W-1))),(clip((y),(0),(IMG_H-1)))))
#define AT(x,y)           (AT_CLIPING((x),(y)))
#endif  //DEFINE_H_