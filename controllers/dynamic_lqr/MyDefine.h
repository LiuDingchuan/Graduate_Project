/* some const */
#define True 1
#define False 0

#define uchar unsigned char
#define uint8 unsigned char
#define uint16 unsigned short int
#define uint32 unsigned long
#define int16 short int
#define int32 long

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;

typedef int32_t s32;
typedef int16_t s16;
typedef int8_t s8;

typedef const int32_t sc32; /*!< Read Only */
typedef const int16_t sc16; /*!< Read Only */
typedef const int8_t sc8;   /*!< Read Only */

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;
typedef volatile uint8_t flagu8;

typedef const uint32_t uc32; /*!< Read Only */
typedef const uint16_t uc16; /*!< Read Only */
typedef const uint8_t uc8;   /*!< Read Only */

#define PI 3.141593f
#define PI_half 1.570796f
#define G_gravity 9.8f
#define cos_30 0.8660254f
#define cos_60 0.5f
#define cos_45 0.7071067f
#define sin_30 0.5f
#define sin_60 0.8660254f
#define sin_45 0.7071067f

/* some simple math function */
#define ABS(x) ((x) > 0 ? (x) : -(x))                                                         // 绝对值；括号x的括号不能丢
#define SIGN(x) ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))                                            // 获取数据正负号
#define SQUARE(x) ((x) * (x))                                                                 // 平方
#define HYPOT(x, y) (sqrt((x) * (x) + (y) * (y)))                                             // 斜边
#define BELONG(x, min, max) ((x) >= (MIN(min, max)) && (x) <= (MAX(min, max)) ? True : False) // if x in [min, max] return True
#define MAX(x, y) (((x) >= (y)) ? (x) : (y))                                                  // 两个数的最大值
#define MIN(x, y) (((x) >= (y)) ? (y) : (x))                                                  // 两个数的最小值
#define rad2deg(X) ((X) / PI * 180.0)                                                         // 角度转换                                                   // 角度转换
#define deg2rad(X) ((X) / 180.0 * PI)
#define Limit(x, max, min) ((x) > (MAX(min, max)) ? (MAX(min, max)) : ((x) < (MIN(min, max)) ? (MIN(min, max)) : (x)))

/* 两点间距离 */
#define CountDistance(x_from, y_from, x_to, y_to) \
    (float)(sqrt((x_to - x_from) * (x_to - x_from) + (y_to - y_from) * (y_to - y_from)))
/* 数量积 */
#define InnerProduct(vector1, vector2) \
    (float)(vector1.x * vector2.x + vector1.y * vector2.y)
#define Deadzone(input, threshold) ((fabs(input) < threshold) ? 0 : input)
