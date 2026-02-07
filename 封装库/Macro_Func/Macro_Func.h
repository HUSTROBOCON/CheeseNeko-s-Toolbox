#ifndef __MACRO_FUNC_H
#define __MACRO_FUNC_H

#include <stdint.h>

/**
 * @brief 限制变量在范围内
 * @param _IN 输入变量 (会被修改)
 * @param _MIN 最小值
 * @param _MAX 最大值
 */
#define SATURATE(_IN, _MIN, _MAX) \
        {                             \
                if ((_IN) <= (_MIN))      \
                        (_IN) = (_MIN);       \
                else if ((_IN) >= (_MAX)) \
                        (_IN) = (_MAX);       \
        }

#endif
