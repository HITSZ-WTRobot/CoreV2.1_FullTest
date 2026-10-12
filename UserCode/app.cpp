#include "cmsis_os2.h"
#include "eth/eth.hpp"

extern "C" void Init(void* argument)
{
    /* 初始化代码 */
    ETH_Init();

    /* 初始化完成后退出线程 */
    osThreadExit();
}
