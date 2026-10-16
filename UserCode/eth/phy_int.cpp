/**
 * @file phy_int.cpp
 * @brief 唯一 HAL EXTI 软件桥接：GPIO_Driver 分发给 DP83822/Key 等注册者。
 *
 * CubeMX 生成 GPIO、NVIC 和 EXTI15_10/EXTI9_5/EXTI2 IRQHandler；本文件不重复定义。
 * PB10 PHY INT 为上拉下降沿、priority 5，ISR 只经 DP83822 回调唤醒 LwipPlatform，
 * 不访问 MDIO、不轮询、不修改硬件配置。链路确认全部在线程/既有包内完成。
 */
#include "gpio_driver.hpp"
#include "main.h"

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    bsp::gpio::DispatchExtiInterrupt(GPIO_Pin);
}
