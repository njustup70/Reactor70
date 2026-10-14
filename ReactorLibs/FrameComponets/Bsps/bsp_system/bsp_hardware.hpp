#pragma once
#include "bsp_uart.hpp"
#include "bsp_can.hpp"
#include "bsp_spi.hpp"

namespace Hardware
{
    /***---------------     框架串口    ---------------***/
    /// @brief 工控机串口
    extern BSP::UART::UartID huart_host;
    /// @brief 遥控器串口
    extern BSP::UART::UartID huart_farcon;
    /// @brief 里程计串口
    extern BSP::UART::UartID huart_odom;
    /// @brief 其他串口
    extern BSP::UART::UartID huart_other;
    /// @brief 日志串口
    extern BSP::UART::UartID huart_log;     // 专用日志串口（如果需要）
    /// @brief 光通信串口
    extern BSP::UART::UartID huart_optical;
    /// @brief SICK激光串口
    extern BSP::UART::UartID huart_sick;
    /***---------------     框架CAN    ---------------***/
    /// @brief 框架所用CAN句柄
    extern BSP::CAN::CanID hcan_main;
    /// @brief 框架所用CAN句柄
    extern BSP::CAN::CanID hcan_sub;

    /***---------------     框架SPI总线    ---------------***/
    /// @brief 主 SPI 总线
    extern BSP::SPI::SpiID spi_main_bus;
    /// @brief IMU SPI 总线
    extern BSP::SPI::SpiID spi_imu_bus;
    /// @brief 扩展 SPI 总线
    extern BSP::SPI::SpiID spi_ext_bus;

    
}
