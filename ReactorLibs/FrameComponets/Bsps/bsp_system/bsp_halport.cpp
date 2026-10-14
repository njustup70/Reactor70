#include "bsp_halport.hpp"
#include "bsp_hardware.hpp"

namespace Hardware
{
    float MainFreq_MHz = 72.0f;

    bool RTTLogAtUart = false;
    bool RTTLogUartItrp = false;

    /**-----    配置CAN总线     -----**/
    BSP::CAN::CanID hcan_main = nullptr;
    BSP::CAN::CanID hcan_sub = nullptr;

    /**-----    配置串口    -----**/
    BSP::UART::UartID huart_host = nullptr;
    BSP::UART::UartID huart_farcon = nullptr;
    BSP::UART::UartID huart_odom = nullptr;
    BSP::UART::UartID huart_other = nullptr;
    BSP::UART::UartID huart_log = nullptr;   
    BSP::UART::UartID huart_optical = nullptr;  
    BSP::UART::UartID huart_sick = nullptr;  

    /**-----    配置SPI总线    -----**/
    BSP::SPI::SpiID spi_main_bus = nullptr;
    BSP::SPI::SpiID spi_imu_bus = nullptr;
    BSP::SPI::SpiID spi_ext_bus = nullptr;

    /**-----    配置定时器    -----**/
    TIM_HandleTypeDef* htim_led = nullptr;

    /**-----    配置 GPIO   -----**/
    
}
