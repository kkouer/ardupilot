#include "Copter.h"

// setup UART at 57600 kkouer add
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == NULL)
    {
        // that UART doesn't exist on this platform
        return;
    }
    /// begin(baudrate,Rx,Tx)
    uart->begin(115200);
}

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    //防止参数错误导致初始化失败
    if(copter.g.data16_port_num<0)
        return;
    // put your initialisation code here
    // this will be called once at start-up
    gcs().send_text(MAV_SEVERITY_INFO, "user hook init!");
    setup_uart(hal.serial(4), "uartD");
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    
    uint8_t portNum = 4;//(uint8_t)(copter.g.data16_port_num);

    char incoming[256];
    uint8_t data;
    uint8_t index = 0;
    int16_t numc;

    numc = hal.serial(portNum)->available();

    // gcs().send_text(MAV_SEVERITY_INFO, "data16 port num %d",portNum);
    //gcs().send_text(MAV_SEVERITY_INFO, "data16 length %u ", numc);

    // 如果有可用数据
    if (numc > 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "data length %u", numc);  // 输出数据长度

        // 读取数据并存储
        for (int16_t i = 0; i < numc && index < sizeof(incoming) - 1; i++) {
            data = hal.serial(portNum)->read();
            incoming[index++] = (char)data;  // 将字节数据存储为字符
        }

        // 确保字符串以 '\0' 结束，便于正确显示
        //incoming[index] = '\0';  // 字符串结束符

        // 输出接收到的数据（作为字符串）
        gcs().send_text(MAV_SEVERITY_INFO, "Received data: %s", incoming);

        // 也可以选择以十六进制方式输出接收到的字节
        // gcs().send_text(MAV_SEVERITY_INFO, "Received data (hex):");
        // for (int i = 0; i < numc; i++) {
        //     gcs().send_text(MAV_SEVERITY_INFO, "0x%02X", incoming[i]);  // 以十六进制格式输出每个字节
        // }
    }
    // if (numc > 0)
    // {
    //     mavlink_data16_t p1;
    //     memset(&p1, 0, sizeof(p1));

    //     hal.serial(portNum)->printf("in loop! length: %u \n", numc);

    //     //防止溢出错误
    //     if (numc > 16)
    //         numc = 16;
    //     for (int16_t i = 0; i < numc; i++)
    //     {
    //         data = hal.serial(portNum)->read();
    //         if (i < numc)
    //         {
    //             if (i == 0)
    //             {
    //                 p1.type = (uint8_t)data;
    //             }
    //             else
    //             {
    //                 p1.data[i - 1] = (uint8_t)data;
    //             }
    //             incoming[index++] = data;
    //         }
    //         if (i == numc - 1)
    //         {
    //             incoming[index] = '\0';
    //             index = 0;
    //             hal.serial(portNum)->printf("value: %s\n", incoming);
    //             p1.len = numc;

    //             mavlink_msg_data16_send(MAVLINK_COMM_0, p1.type, p1.len, p1.data);
    //         }
    //     }
    // }

    // put your 1Hz code here
    // gcs().send_text(MAV_SEVERITY_INFO, "user hook 1 hz!");
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
