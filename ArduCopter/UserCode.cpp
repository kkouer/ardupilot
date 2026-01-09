#include "Copter.h"
#include "AP_Scale_Driver.h"

// Define the global instance here
AP_Scale_Driver scale_driver;

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    scale_driver.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    scale_driver.update();
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
    scale_driver.send_mavlink();

    // Simple RC trigger for Tare (using Channel 7)
    // 0-indexed: 6 = Channel 7
    // RC_Channel *chan7 = RC_Channels::rc_channel(6);
    // if (chan7 != nullptr) {
    //     uint16_t pwm = chan7->get_radio_in();
    //     static bool tare_triggered = false;

    //     // Toggle logic: Low -> High triggers action
    //     if (pwm > 1700 && !tare_triggered) {
    //         scale_driver.tare();
    //         tare_triggered = true;
    //     } else if (pwm < 1300) {
    //         tare_triggered = false; // Reset when switch goes low
    //     }
    // }
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
    // put your 1Hz code here
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
