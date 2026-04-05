//
// Created by liaohy on 3/31/26.
//

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os2.h"
#include "usb_device.h"

#ifdef __cplusplus
}
#endif

#include "App.hpp"

extern "C" void StartDefaultTask(void *argument)
{
    static_cast<void>(argument);
    MX_USB_DEVICE_Init();
    appInit();

    for(;;)
    {
        appLoop();
        osDelay(100);
    }
}
