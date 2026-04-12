#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"

#ifdef __cplusplus
}
#endif

#include "App.hpp"

#include "Format.h"
#include "Imu.hpp"
#include "Logger.h"
#include "stm32/Exti.hpp"
#include "stm32/Gpio.hpp"
#include "stm32/Spi.hpp"
#include "stm32/USBCDC.hpp"

namespace {
bool appReady = false;

Spi imuSpi{&hspi2};
Gpio accelCs{BMI088_CS1_GPIO_Port, BMI088_CS1_Pin};
Gpio gyroCs{BMI088_CS2_GPIO_Port, BMI088_CS2_Pin};
Exti accelExti{BMI088_INT1_Pin};
Exti gyroExti{BMI088_INT3_Pin};

User::Imu imu{imuSpi, accelCs, gyroCs, accelExti, gyroExti};

EP::Component::Logger<CDCPrint> logger{};

void logImuSample(const User::Attitude& attitude) noexcept {
    logger.log<EP::Component::Str{"IMU,RPY,{.3},{.3},{.3}\r\n"}>(
        attitude.roll,
        attitude.pitch,
        attitude.yaw
    );
    logger.flush();
}

bool checkOk(const BspStatus status) noexcept {
    return status == BspStatus::ok;
}
}

void appInit() noexcept {
    appReady = false;

    appReady = checkOk(imu.init());
}

void appLoop() noexcept {
    if (!appReady) { return; }

    if (imu.update()) {
        // logImuSample(imu.getAttitude());
    }
}
