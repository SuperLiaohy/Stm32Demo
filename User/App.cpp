#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"

#ifdef __cplusplus
}
#endif

#include "App.hpp"

#include "Format.h"
#include "Logger.h"
#include "stm32/Delay.hpp"
#include "stm32/Exti.hpp"
#include "stm32/Gpio.hpp"
#include "stm32/Spi.hpp"
#include "stm32/USBCDC.hpp"
#include "bmi088/Bmi088.hpp"

namespace {
volatile bool accelDataReady = false;
volatile bool gyroDataReady = false;
bool appReady = false;

Spi imuSpi{&hspi2};
Gpio accelCs{BMI088_CS1_GPIO_Port, BMI088_CS1_Pin};
Gpio gyroCs{BMI088_CS2_GPIO_Port, BMI088_CS2_Pin};
Exti accelExti{BMI088_INT1_Pin};
Exti gyroExti{BMI088_INT3_Pin};

EP::Driver::Bmi088<Spi, Gpio, Gpio, Delay, Exti> imu{imuSpi, accelCs, gyroCs};

EP::Driver::Bmi088Data accelData{};
EP::Driver::Bmi088Data gyroData{};
bool accelDataValid = false;
bool gyroDataValid = false;

EP::Component::Logger<CDCPrint> logger{};

void logImuSample() noexcept {
    if (!accelDataValid || !gyroDataValid) { return; }

    logger.log<EP::Component::Str{"BMI088,A,{.3},{.3},{.3},G,{.3},{.3},{.3}\r\n"}>(
        accelData.x,
        accelData.y,
        accelData.z,
        gyroData.x,
        gyroData.y,
        gyroData.z
    );
    logger.flush();
}

void onAccelDrdy(void*) noexcept {
    accelDataReady = true;
}

void onGyroDrdy(void*) noexcept {
    gyroDataReady = true;
}

bool checkOk(const BspStatus status) noexcept {
    return status == BspStatus::ok;
}
}

void appInit() noexcept {

    accelDataReady = false;
    gyroDataReady = false;
    accelDataValid = false;
    gyroDataValid = false;
    appReady = false;

    if (!checkOk(accelExti.registerCallback(onAccelDrdy, nullptr))) { return; }
    if (!checkOk(gyroExti.registerCallback(onGyroDrdy, nullptr))) { return; }

    if (!checkOk(imu.init())) { return; }
    if (!checkOk(imu.configureAccel(EP::Driver::Bmi088AccelConfig{}))) { return; }
    if (!checkOk(imu.configureGyro(EP::Driver::Bmi088GyroConfig{}))) { return; }

    if (!checkOk(imu.configureAccelDrdyInterrupt(
        EP::Driver::Bmi088AccelDrdyRoute::int1,
        EP::Driver::Bmi088IntActiveLevel::activeHigh,
        EP::Driver::Bmi088IntOutputMode::pushPull
    ))) {
        return;
    }

    if (!checkOk(imu.configureGyroDrdyInterrupt(
        EP::Driver::Bmi088GyroDrdyRoute::int3,
        EP::Driver::Bmi088IntActiveLevel::activeHigh,
        EP::Driver::Bmi088IntOutputMode::pushPull
    ))) {
        return;
    }

    appReady = true;
}

void appLoop() noexcept {
    if (!appReady) { return; }

    bool sampleUpdated = false;

    if (accelDataReady) {
        accelDataReady = false;
        if (checkOk(imu.readAccel(accelData))) {
            accelDataValid = true;
            sampleUpdated = true;
        }
    }

    if (gyroDataReady) {
        gyroDataReady = false;
        if (checkOk(imu.readGyro(gyroData))) {
            gyroDataValid = true;
            sampleUpdated = true;
        }
    }

    if (sampleUpdated) {
        logImuSample();
    }
}
