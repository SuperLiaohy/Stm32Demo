#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"

#ifdef __cplusplus
}
#endif

#include "App.hpp"

#include <cstring>

#include "Format.h"
#include "Logger.h"
#include "stm32/Delay.hpp"
#include "stm32/Exti.hpp"
#include "stm32/Gpio.hpp"
#include "stm32/Spi.hpp"
#include "bmi088/Bmi088.hpp"

namespace {
volatile bool accelDataReady = false;
volatile bool gyroDataReady = false;
bool appReady = false;

Spi imuSpi{&hspi2};
Delay imuDelay{};
Gpio accelCs{BMI088_CS1_GPIO_Port, BMI088_CS1_Pin};
Gpio gyroCs{BMI088_CS2_GPIO_Port, BMI088_CS2_Pin};
Exti accelExti{BMI088_INT1_Pin};
Exti gyroExti{BMI088_INT3_Pin};

EP::Driver::Bmi088<Spi, Gpio, Gpio, Delay> imu{imuSpi, accelCs, gyroCs, imuDelay};

EP::Driver::Bmi088Data accelData{};
EP::Driver::Bmi088Data gyroData{};
bool accelDataValid = false;
bool gyroDataValid = false;

char txPendingBuffer[160]{};
uint16_t txPendingLength = 0U;

void cdcPrint(uint8_t* data, size_t len) {
    if ((data == nullptr) || (len == 0U) || (len > sizeof(txPendingBuffer)) || (txPendingLength != 0U)) {
        return;
    }

    std::memcpy(txPendingBuffer, data, len);
    txPendingLength = static_cast<uint16_t>(len);
}

EP::Component::Logger<cdcPrint> logger{};

void serviceCdcTx() noexcept {
    if (txPendingLength == 0U) { return; }

    const uint8_t txStatus = CDC_Transmit_HS(reinterpret_cast<uint8_t*>(txPendingBuffer), txPendingLength);
    if (txStatus == USBD_OK) {
        txPendingLength = 0U;
        return;
    }

    if (txStatus != USBD_BUSY) {
        txPendingLength = 0U;
    }
}

void logImuSample() noexcept {
    if ((txPendingLength != 0U) || !accelDataValid || !gyroDataValid) { return; }

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
    txPendingLength = 0U;
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

    serviceCdcTx();

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
