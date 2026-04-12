#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
}
#endif

#include "Imu.hpp"

#include <cmath>

namespace {
constexpr float kMsToS = 0.001F;
constexpr float kAccelNormEps = 1e-6F;
constexpr float kDefaultDtS = 0.001F;
}

namespace User {

Imu::Imu(Spi& spi,
         Gpio& accelCs,
         Gpio& gyroCs,
         Exti& accelExti,
         Exti& gyroExti) noexcept :
    spi_(spi),
    accelCs_(accelCs),
    gyroCs_(gyroCs),
    accelExti_(accelExti),
    gyroExti_(gyroExti),
    bmi088_(spi_, accelCs_, gyroCs_) {
}

Imu::AttitudeEkf::StateVec Imu::AttitudeEkf::f(const StateVec& x, const CtrlVec& u, const float dt) const noexcept {
    StateVec nextX = x;

    const float q0 = x(0);
    const float q1 = x(1);
    const float q2 = x(2);
    const float q3 = x(3);

    const float bx = x(4);
    const float by = x(5);
    const float bz = x(6);

    const float wx = u(0) - bx;
    const float wy = u(1) - by;
    const float wz = u(2) - bz;

    const float qDot0 = -0.5F * (wx * q1 + wy * q2 + wz * q3);
    const float qDot1 = 0.5F * (wx * q0 + wy * q3 - wz * q2);
    const float qDot2 = 0.5F * (wy * q0 + wz * q1 - wx * q3);
    const float qDot3 = 0.5F * (wz * q0 + wx * q2 - wy * q1);

    nextX(0) = q0 + qDot0 * dt;
    nextX(1) = q1 + qDot1 * dt;
    nextX(2) = q2 + qDot2 * dt;
    nextX(3) = q3 + qDot3 * dt;

    const float qNorm = std::sqrt(
        nextX(0) * nextX(0) +
        nextX(1) * nextX(1) +
        nextX(2) * nextX(2) +
        nextX(3) * nextX(3)
    );

    if (qNorm > kAccelNormEps) {
        nextX(0) /= qNorm;
        nextX(1) /= qNorm;
        nextX(2) /= qNorm;
        nextX(3) /= qNorm;
    } else {
        nextX(0) = 1.0F;
        nextX(1) = 0.0F;
        nextX(2) = 0.0F;
        nextX(3) = 0.0F;
    }

    return nextX;
}

Imu::AttitudeEkf::MeasVec Imu::AttitudeEkf::h(const StateVec& x) const noexcept {
    MeasVec zHat{};

    const float q0 = x(0);
    const float q1 = x(1);
    const float q2 = x(2);
    const float q3 = x(3);

    zHat(0) = 2.0F * (q1 * q3 - q0 * q2);
    zHat(1) = 2.0F * (q0 * q1 + q2 * q3);
    zHat(2) = (q0 * q0) - (q1 * q1) - (q2 * q2) + (q3 * q3);
    return zHat;
}

Imu::AttitudeEkf::StateMat Imu::AttitudeEkf::F(const StateVec& x, const CtrlVec& u, const float dt) const noexcept {
    StateMat jacobian = StateMat::eyes();

    const float q0 = x(0);
    const float q1 = x(1);
    const float q2 = x(2);
    const float q3 = x(3);

    const float bx = x(4);
    const float by = x(5);
    const float bz = x(6);

    const float wx = u(0) - bx;
    const float wy = u(1) - by;
    const float wz = u(2) - bz;

    const float halfDt = 0.5F * dt;

    jacobian(0, 1) = -halfDt * wx;
    jacobian(0, 2) = -halfDt * wy;
    jacobian(0, 3) = -halfDt * wz;
    jacobian(0, 4) = halfDt * q1;
    jacobian(0, 5) = halfDt * q2;
    jacobian(0, 6) = halfDt * q3;

    jacobian(1, 0) = halfDt * wx;
    jacobian(1, 2) = -halfDt * wz;
    jacobian(1, 3) = halfDt * wy;
    jacobian(1, 4) = -halfDt * q0;
    jacobian(1, 5) = -halfDt * q3;
    jacobian(1, 6) = halfDt * q2;

    jacobian(2, 0) = halfDt * wy;
    jacobian(2, 1) = halfDt * wz;
    jacobian(2, 3) = -halfDt * wx;
    jacobian(2, 4) = halfDt * q3;
    jacobian(2, 5) = -halfDt * q0;
    jacobian(2, 6) = -halfDt * q1;

    jacobian(3, 0) = halfDt * wz;
    jacobian(3, 1) = -halfDt * wy;
    jacobian(3, 2) = halfDt * wx;
    jacobian(3, 4) = -halfDt * q2;
    jacobian(3, 5) = halfDt * q1;
    jacobian(3, 6) = -halfDt * q0;

    return jacobian;
}

Imu::AttitudeEkf::StateToMeasMat Imu::AttitudeEkf::H(const StateVec& x) const noexcept {
    StateToMeasMat measurementJacobian = StateToMeasMat::zeros();

    const float q0 = x(0);
    const float q1 = x(1);
    const float q2 = x(2);
    const float q3 = x(3);

    measurementJacobian(0, 0) = -2.0F * q2;
    measurementJacobian(0, 1) = 2.0F * q3;
    measurementJacobian(0, 2) = -2.0F * q0;
    measurementJacobian(0, 3) = 2.0F * q1;

    measurementJacobian(1, 0) = 2.0F * q1;
    measurementJacobian(1, 1) = 2.0F * q0;
    measurementJacobian(1, 2) = 2.0F * q3;
    measurementJacobian(1, 3) = 2.0F * q2;

    measurementJacobian(2, 0) = 2.0F * q0;
    measurementJacobian(2, 1) = -2.0F * q1;
    measurementJacobian(2, 2) = -2.0F * q2;
    measurementJacobian(2, 3) = 2.0F * q3;

    return measurementJacobian;
}

Imu::AttitudeEkf::StateMat Imu::AttitudeEkf::Q() const noexcept {
    StateMat processNoise = StateMat::zeros();
    processNoise(0, 0) = 1.0e-5F;
    processNoise(1, 1) = 1.0e-5F;
    processNoise(2, 2) = 1.0e-5F;
    processNoise(3, 3) = 1.0e-5F;
    processNoise(4, 4) = 1.0e-6F;
    processNoise(5, 5) = 1.0e-6F;
    processNoise(6, 6) = 1.0e-6F;
    return processNoise;
}

Imu::AttitudeEkf::MeasMat Imu::AttitudeEkf::R() const noexcept {
    MeasMat measurementNoise = MeasMat::zeros();
    measurementNoise(0, 0) = 2.5e-2F;
    measurementNoise(1, 1) = 2.5e-2F;
    measurementNoise(2, 2) = 2.5e-2F;
    return measurementNoise;
}

BspStatus Imu::init() noexcept {
    initialized_ = false;
    accelDataReady_ = false;
    gyroDataReady_ = false;
    lastUpdateTickMs_ = HAL_GetTick();

    attitude_ = {};

    typename AttitudeEkf::StateVec initialState = AttitudeEkf::StateVec::zeros();
    initialState(0) = 1.0F;
    ekf_.setState(initialState);
    ekf_.setCovariance(AttitudeEkf::StateMat::eyes());

    if (!checkOk(accelExti_.registerCallback(onAccelDrdy, this))) {
        return BspStatus::hardwareError;
    }
    if (!checkOk(gyroExti_.registerCallback(onGyroDrdy, this))) {
        return BspStatus::hardwareError;
    }

    if (!checkOk(bmi088_.init())) {
        return BspStatus::hardwareError;
    }
    if (!checkOk(bmi088_.configureAccel(EP::Driver::Bmi088AccelConfig{}))) {
        return BspStatus::hardwareError;
    }
    if (!checkOk(bmi088_.configureGyro(EP::Driver::Bmi088GyroConfig{}))) {
        return BspStatus::hardwareError;
    }

    if (!checkOk(bmi088_.configureAccelDrdyInterrupt(
        EP::Driver::Bmi088AccelDrdyRoute::int1,
        EP::Driver::Bmi088IntActiveLevel::activeHigh,
        EP::Driver::Bmi088IntOutputMode::pushPull
    ))) {
        return BspStatus::hardwareError;
    }

    if (!checkOk(bmi088_.configureGyroDrdyInterrupt(
        EP::Driver::Bmi088GyroDrdyRoute::int3,
        EP::Driver::Bmi088IntActiveLevel::activeHigh,
        EP::Driver::Bmi088IntOutputMode::pushPull
    ))) {
        return BspStatus::hardwareError;
    }

    initialized_ = true;
    return BspStatus::ok;
}

bool Imu::update() noexcept {
    if (!initialized_) {
        return false;
    }

    const std::uint32_t currentTickMs = HAL_GetTick();
    const std::uint32_t deltaTickMs = currentTickMs - lastUpdateTickMs_;
    lastUpdateTickMs_ = currentTickMs;

    const float dt = (deltaTickMs > 0U) ? (static_cast<float>(deltaTickMs) * kMsToS) : kDefaultDtS;

    bool updated = false;

    if (gyroDataReady_) {
        gyroDataReady_ = false;
        if (checkOk(bmi088_.readGyro(gyroData_))) {
            typename AttitudeEkf::CtrlVec u = AttitudeEkf::CtrlVec::zeros();
            u(0) = gyroData_.x * kDegToRad;
            u(1) = gyroData_.y * kDegToRad;
            u(2) = gyroData_.z * kDegToRad;

            ekf_.predict(u, dt);
            updated = true;
        }
    }

    if (accelDataReady_) {
        accelDataReady_ = false;
        if (checkOk(bmi088_.readAccel(accelData_))) {
            typename AttitudeEkf::MeasVec z = AttitudeEkf::MeasVec::zeros();

            const float accelNorm = std::sqrt(
                accelData_.x * accelData_.x +
                accelData_.y * accelData_.y +
                accelData_.z * accelData_.z
            );

            if (accelNorm > kAccelNormEps) {
                z(0) = accelData_.x / accelNorm;
                z(1) = accelData_.y / accelNorm;
                z(2) = accelData_.z / accelNorm;
                updated = ekf_.update(z) || updated;
            }
        }
    }

    const auto& state = ekf_.getState();
    typename AttitudeEkf::StateVec normalizedState = state;

    const float qNorm = std::sqrt(
        state(0) * state(0) +
        state(1) * state(1) +
        state(2) * state(2) +
        state(3) * state(3)
    );

    if (qNorm > kAccelNormEps) {
        normalizedState(0) = state(0) / qNorm;
        normalizedState(1) = state(1) / qNorm;
        normalizedState(2) = state(2) / qNorm;
        normalizedState(3) = state(3) / qNorm;
    } else {
        normalizedState(0) = 1.0F;
        normalizedState(1) = 0.0F;
        normalizedState(2) = 0.0F;
        normalizedState(3) = 0.0F;
    }

    ekf_.setState(normalizedState);

    const EP::Math::UnitQuat quat{
        normalizedState(0),
        normalizedState(1),
        normalizedState(2),
        normalizedState(3)
    };
    const auto euler = quat.rotAngle<EP::Math::RotAngle::FIXED_XYZ>();
    if (euler.has_value()) {
        attitude_.roll = (*euler)[0];
        attitude_.pitch = (*euler)[1];
        attitude_.yaw = (*euler)[2];
    }

    return updated;
}

Attitude Imu::getAttitude() const noexcept {
    return attitude_;
}

void Imu::onAccelDrdy(void* userContext) noexcept {
    if (userContext == nullptr) {
        return;
    }

    auto* instance = static_cast<Imu*>(userContext);
    instance->accelDataReady_ = true;
}

void Imu::onGyroDrdy(void* userContext) noexcept {
    if (userContext == nullptr) {
        return;
    }

    auto* instance = static_cast<Imu*>(userContext);
    instance->gyroDataReady_ = true;
}

bool Imu::checkOk(const BspStatus status) noexcept {
    return status == BspStatus::ok;
}

}
