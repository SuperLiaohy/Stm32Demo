#pragma once

#include <cstdint>

#include "stm32/Delay.hpp"
#include "stm32/Exti.hpp"
#include "stm32/Gpio.hpp"
#include "stm32/Spi.hpp"
#include "bmi088/Bmi088.hpp"
#include "Filter/Ekf.hpp"
#include "Quaternion/UnitQuat.hpp"

namespace User {

struct Attitude {
    float roll{0.0F};
    float pitch{0.0F};
    float yaw{0.0F};
};

class Imu {
public:
    class AttitudeEkf final : public EP::Math::Ekf<AttitudeEkf, 7U, 3U, 3U> {
    public:
        using Base = EP::Math::Ekf<AttitudeEkf, 7U, 3U, 3U>;
        using StateVec = typename Base::StateVec;
        using MeasVec = typename Base::MeasVec;
        using CtrlVec = typename Base::CtrlVec;
        using StateMat = typename Base::StateMat;
        using MeasMat = typename Base::MeasMat;
        using MeasToStateMat = typename Base::MeasToStateMat;
        using StateToMeasMat = typename Base::StateToMeasMat;

        [[nodiscard]] StateVec f(const StateVec& x, const CtrlVec& u, float dt) const noexcept;
        [[nodiscard]] MeasVec h(const StateVec& x) const noexcept;
        [[nodiscard]] StateMat F(const StateVec& x, const CtrlVec& u, float dt) const noexcept;
        [[nodiscard]] StateToMeasMat H(const StateVec& x) const noexcept;
        [[nodiscard]] StateMat Q() const noexcept;
        [[nodiscard]] MeasMat R() const noexcept;
    };

    using Bmi088Type = EP::Driver::Bmi088<Spi, Gpio, Gpio, Delay, Exti>;

    Imu(Spi& spi,
        Gpio& accelCs,
        Gpio& gyroCs,
        Exti& accelExti,
        Exti& gyroExti) noexcept;

    Imu(const Imu&) = delete;
    Imu& operator=(const Imu&) = delete;
    Imu(Imu&&) = delete;
    Imu& operator=(Imu&&) = delete;

    BspStatus init() noexcept;
    [[nodiscard]] bool update() noexcept;
    [[nodiscard]] Attitude getAttitude() const noexcept;

private:
    static constexpr float kDegToRad = 0.01745329251994329577F;

    static void onAccelDrdy(void* userContext) noexcept;
    static void onGyroDrdy(void* userContext) noexcept;

    [[nodiscard]] static bool checkOk(BspStatus status) noexcept;

    Spi& spi_;
    Gpio& accelCs_;
    Gpio& gyroCs_;
    Exti& accelExti_;
    Exti& gyroExti_;

    Bmi088Type bmi088_;
    AttitudeEkf ekf_;

    EP::Driver::Bmi088Data accelData_{};
    EP::Driver::Bmi088Data gyroData_{};
    Attitude attitude_{};

    volatile bool accelDataReady_{false};
    volatile bool gyroDataReady_{false};
    bool initialized_{false};

    std::uint32_t lastUpdateTickMs_{0U};
};

}
