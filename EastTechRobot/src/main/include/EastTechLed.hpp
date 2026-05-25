////////////////////////////////////////////////////////////////////////////////
/// @file   EastTechLed.hpp
/// @author David Stalter
///
/// @details
/// Implements functionality for controlling LEDs on a robot.
///
/// Copyright (c) 2026 East Technical High School
////////////////////////////////////////////////////////////////////////////////

#ifndef EASTTECHLED_HPP
#define EASTTECHLED_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/DriverStation.h"                              // for alliance

// C++ INCLUDES
#include "ctre/phoenix6/CANBus.hpp"                         // for creating CANBus objects
#include "ctre/phoenix6/CANdle.hpp"                         // for interacting with the CANdle
#include "ctre/phoenix6/controls/RainbowAnimation.hpp"      // for creating animations on the CANdle

using namespace frc;
using namespace ctre::phoenix6;
using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::controls;
using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6::signals;


////////////////////////////////////////////////////////////////
/// @class EastTechLedController
///
/// Declarations for managing the LEDs on a robot.
///
////////////////////////////////////////////////////////////////
class EastTechLedController
{
public:
    typedef std::function<DriverStation::Alliance()> GetAllianceLambdaType;

    enum class LedAnimation
    {
        LED_NO_ANIMATION,
        LED_RAINBOW_ANIMATION
    };

    // Constructor
    EastTechLedController(uint32_t numLeds, int candleCanId, const CANBus & rCandleCanBus, GetAllianceLambdaType getAllianceLambda);

    inline void SetLedsToAllianceColor();
    void SetAnimation(LedAnimation ledAnimation);

private:
    GetAllianceLambdaType m_GetAllianceLambda;                  // Lambda to retrieve the alliance color
    CANdle * m_pCandle;                                         // Controls an RGB LED strip
    SolidColor m_LedStripSolidColor;                            // Used when setting the LEDs to RGB values
    RainbowAnimation m_RainbowAnimation;                        // Rainbow animation configuration (brightness, speed, # LEDs)
    EmptyAnimation m_EmptyAnimation;                            // Empty animation to clear the configuration in a slot
    static constexpr const RGBWColor RGBW_OFF{0, 0, 0, 0};      // Common RGBWColor expression representing LEDs off
};



////////////////////////////////////////////////////////////////
/// @method EastTechLedController::SetLedsToAllianceColor
///
/// Sets the LEDs to the alliance color.
///
////////////////////////////////////////////////////////////////
void EastTechLedController::SetLedsToAllianceColor()
{
    // If there is an active animation, it will conflict with the commands below
    m_pCandle->SetControl(m_EmptyAnimation);

    switch (m_GetAllianceLambda())
    {
        case DriverStation::Alliance::kRed:
        {
            constexpr const RGBWColor RGBW_RED{255, 0, 0, 0};
            m_pCandle->SetControl(m_LedStripSolidColor.WithColor(RGBW_RED));
            break;
        }
        case DriverStation::Alliance::kBlue:
        {
            constexpr const RGBWColor RGBW_BLUE{0, 0, 255, 0};
            m_pCandle->SetControl(m_LedStripSolidColor.WithColor(RGBW_BLUE));
            break;
        }
        default:
        {
            break;
        }
    }
}

#endif // EASTTECHLED_HPP
