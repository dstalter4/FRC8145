////////////////////////////////////////////////////////////////////////////////
/// @file   EastTechLed.cpp
/// @author David Stalter
///
/// @details
/// Implements functionality for controlling LEDs on a robot.
///
/// Copyright (c) 2026 East Technical High School
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
#include <cctype>               // for alphanumeric character checking

// C INCLUDES
// (none)

// C++ INCLUDES
#include "EastTechLed.hpp"           // class declaration


////////////////////////////////////////////////////////////////
/// @method EastTechLedController::EastTechLedController
///
/// Constructor
///
////////////////////////////////////////////////////////////////
EastTechLedController::EastTechLedController(uint32_t numLeds, int candleCanId, const CANBus & rCandleCanBus, GetAllianceLambdaType getAllianceLambda) :
    m_GetAllianceLambda(getAllianceLambda),
    m_pCandle(new CANdle(candleCanId, rCandleCanBus)),
    m_LedStripSolidColor(0, (numLeds - 1)),
    m_RainbowAnimation (0, (numLeds - 1)),
    m_EmptyAnimation(0)
{
    // Alliance color must manually be set later because
    // it may not be known when constructors run.

    CANdleConfiguration candleConfig;
    candleConfig.LED.StripType = StripTypeValue::GRB;
    m_pCandle->GetConfigurator().Apply(candleConfig);

    m_RainbowAnimation.FrameRate = 50_Hz;

    // Default behavior at construction is the rainbow animation
    m_pCandle->SetControl(m_RainbowAnimation);
}



////////////////////////////////////////////////////////////////
/// @method EastTechLedController::SetAnimation
///
/// This method will set the LED strip to a specified animation.
///
////////////////////////////////////////////////////////////////
void EastTechLedController::SetAnimation(LedAnimation ledAnimation)
{
    switch (ledAnimation)
    {
        case LedAnimation::LED_RAINBOW_ANIMATION:
        {
            m_pCandle->SetControl(m_RainbowAnimation);
            break;
        }
        default:
        {
            break;
        }
    }
}
