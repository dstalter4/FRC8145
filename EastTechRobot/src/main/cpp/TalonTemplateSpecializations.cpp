////////////////////////////////////////////////////////////////////////////////
/// @file   TalonTemplateSpecializations.cpp
/// @author David Stalter
///
/// @details
/// A translation unit to build the required template specializations for the
/// different types of talons used on the robot.
///
/// Copyright (c) 2021 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

// Indicate the template specializations should be built
#define BUILD_TALON_TEMPLATE_SPECIALIZATIONS

// Build for the SRX talons
#define TALON_TYPE TalonSRX
#include "TalonMotorGroup.cpp"
#undef TALON_TYPE

// Build for the Falcon talons
#define TALON_TYPE TalonFX
#include "TalonMotorGroup.cpp"
#undef TALON_TYPE
