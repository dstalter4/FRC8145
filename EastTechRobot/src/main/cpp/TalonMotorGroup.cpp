////////////////////////////////////////////////////////////////////////////////
/// @file   TalonMotorGroup.cpp
/// @author David Stalter
///
/// @details
/// A class designed to work with a group of CAN Talon speed controllers working
/// in tandem.
///
/// Copyright (c) 2024 East Technical High School
////////////////////////////////////////////////////////////////////////////////

// Control building of the template functions for the TalonMotorGroup class.
// There are two ways we can build the template functions.  The compiler can
// handle everything like normal with the function bodies in the header (which
// is the current approach).  The other option is that we explicitly build
// specializations of the entire class.  In that case, the function bodies
// should all be placed back in this file.  TalonTemplateSpecializations.cpp
// will then handle building the template functions with the types needed.
// The advantage of handling the specializations ourselves is that things
// won't be inline, and use of new types of talons will result in link errors.

#ifdef BUILD_TALON_TEMPLATE_SPECIALIZATIONS

#ifndef TALON_TYPE
#error "Talon type is not defined!"
#endif

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "EastTechTalon.hpp"        // for class declaration

// STATIC MEMBER DATA
// (none)

// Place the template functions here and change their parameter from TalonType to TALON_TYPE.

#endif // BUILD_TALON_TEMPLATE_SPECIALIZATIONS
