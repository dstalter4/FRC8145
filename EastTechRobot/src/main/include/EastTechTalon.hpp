////////////////////////////////////////////////////////////////////////////////
/// @file   EastTechTalon.hpp
/// @author David Stalter
///
/// @details
/// A class designed to work with a group of CAN Talon speed controllers working
/// in tandem.
///
/// Copyright (c) 2025 East Technical High School
////////////////////////////////////////////////////////////////////////////////

#ifndef EASTTECHTALON_HPP
#define EASTTECHTALON_HPP

// SYSTEM INCLUDES
#include <cstdio>                               // for std::snprintf

// C INCLUDES
#include "ctre/phoenix6/TalonFX.hpp"            // for CTRE TalonFX API
#include "frc/smartdashboard/SmartDashboard.h"  // for interacting with the smart dashboard

// C++ INCLUDES
#include "RobotUtils.hpp"                       // for ConvertCelsiusToFahrenheit

using namespace frc;
using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::controls;
using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6::signals;


////////////////////////////////////////////////////////////////
/// @namespace EastTechTalon
///
/// Namespace that contains declarations for interacting with
/// Talon speed controllers specific to East Tech.
///
////////////////////////////////////////////////////////////////
namespace EastTech
{
namespace Talon
{
    // Represents how a motor will be controlled
    enum MotorGroupControlMode
    {
        LEADER,                 // First motor in a group
        FOLLOW,                 // Motor follows the leader
        FOLLOW_INVERSE,         // Motor follows the leader, but inverse
        INDEPENDENT,            // Motor needs to be set independently
        INVERSE,                // Motor is the inverse value of the leader
        INDEPENDENT_OFFSET,     // Motor is set independently, but with a different value from leader
        INVERSE_OFFSET,         // Motor is set independently, but with the a different inverse value from leader
        CUSTOM                  // Motor needs to be set later to an option above
    };

    // Represents a combination of objects to use with a TalonFX motor controller
    struct TalonFxMotorController
    {
        // The Phoenix 6 API requires using different objects with SetControl()
        // function calls.  Create different possible objects to the main robot
        // code doesn't have to worry about it.
        TalonFX * m_pTalonFx;
        DutyCycleOut m_DutyCycleOut;
        PositionVoltage m_PositionVoltage;
        TalonFXConfiguration m_MotorConfiguration;

        // Constructor
        TalonFxMotorController(int canId) :
            m_pTalonFx(new TalonFX(canId)),
            m_DutyCycleOut(0.0),
            m_PositionVoltage(0.0_tr),
            m_MotorConfiguration()
        {
            m_pTalonFx->GetConfigurator().Apply(m_MotorConfiguration);
            m_pTalonFx->ClearStickyFaults();
        }

        // Applies the struct-local configuration, which may have been updated externally
        void ApplyConfiguration()
        {
            (void)m_pTalonFx->GetConfigurator().Apply(m_MotorConfiguration);
        }

        // Applies a configuration.  This will not update the struct-local
        // configuration and directly overwrites the configuration on the device.
        // Template type must match CTRE options.
        template <typename ConfigType>
        void ApplyConfiguration(ConfigType config)
        {
            // For now, don't let this be used
            ASSERT(false);
            (void)m_pTalonFx->GetConfigurator().Apply(config);
        }

        // Set the output using duty cycle
        void SetDutyCycle(double dutyCycle)
        {
            (void)m_pTalonFx->SetControl(m_DutyCycleOut.WithOutput(dutyCycle));
        }

        // Set the output to hold a specified position
        void SetPositionVoltage(double angle)
        {
            units::angle::degree_t degrees(angle);
            units::angle::turn_t turns(degrees);
            (void)m_pTalonFx->SetControl(m_PositionVoltage.WithPosition(turns));
        }
    };

    // A structure that doesn't create real TalonFX objects.
    // Intended to be used to keep multiple robot configuration
    // options available (i.e. interchange with TalonMotorGroup).
    struct EmptyTalonFx
    {
        EmptyTalonFx(const char *, unsigned, unsigned, MotorGroupControlMode, NeutralModeValue, bool) {}

        // TalonMotorGroup stubs
        inline void Set(double) {}
        inline void Set(double, double) {}
        inline void DisplayStatusInformation() {}
        inline EmptyTalonFx * GetMotorObject(unsigned) { return &m_EmptyTalonFxObj; }

        // TalonFX stubs
        template <typename Type>
        inline void Apply(Type) {}
        inline void SetPosition(units::angle::turn_t) {}
        inline EmptyTalonFx & GetPosition() { return m_EmptyTalonFxObj; }
        inline EmptyTalonFx & GetConfigurator() { return m_EmptyTalonFxObj; }
        inline EmptyTalonFx & GetValue() { return m_EmptyTalonFxObj; }
        inline double value() { return 0.0; }

        // Singleton
        static EmptyTalonFx m_EmptyTalonFxObj;
    };

    static const bool CURRENT_LIMITING_ENABLED = false;
}
}



////////////////////////////////////////////////////////////////
/// @class TalonMotorGroup
///
/// Class that provides methods for interacting with a group of
/// Talon speed controllers.
///
/// @todo: Remove template.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
class TalonMotorGroup
{
public:

    typedef EastTech::Talon::MotorGroupControlMode MotorGroupControlMode;
    
    // Constructor
    TalonMotorGroup(
                     const char * pName,
                     unsigned numMotors,
                     unsigned leaderCanId,
                     MotorGroupControlMode nonLeaderControlMode,
                     NeutralModeValue neutralMode,
                     bool bIsDriveMotor = false
                   );

    // Retrieve a specific motor object
    TalonType * GetMotorObject(unsigned canId = GROUP_LEADER_CAN_ID);

    // @todo: This breaks the template parameter since it assumes TalonFX.
    // Retrieve the configuration for a specific motor object
    TalonFXConfiguration * GetMotorConfiguration(unsigned canId = GROUP_LEADER_CAN_ID);

    // Applies the struct-local configuration, which may have been updated externally
    void ApplyConfiguration(unsigned canId = APPLY_TO_ALL_PSEUDO_CAN_ID);

    // Applies a configuration.  This will not update the struct-local
    // configuration and directly overwrites the configuration on the device.
    // Template type must match CTRE options.
    template <typename ConfigType>
    void ApplyConfiguration(ConfigType config, unsigned canId = APPLY_TO_ALL_PSEUDO_CAN_ID);

    // Adds a new motor to a group
    bool AddMotorToGroup(MotorGroupControlMode controlMode, bool bIsDriveMotor = false);
    
    // Function to set the speed of each motor in the group
    void Set(double value, double offset = 0.0);
    
    // Function to set the motor group output to hold specified angle
    void SetAngle(double angle);
    
    // Sets the control mode of a motor in a group (intended for use with the CUSTOM group control mode)
    bool SetMotorInGroupControlMode(unsigned canId, MotorGroupControlMode controlMode);
    
    // Change Talon mode between brake/coast
    void SetCoastMode();
    void SetBrakeMode();
    
    // Return the value of the sensor connected to the Talon
    int GetEncoderValue();
    void TareEncoder();

    // Displays information to the driver station about the motor group
    void DisplayStatusInformation();
    
private:

    // Represents information about a single motor in a group
    struct MotorInfo
    {
        // Storage space for strings for the smart dashboard
        struct DisplayStrings
        {
            static const unsigned MAX_MOTOR_DISPLAY_STRING_LENGTH = 64U;
            char m_CurrentTemperatureString[MAX_MOTOR_DISPLAY_STRING_LENGTH];
            char m_HighestTemperatureString[MAX_MOTOR_DISPLAY_STRING_LENGTH];
            char m_ResetOccurredString[MAX_MOTOR_DISPLAY_STRING_LENGTH];
        };

        // Member data
        TalonType * m_pTalon;
        DutyCycleOut m_DutyCycleOut;
        PositionVoltage m_PositionVoltage;
        TalonFXConfiguration m_MotorConfiguration;
        const char * m_pName;
        MotorGroupControlMode m_ControlMode;
        unsigned m_CanId;
        double m_CurrentTemperature;
        double m_HighestTemperature;
        bool m_bResetOccurred;
        bool m_bIsDriveMotor;
        DisplayStrings m_DisplayStrings;
        
        MotorInfo(const char * pName, MotorGroupControlMode controlMode, NeutralModeValue neutralMode, unsigned canId, unsigned groupNumber, bool bIsDriveMotor = false) :
            m_pTalon(new TalonType(static_cast<int>(canId))),
            m_DutyCycleOut(0.0),
            m_PositionVoltage(0.0_tr),
            m_MotorConfiguration(),
            m_pName(pName),
            m_ControlMode(controlMode),
            m_CanId(canId),
            m_CurrentTemperature(0.0),
            m_HighestTemperature(0.0),
            m_bResetOccurred(false),
            m_bIsDriveMotor(bIsDriveMotor)
        {
            m_MotorConfiguration.MotorOutput.NeutralMode = neutralMode;

            if (controlMode == EastTech::Talon::FOLLOW_INVERSE)
            {
                m_MotorConfiguration.MotorOutput.Inverted = true;
            }

            // @todo: Move in sensor too?
            if (EastTech::Talon::CURRENT_LIMITING_ENABLED && bIsDriveMotor)
            {
                // Limits were 40.0, 55.0, 0.1
                m_MotorConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 55.0_A;
                m_MotorConfiguration.CurrentLimits.SupplyCurrentLimit = 60.0_A;
                m_MotorConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.1_s;
                m_MotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
            }

            (void)m_pTalon->GetConfigurator().Apply(m_MotorConfiguration);
            m_pTalon->ClearStickyFaults();

            // Build the strings to use in the display method
            std::snprintf(&m_DisplayStrings.m_CurrentTemperatureString[0], DisplayStrings::MAX_MOTOR_DISPLAY_STRING_LENGTH, "%s #%u %s", m_pName, groupNumber, "temperature (F)");
            std::snprintf(&m_DisplayStrings.m_HighestTemperatureString[0], DisplayStrings::MAX_MOTOR_DISPLAY_STRING_LENGTH, "%s #%u %s", m_pName, groupNumber, "highest temperature (F)");
            std::snprintf(&m_DisplayStrings.m_ResetOccurredString[0], DisplayStrings::MAX_MOTOR_DISPLAY_STRING_LENGTH, "%s #%u %s", m_pName, groupNumber, "reset occurred");
        }

        // Helper routine for configuring some settings on follower talons
        void SetAsFollower(unsigned leaderCanId, bool bInvert)
        {
            // Follower will honor invert control, StrictFollower ignores invert control
            Follower follower(leaderCanId, bInvert);
            (void)m_pTalon->SetControl(follower);

            // Phoenix 6 Example: Get the StatusSignal objects and call SetUpdateFrequency() on them.
            //(void)m_pTalon->GetDeviceTemp().SetUpdateFrequency(100_Hz);
        }
    };

    static const unsigned MAX_NUMBER_OF_MOTORS = 4;
    static const unsigned GROUP_LEADER_CAN_ID = 0xFF;
    static const unsigned APPLY_TO_ALL_PSEUDO_CAN_ID = 0xFF;

    // Member variables
    unsigned m_NumMotors;                                   // Number of motors in the group
    unsigned m_LeaderCanId;                                 // Keep track of the CAN ID of the leader Talon in the group
    // @todo: No array, linked list?
    MotorInfo * m_pMotorsInfo[MAX_NUMBER_OF_MOTORS];        // The motor objects
    
    // Prevent default construction/deletion/copy/assignment
    TalonMotorGroup();
    ~TalonMotorGroup();
    TalonMotorGroup( const TalonMotorGroup& ) = delete;
    TalonMotorGroup & operator=( const TalonMotorGroup& ) = delete;
};



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::GetMotorObject
///
/// Retrieves a specific Talon motor object from the motor
/// group.  By default it will return the first motor object in
/// the group (the leader Talon).  If a CAN ID is specified, it
/// will retrieve that object instead.  This purpose of this
/// function is to allow robot code to make specific calls on a
/// motor object that may only apply to one motor in a group or
/// a specific motor type since this is a template class.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
TalonType * TalonMotorGroup<TalonType>::GetMotorObject(unsigned canId)
{
    TalonType * pTalonObject = nullptr;

    // By default, return the first object in the group
    if (canId == GROUP_LEADER_CAN_ID)
    {
        pTalonObject = m_pMotorsInfo[0]->m_pTalon;
    }
    // If a specific CAN ID was given
    else
    {
        // Loop through the motors
        for (unsigned i = 0U; i < m_NumMotors; i++)
        {
            // Check if this is the right motor
            if (m_pMotorsInfo[i]->m_CanId == canId)
            {
                pTalonObject = m_pMotorsInfo[i]->m_pTalon;
                break;
            }
        }
    }

    return pTalonObject;
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::GetMotorConfiguration
///
/// Retrieves a specific Talon motor configuration from the
/// motor group.  By default it will return the configuration of
/// the first motor in the group (the leader Talon).  If a CAN
/// ID is specified, it will retrieve that configuration instead.
/// The purpose of this function is to allow robot code to make
/// specific calls on a motor configuration that may only apply
/// to one motor in a group or for a specific configuration type
/// since this is a template class.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
TalonFXConfiguration * TalonMotorGroup<TalonType>::GetMotorConfiguration(unsigned canId)
{
    TalonFXConfiguration * pTalonConfig = nullptr;

    // By default, return the first object in the group
    if (canId == GROUP_LEADER_CAN_ID)
    {
        pTalonConfig = &(m_pMotorsInfo[0]->m_MotorConfiguration);
    }
    // If a specific CAN ID was given
    else
    {
        // Loop through the motors
        for (unsigned i = 0U; i < m_NumMotors; i++)
        {
            // Check if this is the right motor
            if (m_pMotorsInfo[i]->m_CanId == canId)
            {
                pTalonConfig = &(m_pMotorsInfo[i]->m_MotorConfiguration);
                break;
            }
        }
    }

    return pTalonConfig;
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::ApplyConfiguration
///
/// Applies an updated configuration to the talon motors in a
/// group.  Applying a configuration to only a specific motor in
/// the group should use the GetMotorConfiguration() function.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
void TalonMotorGroup<TalonType>::ApplyConfiguration(unsigned canId)
{
    // Loop through the motors.  This is slightly inefficient because
    // we can't break out of the loop if we find a match since we have
    // to consider the 'apply to all' case.
    for (unsigned i = 0U; i < m_NumMotors; i++)
    {
        // If the config is applied to all motors in the group, or we found the CAN ID
        if ((canId == APPLY_TO_ALL_PSEUDO_CAN_ID) || (m_pMotorsInfo[i]->m_CanId == canId))
        {
            // Apply the config
            (void)m_pMotorsInfo[i]->m_pTalon->GetConfigurator().Apply(m_pMotorsInfo[i]->m_MotorConfiguration);
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::ApplyConfiguration
///
/// Applies a specific configuration to the talon motors in a
/// group.  Applying a configuration to only a specific motor in
/// the group should use the GetMotorConfiguration() function.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
template <typename ConfigType>
void TalonMotorGroup<TalonType>::ApplyConfiguration(ConfigType config, unsigned canId)
{
    // For now, don't let this be used
    ASSERT(false);

    // Loop through the motors.  This is slightly inefficient because
    // we can't break out of the loop if we find a match since we have
    // to consider the 'apply to all' case.
    for (unsigned i = 0U; i < m_NumMotors; i++)
    {
        // If the config is applied to all motors in the group, or we found the CAN ID
        if ((canId == APPLY_TO_ALL_PSEUDO_CAN_ID) || (m_pMotorsInfo[i]->m_CanId == canId))
        {
            // This doesn't update the local configuration, which can't be done with the template
            //m_pMotorsInfo[i]->m_MotorConfiguration.<member> = config;
            (void)m_pMotorsInfo[i]->m_pTalon->GetConfigurator().Apply(config);
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::TalonMotorGroup
///
/// Constructor.  Creates the number of motors specified
/// starting from the CAN ID passed in.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
TalonMotorGroup<TalonType>::TalonMotorGroup(const char * pName, unsigned numMotors, unsigned leaderCanId,
                                            MotorGroupControlMode nonLeaderControlMode, NeutralModeValue neutralMode, bool bIsDriveMotor) :
    m_NumMotors(numMotors),
    m_LeaderCanId(leaderCanId)
{
    // Loop for each motor to create
    for (unsigned i = 0U; (i < numMotors) && (i < MAX_NUMBER_OF_MOTORS); i++)
    {
        // Group IDs are used in creating the strings and are not zero based
        unsigned groupId = i + 1U;

        // The leader Talon is unique
        if (i == 0U)
        {
            // Create it
            m_pMotorsInfo[i] = new MotorInfo(pName, EastTech::Talon::LEADER, neutralMode, leaderCanId, groupId, bIsDriveMotor);
        }
        // Non-leader Talons
        else
        {
            // Create it
            m_pMotorsInfo[i] = new MotorInfo(pName, nonLeaderControlMode, neutralMode, (leaderCanId + i), groupId, bIsDriveMotor);

            // Only set follow for Talon groups that will be configured as
            // such.  The CTRE Phoenix library now passes the control mode in
            // the Set() method, so we only need to set the followers here.
            if ((nonLeaderControlMode == EastTech::Talon::FOLLOW) || (nonLeaderControlMode == EastTech::Talon::FOLLOW_INVERSE))
            {
                bool bInvert = (nonLeaderControlMode == EastTech::Talon::FOLLOW) ? false : true;
                m_pMotorsInfo[i]->SetAsFollower(leaderCanId, bInvert);
            }
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::AddMotorToGroup
///
/// Method to add a new motor to a motor group.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
bool TalonMotorGroup<TalonType>::AddMotorToGroup(MotorGroupControlMode controlMode, bool bIsDriveMotor)
{
    bool bResult = false;

    // Make sure there's room for another motor in this group
    if (m_NumMotors < MAX_NUMBER_OF_MOTORS)
    {
        // The new motor CAN ID is the first motor's ID + current number of group motors present
        unsigned newMotorCanId = m_pMotorsInfo[0]->m_CanId + m_NumMotors;

        // m_NumMotors can be leveraged as the index, as it represents the next unused array element
        // All motors in a group have the same name, so we use the existing one.  Group ID is computed from m_NumMotors.
        m_pMotorsInfo[m_NumMotors] = new MotorInfo(m_pMotorsInfo[0]->m_pName, controlMode, newMotorCanId, (m_NumMotors + 1), bIsDriveMotor);
        
        // If this Talon will be a follower, be sure to call Set() to enable it
        if ((controlMode == EastTech::Talon::FOLLOW) || (controlMode == EastTech::Talon::FOLLOW_INVERSE))
        {
            bool bInvert = (controlMode == EastTech::Talon::FOLLOW) ? false : true;
            m_pMotorsInfo[m_NumMotors]->SetAsFollower(m_LeaderCanId, bInvert);
        }

        // Increase the number of motors
        m_NumMotors++;
        
        // Indicate success
        bResult = true;
    }

    return bResult;
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::SetMotorInGroupControlMode
///
/// Method to set the control mode of a motor in a group.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
bool TalonMotorGroup<TalonType>::SetMotorInGroupControlMode(unsigned canId, MotorGroupControlMode controlMode)
{
    bool bResult = false;
    
    // Search for the correct motor in the group
    for (unsigned i = 0U; i < m_NumMotors; i++)
    {
        // If it matches...
        if (m_pMotorsInfo[i]->m_CanId == canId)
        {
            // ...set the control mode
            m_pMotorsInfo[i]->m_ControlMode = controlMode;

            // If this Talon will be a follower, be sure to call Set() to enable it
            if ((controlMode == EastTech::Talon::FOLLOW) || (controlMode == EastTech::Talon::FOLLOW_INVERSE))
            {
                bool bInvert = (controlMode == EastTech::Talon::FOLLOW) ? false : true;
                m_pMotorsInfo[i]->SetAsFollower(m_LeaderCanId, bInvert);
            }
            else
            {
                // The previous mode might have had follower frame rates, so they need to be reset
                // Phoenix 6 Example: Get the StatusSignal objects and call SetUpdateFrequency() on them.
                //(void)m_pMotorsInfo[i]->m_pTalon->GetDeviceTemp().SetUpdateFrequency(100_Hz);
            }

            // Update the inverted status.  Only FOLLOW_INVERSE uses the built-in invert.
            if (controlMode == EastTech::Talon::FOLLOW_INVERSE)
            {
                m_pMotorsInfo[i]->m_MotorConfiguration.MotorOutput.WithInverted(true);
                m_pMotorsInfo[i]->m_pTalon->GetConfigurator().Apply(m_pMotorsInfo[i].m_MotorConfiguration.MotorOutput);
            }
            else
            {
                m_pMotorsInfo[i]->m_MotorConfiguration.MotorOutput.WithInverted(false);
                m_pMotorsInfo[i]->m_pTalon->GetConfigurator().Apply(m_pMotorsInfo[i].m_MotorConfiguration.MotorOutput);
            }
            
            // Indicate success
            bResult = true;
        }
    }

    return bResult;
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::SetCoastMode
///
/// Method to change a talon to coast mode.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
void TalonMotorGroup<TalonType>::SetCoastMode()
{
    for (unsigned i = 0U; i < m_NumMotors; i++)
    {
        m_pMotorsInfo[i]->m_MotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue::Coast;
        m_pMotorsInfo[i]->m_pTalon->GetConfigurator().Apply(m_pMotorsInfo[i]->m_MotorConfiguration.MotorOutput);
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::SetBrakeMode
///
/// Method to change a talon to brake mode.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
void TalonMotorGroup<TalonType>::SetBrakeMode()
{
    for (unsigned i = 0U; i < m_NumMotors; i++)
    {
        m_pMotorsInfo[i]->m_MotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue::Brake;
        m_pMotorsInfo[i]->m_pTalon->GetConfigurator().Apply(m_pMotorsInfo[i]->m_MotorConfiguration.MotorOutput);
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::Set
///
/// Method to set the speed of each motor in the group.  The
/// offset parameter is only valid for motor groups configured
/// as *_OFFSET.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
void TalonMotorGroup<TalonType>::Set(double value, double offset)
{
    for (unsigned i = 0U; i < m_NumMotors; i++)
    {
        // Setting motor values for groups assumes that the first half of
        // motors in a group should always get the same value, and the second
        // half of motors in a group could be different (such as inverse or offset).
        // Keep track of which segment of the motor group this motor is in.
        
        // Most modes wil need to call Set() later, but some won't
        bool bCallSet = true;
        
        // The value that will be passed to Set()
        double valueToSet = 0.0;
        
        // Check what the control mode of this motor is.  Most CAN Talons
        // will be set to follow, but some may be independent or inverse (such
        // as if they need to drive in different directions).
        switch (m_pMotorsInfo[i]->m_ControlMode)
        {
            case EastTech::Talon::LEADER:
            case EastTech::Talon::INDEPENDENT:
            {
                // The leader always gets set via duty cycle, as do motors
                // that are independently controlled (not follow or inverse).
                valueToSet = value;
                break;
            }
            case EastTech::Talon::FOLLOW:
            case EastTech::Talon::FOLLOW_INVERSE:
            {
                // Nothing to do, motor had SetControl() called during object construction
                bCallSet = false;
                break;
            }
            case EastTech::Talon::INVERSE:
            {
                // Motor is attached to drive in opposite direction of leader
                valueToSet = -value;
                break;
            }
            case EastTech::Talon::INDEPENDENT_OFFSET:
            {
                // The non-leader motor has a different value in this case
                valueToSet = value + offset;
                break;
            }
            case EastTech::Talon::INVERSE_OFFSET:
            {
                // The non-leader motor has a different value in this case
                valueToSet = -(value + offset);
                break;
            }
            default:
            {
                // Can reach here with CUSTOM motors still set.  Calling code should
                // update those motors to a different control mode via class API calls.
                break;
            }
        };
            
        if (bCallSet)
        {
            // Set the value in the Talon
            (void)m_pMotorsInfo[i]->m_pTalon->SetControl(m_pMotorsInfo[i]->m_DutyCycleOut.WithOutput(valueToSet));
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::SetAngle
///
/// Method to set the output of a motor to hold a specified
/// angle.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
void TalonMotorGroup<TalonType>::SetAngle(double angle)
{
    // The first entry in the motor info array should always
    // be a leader, so that one will be the only one updated.
    // When holding position, we don't want to have to manage
    // the target angles for multiple motors since it would
    // require tracking different set points based on the
    // encoders.  For a motor group to successfully hold
    // position, one of them needs to be a follower.
    // @todo: Check that this configuration is only applied to follower groups.

    // Set the control output
    units::angle::degree_t degrees(angle);
    units::angle::turn_t turns(degrees);
    (void)m_pMotorsInfo[0]->m_pTalon->SetControl(m_pMotorsInfo[0]->m_PositionVoltage.WithPosition(turns));
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::DisplayStatusInformation
///
/// Sends status information to the smart dashboard.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
void TalonMotorGroup<TalonType>::DisplayStatusInformation()
{
    for (unsigned i = 0U; i < m_NumMotors; i++)
    {
        m_pMotorsInfo[i]->m_CurrentTemperature = RobotUtils::ConvertCelsiusToFahrenheit(m_pMotorsInfo[i]->m_pTalon->GetDeviceTemp().GetValueAsDouble());
        if (m_pMotorsInfo[i]->m_CurrentTemperature > m_pMotorsInfo[i]->m_HighestTemperature)
        {
            m_pMotorsInfo[i]->m_HighestTemperature = m_pMotorsInfo[i]->m_CurrentTemperature;
        }

        // @todo: Also consider sticky faults?
        m_pMotorsInfo[i]->m_bResetOccurred = m_pMotorsInfo[i]->m_pTalon->HasResetOccurred();

        SmartDashboard::PutNumber(m_pMotorsInfo[i]->m_DisplayStrings.m_CurrentTemperatureString, m_pMotorsInfo[i]->m_CurrentTemperature);
        SmartDashboard::PutNumber(m_pMotorsInfo[i]->m_DisplayStrings.m_HighestTemperatureString, m_pMotorsInfo[i]->m_HighestTemperature);
        SmartDashboard::PutBoolean(m_pMotorsInfo[i]->m_DisplayStrings.m_ResetOccurredString, m_pMotorsInfo[i]->m_bResetOccurred);
    }
}

#endif // EASTTECHTALON_HPP
