////////////////////////////////////////////////////////////////////////////////
/// @file   TalonMotorGroup.hpp
/// @author David Stalter
///
/// @details
/// A class designed to work with a group of CAN Talon speed controllers working
/// in tandem.
///
/// Copyright (c) 2024 Youth Technology Academy
////////////////////////////////////////////////////////////////////////////////

#ifndef TALONMOTORGROUP_HPP
#define TALONMOTORGROUP_HPP

// CTRE output is noisy this year, making it impossible to find real errors
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

// SYSTEM INCLUDES
#include <cstdio>                               // for std::snprintf

// C INCLUDES
#include "ctre/Phoenix.h"                       // for CTRE library interfaces
#include "frc/smartdashboard/SmartDashboard.h"  // for interacting with the smart dashboard

// C++ INCLUDES
#include "RobotUtils.hpp"                       // for ConvertCelsiusToFahrenheit

using namespace frc;


////////////////////////////////////////////////////////////////
/// @namespace EastTechTalon
///
/// Namespace that contains declarations for interacting with
/// Talon speed controllers specific to East Tech.
///
////////////////////////////////////////////////////////////////
namespace EastTechTalon
{
    // Represents how a motor will be controlled
    enum MotorGroupControlMode
    {
        MASTER,                 // First motor in a group
        FOLLOW,                 // Motor follows the master
        FOLLOW_INVERSE,         // Motor follows the master, but inverse
        INDEPENDENT,            // Motor needs to be set independently
        INVERSE,                // Motor is the inverse value of the master
        INDEPENDENT_OFFSET,     // Motor is set independently, but with a different value from master
        INVERSE_OFFSET,         // Motor is set independently, but with the a different inverse value from master
        CUSTOM                  // Motor needs to be set later to an option above
    };

    static const bool CURRENT_LIMITING_ENABLED = false;
}



////////////////////////////////////////////////////////////////
/// @class TalonMotorGroup
///
/// Class that provides methods for interacting with a group of
/// Talon speed controllers.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
class TalonMotorGroup
{
public:

    typedef EastTechTalon::MotorGroupControlMode MotorGroupControlMode;
    
    // Constructor
    TalonMotorGroup(
                     const char * pName,
                     unsigned numMotors,
                     unsigned masterCanId,
                     MotorGroupControlMode nonMasterControlMode,
                     NeutralMode neutralMode,
                     FeedbackDevice sensor = FeedbackDevice::None,
                     bool bIsDriveMotor = false
                   );

    // Retrieve a specific motor object
    TalonType * GetMotorObject(unsigned canId = GROUP_MASTER_CAN_ID);

    // Adds a new motor to a group
    bool AddMotorToGroup(MotorGroupControlMode controlMode, bool bIsDriveMotor = false);
    
    // Function to set the speed of each motor in the group
    void Set(double value, double offset = 0.0);
    
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
        const char * m_pName;
        MotorGroupControlMode m_ControlMode;
        unsigned m_CanId;
        double m_CurrentTemperature;
        double m_HighestTemperature;
        bool m_bResetOccurred;
        bool m_bIsDriveMotor;
        DisplayStrings m_DisplayStrings;
        
        MotorInfo(const char * pName, MotorGroupControlMode controlMode, NeutralMode neutralMode, unsigned canId, unsigned groupNumber, bool bIsDriveMotor = false) :
            m_pTalon(new TalonType(static_cast<int>(canId))),
            m_pName(pName),
            m_ControlMode(controlMode),
            m_CanId(canId),
            m_CurrentTemperature(0.0),
            m_HighestTemperature(0.0),
            m_bResetOccurred(false),
            m_bIsDriveMotor(bIsDriveMotor)
        {
            m_pTalon->SetNeutralMode(neutralMode);

            if (controlMode == EastTechTalon::FOLLOW_INVERSE)
            {
                m_pTalon->SetInverted(true);
            }

            // @todo: Move in sensor too?
            if (EastTechTalon::CURRENT_LIMITING_ENABLED && bIsDriveMotor)
            {
                // Limits were 40.0, 55.0, 0.1
                const StatorCurrentLimitConfiguration DRIVE_MOTOR_STATOR_CURRENT_LIMIT_CONFIG = {true, 55.0, 60.0, 0.1};
                m_pTalon->ConfigStatorCurrentLimit(DRIVE_MOTOR_STATOR_CURRENT_LIMIT_CONFIG);
            }

            // Build the strings to use in the display method
            std::snprintf(&m_DisplayStrings.m_CurrentTemperatureString[0], DisplayStrings::MAX_MOTOR_DISPLAY_STRING_LENGTH, "%s #%u %s", m_pName, groupNumber, "temperature (F)");
            std::snprintf(&m_DisplayStrings.m_HighestTemperatureString[0], DisplayStrings::MAX_MOTOR_DISPLAY_STRING_LENGTH, "%s #%u %s", m_pName, groupNumber, "highest temperature (F)");
            std::snprintf(&m_DisplayStrings.m_ResetOccurredString[0], DisplayStrings::MAX_MOTOR_DISPLAY_STRING_LENGTH, "%s #%u %s", m_pName, groupNumber, "reset occurred");
        }

        // Helper routine for configuring some settings on follower talons
        void SetAsFollower(unsigned masterCanId)
        {
            static const uint8_t FOLLOWER_FRAME_RATE_MS = 100U;

            // Set it as a follower
            m_pTalon->Set(ControlMode::Follower, masterCanId);

            // The Phoenix documentation states: "Motor controllers that are followers can have slower
            // update rates for [status groups 1/2] without impacting performance."
            // @todo: Consider setting other rates, even for all motor controllers
            m_pTalon->SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, FOLLOWER_FRAME_RATE_MS);
            m_pTalon->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, FOLLOWER_FRAME_RATE_MS);
        }
    };

    static const unsigned MAX_NUMBER_OF_MOTORS = 4;
    static const unsigned GROUP_MASTER_CAN_ID = 0xFF;

    // Member variables
    unsigned m_NumMotors;                                   // Number of motors in the group
    unsigned m_MasterCanId;                                 // Keep track of the CAN ID of the master Talon in the group
    FeedbackDevice m_Sensor;                                // Keep track of the sensor attached to the Talon (assumes one sensor per group)
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
/// the group (the master Talon).  If a CAN ID is specified, it
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
    if (canId == GROUP_MASTER_CAN_ID)
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
/// @method TalonMotorGroup::TalonMotorGroup
///
/// Constructor.  Creates the number of motors specified
/// starting from the CAN ID passed in.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
TalonMotorGroup<TalonType>::TalonMotorGroup(const char * pName, unsigned numMotors, unsigned masterCanId,
                                            MotorGroupControlMode nonMasterControlMode, NeutralMode neutralMode, FeedbackDevice sensor, bool bIsDriveMotor) :
    m_NumMotors(numMotors),
    m_MasterCanId(masterCanId),
    m_Sensor(sensor)
{
    // Loop for each motor to create
    for (unsigned i = 0U; (i < numMotors) && (i < MAX_NUMBER_OF_MOTORS); i++)
    {
        // Group IDs are used in creating the strings and are not zero based
        unsigned groupId = i + 1U;

        // The master Talon is unique
        if (i == 0U)
        {
            // Create it
            m_pMotorsInfo[i] = new MotorInfo(pName, EastTechTalon::MASTER, neutralMode, masterCanId, groupId, bIsDriveMotor);
            
            // This assumes only the first controller in a group has a sensor
            if (sensor != FeedbackDevice::None)
            {
                // Sensor initialization (feedbackDevice, pidIdx, timeoutMs)
                m_pMotorsInfo[0]->m_pTalon->ConfigSelectedFeedbackSensor(sensor, 0, 0);
            }
        }
        // Non-master Talons
        else
        {
            // Create it
            m_pMotorsInfo[i] = new MotorInfo(pName, nonMasterControlMode, neutralMode, (masterCanId + i), groupId, bIsDriveMotor);

            // Only set follow for Talon groups that will be configured as
            // such.  The CTRE Phoenix library now passes the control mode in
            // the Set() method, so we only need to set the followers here.
            if ((nonMasterControlMode == EastTechTalon::FOLLOW) || (nonMasterControlMode == EastTechTalon::FOLLOW_INVERSE))
            {
                m_pMotorsInfo[i]->SetAsFollower(masterCanId);
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
        if ((controlMode == EastTechTalon::FOLLOW) || (controlMode == EastTechTalon::FOLLOW_INVERSE))
        {
            m_pMotorsInfo[m_NumMotors]->SetAsFollower(m_MasterCanId);
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
            if ((controlMode == EastTechTalon::FOLLOW) || (controlMode == EastTechTalon::FOLLOW_INVERSE))
            {
                m_pMotorsInfo[i]->SetAsFollower(m_MasterCanId);
            }
            else
            {
                // The previous mode might have had follower frame rates, so they need to be reset
                static const uint8_t DEFAULT_STATUS_GROUP_1_FRAME_RATE_MS = 10U;
                static const uint8_t DEFAULT_STATUS_GROUP_2_FRAME_RATE_MS = 20U;
                m_pMotorsInfo[i]->m_pTalon->SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, DEFAULT_STATUS_GROUP_1_FRAME_RATE_MS);
                m_pMotorsInfo[i]->m_pTalon->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, DEFAULT_STATUS_GROUP_2_FRAME_RATE_MS);
            }

            // Update the inverted status.  Only FOLLOW_INVERSE uses the built-in invert.
            if (controlMode == EastTechTalon::FOLLOW_INVERSE)
            {
                m_pMotorsInfo[i]->m_pTalon->SetInverted(true);
            }
            else
            {
                m_pMotorsInfo[i]->m_pTalon->SetInverted(false);
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
        m_pMotorsInfo[i]->m_pTalon->SetNeutralMode(NeutralMode::Coast);
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
        m_pMotorsInfo[i]->m_pTalon->SetNeutralMode(NeutralMode::Brake);
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::TareEncoder
///
/// Method to tare the value on an encoder feedback device
/// connected to a Talon controller.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
void TalonMotorGroup<TalonType>::TareEncoder()
{
    if (m_Sensor == FeedbackDevice::CTRE_MagEncoder_Relative)
    {
        // sensorPos, pidIdx, timeoutMs
        m_pMotorsInfo[0]->m_pTalon->SetSelectedSensorPosition(0, 0, 0);
    }
}



////////////////////////////////////////////////////////////////
/// @method TalonMotorGroup::GetEncoderValue
///
/// Method to get the value from an encoder feedback device
/// connected to a Talon controller.
///
////////////////////////////////////////////////////////////////
template <class TalonType>
int TalonMotorGroup<TalonType>::GetEncoderValue()
{
    int sensorValue = 0;

    if (m_Sensor == FeedbackDevice::CTRE_MagEncoder_Relative)
    {
        // pidIdx
        sensorValue = m_pMotorsInfo[0]->m_pTalon->GetSelectedSensorPosition(0);
    }
    
    return sensorValue;
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
            case EastTechTalon::MASTER:
            case EastTechTalon::INDEPENDENT:
            {
                // The master always gets set via percent voltage, as do
                // motors that are independently controlled (not follow or inverse).
                valueToSet = value;
                break;
            }
            case EastTechTalon::FOLLOW:
            case EastTechTalon::FOLLOW_INVERSE:
            {
                // Nothing to do, motor had Set() called during object construction
                bCallSet = false;
                break;
            }
            case EastTechTalon::INVERSE:
            {
                // Motor is attached to drive in opposite direction of master
                valueToSet = -value;
                break;
            }
            case EastTechTalon::INDEPENDENT_OFFSET:
            {
                // The non-master motor has a different value in this case
                valueToSet = value + offset;
                break;
            }
            case EastTechTalon::INVERSE_OFFSET:
            {
                // The non-master motor has a different value in this case
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
            m_pMotorsInfo[i]->m_pTalon->Set(ControlMode::PercentOutput, valueToSet);
        }
    }
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
        m_pMotorsInfo[i]->m_CurrentTemperature = RobotUtils::ConvertCelsiusToFahrenheit(m_pMotorsInfo[i]->m_pTalon->GetTemperature());
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

#endif // TALONMOTORGROUP_HPP
