////////////////////////////////////////////////////////////////////////////////
/// @file   EastTechRobot.cpp
/// @author David Stalter
///
/// @details
/// Implementation of the EastTechRobot class.  This file contains the functions
/// for full robot operation in FRC.  It contains the autonomous and operator
/// control routines as well as all necessary support for interacting with all
/// motors, sensors and input/outputs on the robot.
///
/// Copyright (c) 2026 East Technical High School
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
#include <cstddef>                      // for nullptr

// C INCLUDES
// (none)

// C++ INCLUDES
#include "EastTechRobot.hpp"            // for class declaration (and other headers)
#include "RobotUtils.hpp"               // for Trim(), Limit() and DisplayMessage()

// STATIC MEMBER VARIABLES
EastTechRobot * EastTechRobot::m_pThis;


////////////////////////////////////////////////////////////////
/// @method EastTechRobot::EastTechRobot
///
/// Constructor.  Instantiates all robot control objects.
///
////////////////////////////////////////////////////////////////
EastTechRobot::EastTechRobot() :
    m_AutonomousChooser                 (),
    m_pDriveController                  (new DriveControllerType(DRIVE_CONTROLLER_MODEL, DRIVE_JOYSTICK_PORT)),
    m_pAuxController                    (new AuxControllerType(AUX_CONTROLLER_MODEL, AUX_JOYSTICK_PORT)),
    m_RioCanBus                         (RIO_CAN_BUS_NAME),
    m_CanivoreBus                       (CANIVORE_CAN_BUS_NAME),
    m_pPigeon                           (new Pigeon2(PIGEON_CAN_ID, m_CanivoreBus)),
    m_pSwerveDrive                      (new SwerveDrive(m_pPigeon, m_GetCanBusReferenceLambda)),
    m_pLedController                    (new EastTechLedController(NUMBER_OF_LEDS, CANDLE_CAN_ID, m_RioCanBus, [this](){return m_AllianceColor.value();})),
    m_pDebugOutput                      (new DigitalOutput(DEBUG_OUTPUT_DIO_CHANNEL)),
    m_pCompressor                       (new Compressor(PneumaticsModuleType::CTREPCM)),
    m_pMatchModeTimer                   (new Timer()),
    m_pRobotProgramTimer                (new Timer()),
    m_pLimelightCamera                  (new LimelightCamera(LimelightCamera::LimelightModel::LIMELIGHT_3A, "limelight")),
    m_bLimelightFound                   (false),
    m_RobotMode                         (ROBOT_MODE_NOT_SET),
    m_AllianceColor                     (DriverStation::GetAlliance()),
    m_bRioPinsStable                    (false),
    m_bCameraAlignInProgress            (false),
    m_HeartBeat                         (0U)
{
    RobotUtils::DisplayMessage("Robot constructor.");

    // LiveWindow is not used
    LiveWindow::SetEnabled(false);

    // Signal logger is not used
    SignalLogger::EnableAutoLogging(false);

    // Set the autonomous options
    // @todo: Update these outside the constructor?
    m_AutonomousChooser.SetDefaultOption(AUTO_ROUTINE_1_STRING, AUTO_ROUTINE_1_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_2_STRING, AUTO_ROUTINE_2_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_3_STRING, AUTO_ROUTINE_3_STRING);
    m_AutonomousChooser.AddOption(AUTO_NO_ROUTINE_STRING, AUTO_NO_ROUTINE_STRING);
    m_AutonomousChooser.AddOption(AUTO_TEST_ROUTINE_STRING, AUTO_TEST_ROUTINE_STRING);
    SmartDashboard::PutData("Autonomous Modes", &m_AutonomousChooser);

    // Start the free running timer
    m_pRobotProgramTimer->Reset();
    m_pRobotProgramTimer->Start();
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::ResetMemberData
///
/// This method resets relevant member data variables.  Since
/// the robot object is only constructed once, it may be
/// necessary/helpful to return to a state similar to when the
/// constructor first ran (e.g. when enabling/disabling robot
/// states).  Only variables that need to be reset are modified
/// here.  This also works around the issue where non-member
/// static data cannot be easily reinitialized (since clearing
/// the .bss and running static constructors will only happen
/// once on program start up).
///
////////////////////////////////////////////////////////////////
void EastTechRobot::ResetMemberData()
{
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::RobotInit
///
/// This method is run when initializing the robot.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::RobotInit()
{
    RobotUtils::DisplayMessage("RobotInit called.");
    SetStaticThisInstance();

    // Attempt to locate the limelight.  The called function has a
    // search timeout.  If it isn't found, this will have to be
    // called again later.
    m_bLimelightFound = m_pLimelightCamera->FindAndSetNetworkTable();
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::RobotPeriodic
///
/// This method is run in all robot states.  It is called each
/// time a new packet is received from the driver station.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::RobotPeriodic()
{
    static bool bRobotPeriodicStarted = false;
    if (!bRobotPeriodicStarted)
    {
        RobotUtils::DisplayMessage("RobotPeriodic called.");
        bRobotPeriodicStarted = true;
    }

    // @todo: Read and display sensor values for calibration when not enabled
    // @note: From testing, smart dashboard prints of sensor values do give real time data.
    CheckIfRioPinsAreStable();
    UpdateSmartDashboard();
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::CheckIfRioPinsAreStable
///
/// Waits for any sensors on the robot that route to the RIO
/// to stabilize for accurate readings.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::CheckIfRioPinsAreStable()
{
    // This is the logic to wait to take PWM based sensor readings until the RIO is ready.
    // The behavior of the RIO is that it measures how many microseconds the signal is high
    // every second.  This requires waiting to get stable readings.
    // See https://github.com/wpilibsuite/allwpilib/issues/5284 for some related info.
    static units::time::second_t enabledTimeStamp = 0.0_s;
    units::time::second_t currentTimeStamp = m_pRobotProgramTimer->Get();
    if (DriverStation::IsEnabled() && (!m_bRioPinsStable))
    {
        // If the robot was just enabled (in any mode)
        if (enabledTimeStamp == 0.0_s)
        {
            // Set the start time stamp
            enabledTimeStamp = currentTimeStamp;
        }

        // Now check if enough time has passed for the RIO pins to have stabilized
        static constexpr const units::time::second_t RIO_DUTY_CYCLE_ENCODER_STARTUP_DELAY = 2.0_s;
        if ((currentTimeStamp - enabledTimeStamp) > RIO_DUTY_CYCLE_ENCODER_STARTUP_DELAY)
        {
            // Example encoder configuration algorithm

            //double encoderValue = m_pEncoder->Get();
            //units::angle::degree_t encoderValueDegrees(encoderValue * ANGLE_360_DEGREES);

            // This is the delta between the current mechanism position and the desired starting position (or zero point)
            //units::angle::degree_t startingOffsetDegrees = encoderValueDegrees - STARTING_POSITION_ENCODER_VALUE;
            //std::printf("startingOffsetDegrees (start): %f\n", startingOffsetDegrees.value());

            // If the starting offset is negative, we crossed over the absolute encoder boundary
            // We give a tolerance of five degrees in case the mechanism is near where we want to start
            // @todo: Does this need to check for very small readings below zero?
            // @todo: Boundary conditions here will be difficult
            //if (startingOffsetDegrees < ENCODER_BOUNDARY_TOLERANCE_DEGREES)
            //{
                // the 0/1 boundary is 360, so subtract the starting position to see how many degrees were up to that point
                // Add in the absolute value of the overage, which was negative
                //startingOffsetDegrees = (units::angle::degree_t(ANGLE_360_DEGREES) - STARTING_POSITION_ENCODER_VALUE) + encoderValueDegrees;
            //}

            // At this point we have the angle we want relative to zero
            //(void)m_pMotor->GetMotorObject()->GetConfigurator().SetPosition(startingOffsetDegrees);
            //std::printf("encoderValue: %f\n", encoderValue);
            //std::printf("encoderValueDegrees: %f\n", encoderValueDegrees.value());
            //std::printf("startingOffsetDegrees (final): %f\n", startingOffsetDegrees.value());

            m_bRioPinsStable = true;
        }
    }
    else
    {
        // Set the enabled time stamp back to zero until the robot is enabled again
        enabledTimeStamp = 0.0_s;

        // m_bRioPinsStable exists for the life of the program.  Once we have a stable
        // reading acquired, we don't need to do it again until the robot program restarts.
    }
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::ConfigureMotorControllers
///
/// Sets motor controller specific configuration information.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::ConfigureMotorControllers()
{
    // These are the defaults for the configuration (see TalonFX.h)
    //ctre::phoenix::sensors::AbsoluteSensorRange absoluteSensorRange = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
    //double integratedSensorOffsetDegrees = 0;
    //ctre::phoenix::sensors::SensorInitializationStrategy initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToZero;

    // The default constructor for TalonFXConfiguration will call the parent
    // BaseTalonConfiguration constructor with FeedbackDevice::IntegratedSensor.

    /*
    // @todo_phoenix6: Update the example for the new API.
    // Example configuration
    TalonFXConfiguration talonConfig;
    talonConfig.slot0.kP = 0.08;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = 0.3;
    talonConfig.slot0.kF = 0.0;
    talonConfig.absoluteSensorRange = AbsoluteSensorRange::Unsigned_0_to_360;
    talonConfig.integratedSensorOffsetDegrees = 0.0;
    talonConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;
    talonConfig.peakOutputForward = 1.0;
    talonConfig.peakOutputReverse = 1.0;
    talonConfig.slot0.closedLoopPeakOutput = 0.10;

    TalonFX * pTalon = new TalonFX(0xFF);
    pTalon->ConfigFactoryDefault();
    pTalon->ConfigAllSettings(talonConfig);
    pTalon->SetSelectedSensorPosition(0);
    const StatorCurrentLimitConfiguration INTAKE_MOTOR_STATOR_CURRENT_LIMIT_CONFIG = {true, 5.0, 50.0, 5.0};
    pTalon->ConfigStatorCurrentLimit(INTAKE_MOTOR_STATOR_CURRENT_LIMIT_CONFIG);
    */

    // Some notes about applying motor configurations:
    // - The classes/structs in EastTechTalon.hpp have motor configuration objects in them.
    // - Declaring stack local or class scope configuration objects are *separate and
    //   distinct* from the configuration objects in the EastTechTalon.hpp classes/structs.
    // - If a stack local or class scope configuration is applied, it will overwrite
    //   the configuration stored in the device.
    // - Calling the methods provided by EastTechTalon.hpp *never* update the configuration
    //   objects in the classes/structs.  To update those objects, retrieve the objects
    //   via things like GetMotorConfiguration().
    // - The classes/structs in EastTechTalon.hpp provide ApplyConfiguration() routines.
    //   These can be used to directly apply a stack local or class scope configuration,
    //   or to apply an updated configuration when the configuration objects were directly
    //   modified.  Keep the notes above in mind when calling them.
    // - The ApplyConfiguration() method for motor groups will default to applying the
    //   configuration to all motors unless a specific CAN ID is given.  The configuration
    //   applied to each motor in the group is the *saved configuration* for that specific
    //   motor.  It may be different for each motor, depending on the robot code.
    // - Configurations can be applied to the whole configuration object type, or to
    //   sub-types only (e.g. TalonFXConfiguration vs. CurrentLimitsConfigs), as
    //   ApplyConfiguration() is overloaded.  The template version only applies stack
    //   local or class scope configs, so remember the notes above.  Right now the
    //   template version is disabled, so don't call it.
    // - Thank CTRE for all this.  Instead of letting the config be a member of the motor
    //   object class with simple getter/setters, it's separate and overly complex.

    // Example configurations

    // Configure a motor group (only needs to be applied to the lead motor of the group)
    // Brake mode was set when the motor group was constructed
    //(void)m_pMotors->GetMotorConfiguration(MOTORS_CAN_START_ID)->Feedback.WithSensorToMechanismRatio(12.0 / 1.0);
    //(void)m_pMotors->GetMotorConfiguration(MOTORS_CAN_START_ID)->Slot0.WithKP(18.0).WithKI(0.0).WithKD(0.1);
    //m_pMotors->ApplyConfiguration(MOTORS_CAN_START_ID);
    //(void)m_pMotors->GetMotorObject(MOTORS_CAN_START_ID)->GetConfigurator().SetPosition(0.0_tr);

    // Configure a single motor
    //(void)m_pMotor->GetMotorConfiguration()->MotorOutput.WithNeutralMode(NeutralModeValue::Brake);
    //(void)m_pMotor->GetMotorConfiguration()->Feedback.WithSensorToMechanismRatio(135.0 / 1.0);
    //(void)m_pMotor->GetMotorConfiguration()->Slot0.WithKP(18.0).WithKI(0.0).WithKD(0.1);
    //(void)m_pMotor->GetMotorObject()->GetConfigurator().SetPosition(0.0_tr);
    //m_pMotor->ApplyConfiguration();
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::InitialStateSetup
///
/// This method contains the work flow for putting motors,
/// solenoids, etc. into a known state.  It is intended to be
/// used by both autonomous and user control.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::InitialStateSetup()
{
    // First reset any member data
    ResetMemberData();

    // Configure the motor controllers
    ConfigureMotorControllers();

    // Stop/clear any timers, just in case
    // @todo: Make this a dedicated function.
    m_pMatchModeTimer->Stop();
    m_pMatchModeTimer->Reset();

    // Just in case constructor was called before these were set (likely the case)
    m_AllianceColor = DriverStation::GetAlliance();

    // Set the LEDs to the alliance color
    m_pLedController->SetLedsToAllianceColor();

    // Set the limelight priority ID
    if (m_bLimelightFound)
    {
        m_pLimelightCamera->SetPriorityId(LimelightCamera::TaggedFieldElement::ELEMENT_NONE, m_AllianceColor.value());
    }

    // Clear the debug output pin
    m_pDebugOutput->Set(false);

    // Reset the heartbeat
    m_HeartBeat = 0U;

    // Point the swerve modules straight
    m_pSwerveDrive->HomeModules();
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::TeleopInit
///
/// The teleop init method.  This method is called once each
/// time the robot enters teleop control.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::TeleopInit()
{
    RobotUtils::DisplayMessage("TeleopInit called.");

    // Autonomous should have left things in a known state, but just in case, clear everything.
    CommandScheduler::GetInstance().CancelAll();
    InitialStateSetup();

    // Start the mode timer for teleop
    m_pMatchModeTimer->Start();
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::TeleopPeriodic
///
/// The teleop control method.  This method is called
/// periodically while the robot is in teleop control.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::TeleopPeriodic()
{
    // Log a mode change if one occurred
    CheckAndUpdateRobotMode(ROBOT_MODE_TELEOP);

    HeartBeat();

    if (!m_bCameraAlignInProgress)
    {
        SwerveDriveSequence();
    }

    //PneumaticSequence();
    
    CameraSequence();
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::UpdateSmartDashboard
///
/// Updates values in the smart dashboard.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::UpdateSmartDashboard()
{
    // @todo: Check if RobotPeriodic() is called every 20ms and use static counter.

    units::time::second_t matchTime = 0.0_s;
    double batteryVoltage = DriverStation::GetBatteryVoltage();

    if (DriverStation::IsFMSAttached())
    {
        matchTime = DriverStation::GetMatchTime();
    }
    else
    {
        matchTime = m_pMatchModeTimer->Get();
    }

    // Give the drive team some state information
    SmartDashboard::PutBoolean("RIO pins stable", m_bRioPinsStable);
    SmartDashboard::PutBoolean("Limelight found", m_bLimelightFound);
    SmartDashboard::PutNumber("Battery voltage", batteryVoltage);
    SmartDashboard::PutNumber("Match time", matchTime.value());
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::PneumaticSequence
///
/// This method contains the main workflow for updating the
/// state of the pnemuatics on the robot.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::PneumaticSequence()
{
    // @todo: Monitor other compressor API data?
    SmartDashboard::PutBoolean("Compressor status", m_pCompressor->IsEnabled());
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::CameraSequence
///
/// This method handles camera related behavior.  See the
/// camera classes for full details.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::CameraSequence()
{
    // If the limelight wasn't found, don't attempt to do anything
    if (!m_bLimelightFound)
    {
        return;
    }

    if (m_pDriveController->GetButtonState(DRIVE_ALIGN_WITH_CAMERA_BUTTON))
    {
        m_bCameraAlignInProgress = true;
        m_pLimelightCamera->AlignToTargetSwerve(m_LimelightDriveLambda, m_pPigeon->GetYaw().GetValue());
    }
    else
    {
        m_bCameraAlignInProgress = false;
    }

    m_pLimelightCamera->UpdateSmartDashboard();
    SmartDashboard::PutBoolean("Limelight align", m_bCameraAlignInProgress);
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::SwerveDriveSequence
///
/// This method contains the main workflow for swerve drive
/// control.  It will gather input from the drive joystick and
/// then filter those values to ensure they are past a certain
/// threshold (deadband) and generate the information to pass
/// on to the swerve drive system.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::SwerveDriveSequence()
{
    // Check for a switch between field relative and robot centric
    static bool bFieldRelative = true;
    if (m_pDriveController->DetectButtonChange(FIELD_RELATIVE_TOGGLE_BUTTON))
    {
        bFieldRelative = !bFieldRelative;
    }

    if (m_pDriveController->DetectButtonChange(REZERO_SWERVE_BUTTON))
    {
        m_pSwerveDrive->ZeroGyroYaw();
        m_pSwerveDrive->RecalibrateModules();
        m_pSwerveDrive->HomeModules();
    }

    if (m_pDriveController->DetectButtonChange(LOCK_SWERVE_WHEELS_BUTTON))
    {
        m_pSwerveDrive->LockWheels();
    }

    // The GetDriveX() and GetDriveY() functions refer to ***controller joystick***
    // x and y axes.  Multiply by -1.0 here to keep the joystick input retrieval code common.
    double translationAxis = RobotUtils::Trim(m_pDriveController->GetDriveYInput() * -1.0, DRIVE_TRIM_UPPER_LIMIT, DRIVE_TRIM_LOWER_LIMIT);
    double strafeAxis = RobotUtils::Trim(m_pDriveController->GetDriveXInput() * -1.0, DRIVE_TRIM_UPPER_LIMIT, DRIVE_TRIM_LOWER_LIMIT);
    double rotationAxis = RobotUtils::Trim(m_pDriveController->GetDriveRotateInput() * -1.0, DRIVE_TRIM_UPPER_LIMIT, DRIVE_TRIM_LOWER_LIMIT);

    // Override normal control if a fine positioning request is made
    switch (m_pDriveController->GetPovAsDirection())
    {
        case DRIVE_CONTROLS_SWERVE_FORWARD_SLOW_POV:
        {
            translationAxis = SWERVE_DRIVE_SLOW_SPEED;
            strafeAxis = 0.0;
            rotationAxis = 0.0;
            break;
        }
        case DRIVE_CONTROLS_SWERVE_REVERSE_SLOW_POV:
        {
            translationAxis = -SWERVE_DRIVE_SLOW_SPEED;
            strafeAxis = 0.0;
            rotationAxis = 0.0;
            break;
        }
        case DRIVE_CONTROLS_SWERVE_LEFT_OR_CCW_SLOW_POV:
        {
            // Left/right POV control can either toggle strafe or rotation
            translationAxis = 0.0;
            strafeAxis = (EastTech::Drive::Config::SWERVE_SLOW_USE_ROTATION_AXIS) ? (0.0) : (SWERVE_DRIVE_SLOW_SPEED);
            rotationAxis = (EastTech::Drive::Config::SWERVE_SLOW_USE_ROTATION_AXIS) ? (SWERVE_ROTATE_SLOW_SPEED) : (0.0);
            break;
        }
        case DRIVE_CONTROLS_SWERVE_RIGHT_OR_CW_SLOW_POV:
        {
            // Left/right POV control can either toggle strafe or rotation
            translationAxis = 0.0;
            strafeAxis = (EastTech::Drive::Config::SWERVE_SLOW_USE_ROTATION_AXIS) ? (0.0) : (-SWERVE_DRIVE_SLOW_SPEED);
            rotationAxis = (EastTech::Drive::Config::SWERVE_SLOW_USE_ROTATION_AXIS) ? (-SWERVE_ROTATE_SLOW_SPEED) : (0.0);
            break;
        }
        default:
        {
            break;
        }
    }

    SmartDashboard::PutNumber("Strafe", strafeAxis);
    SmartDashboard::PutNumber("Translation", translationAxis);
    SmartDashboard::PutNumber("Rotation", rotationAxis);
    SmartDashboard::PutBoolean("Field Relative", bFieldRelative);

    // Notice that this is sending translation to X and strafe to Y, despite
    // the inputs coming from the opposite of what may be intuitive (strafe as X,
    // translation as Y).  See the comment in Translation2d.h about the robot
    // placed at origin facing the X-axis.  Forward movement increases X and left
    // movement increases Y.
    Translation2d translation = {units::meter_t(translationAxis), units::meter_t(strafeAxis)};

    // Update the swerve module states
    m_pSwerveDrive->SetModuleStates(translation, rotationAxis, bFieldRelative, true);

    // Update the odometry
    m_pSwerveDrive->UpdateOdometry();

    // Display some useful information
    m_pSwerveDrive->UpdateSmartDashboard();
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::DisabledInit
///
/// The disabled init method.  This method is called once each
/// time the robot enters disabled mode.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::DisabledInit()
{
    RobotUtils::DisplayMessage("DisabledInit called.");

    // The check against teleop robot mode is because it represents
    // the last active mode before DisabledInit() was called.  Only
    // save a capture in matches and from teleop.
    constexpr units::time::second_t TELEOP_REWIND_TIME_S = 160.0_s;
    if (DriverStation::IsFMSAttached() && (m_RobotMode == RobotMode::ROBOT_MODE_TELEOP) && m_bLimelightFound)
    {
        m_pLimelightCamera->TriggerRewindCapture(TELEOP_REWIND_TIME_S);
    }

    // Turn the rainbow animation back on
    m_pLedController->SetAnimation(EastTechLedController::LedAnimation::LED_RAINBOW_ANIMATION);
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::DisabledPeriodic
///
/// The disabled control method.  This method is called
/// periodically while the robot is disabled.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::DisabledPeriodic()
{
    // Log a mode change if one occurred
    CheckAndUpdateRobotMode(ROBOT_MODE_DISABLED);
}



////////////////////////////////////////////////////////////////
/// @method main
///
/// Execution start for the robt.
///
////////////////////////////////////////////////////////////////
#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<EastTechRobot>();
}
#endif
