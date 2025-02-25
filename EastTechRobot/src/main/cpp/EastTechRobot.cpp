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
/// Copyright (c) 2024 East Technical High School
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
#include <cstddef>                      // for nullptr
#include <cstring>                      // for memset

// C INCLUDES
// (none)

// C++ INCLUDES
#include "EastTechRobot.hpp"            // for class declaration (and other headers)
#include "RobotCamera.hpp"              // for interacting with cameras
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
    m_pPigeon                           (new Pigeon2(PIGEON_CAN_ID, "canivore-8145")),
    m_pSwerveDrive                      (new SwerveDrive(m_pPigeon)),
    m_pDebugOutput                      (new DigitalOutput(DEBUG_OUTPUT_DIO_CHANNEL)),
    m_pCompressor                       (new Compressor(PneumaticsModuleType::CTREPCM)),
    m_pMatchModeTimer                   (new Timer()),
    m_pSafetyTimer                      (new Timer()),
    m_CameraThread                      (RobotCamera::LimelightThread),
    m_RobotMode                         (ROBOT_MODE_NOT_SET),
    m_AllianceColor                     (DriverStation::GetAlliance()),
    m_bCameraAlignInProgress            (false),
    m_HeartBeat                         (0U)
{
    RobotUtils::DisplayMessage("Robot constructor.");
    
    // LiveWindow is not used
    LiveWindow::SetEnabled(false);
    
    // Set the autonomous options
    m_AutonomousChooser.SetDefaultOption(AUTO_ROUTINE_1_STRING, AUTO_ROUTINE_1_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_2_STRING, AUTO_ROUTINE_2_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_3_STRING, AUTO_ROUTINE_3_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_3_STRING, AUTO_ROUTINE_3_STRING);
    m_AutonomousChooser.AddOption(AUTO_NO_ROUTINE_STRING, AUTO_NO_ROUTINE_STRING);
    m_AutonomousChooser.AddOption(AUTO_TEST_ROUTINE_STRING, AUTO_TEST_ROUTINE_STRING);
    SmartDashboard::PutData("Autonomous Modes", &m_AutonomousChooser);
    
    RobotUtils::DisplayFormattedMessage("The drive forward axis is: %d\n", EastTech::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.RIGHT_TRIGGER);
    RobotUtils::DisplayFormattedMessage("The drive reverse axis is: %d\n", EastTech::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.LEFT_TRIGGER);
    RobotUtils::DisplayFormattedMessage("The drive left/right axis is: %d\n", EastTech::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.LEFT_X_AXIS);

    ConfigureMotorControllers();

    // Spawn the vision thread
    RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::DRIVER_CAMERA);
    RobotCamera::SetLimelightLedMode(RobotCamera::LimelightLedMode::PIPELINE);
    m_CameraThread.detach();
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

    // Stop/clear any timers, just in case
    // @todo: Make this a dedicated function.
    m_pMatchModeTimer->Stop();
    m_pMatchModeTimer->Reset();
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
    
    // Just in case constructor was called before these were set (likely the case)
    m_AllianceColor = DriverStation::GetAlliance();

    // Indicate the camera thread can continue
    RobotCamera::ReleaseThread();

    // Clear the debug output pin
    m_pDebugOutput->Set(false);

    // Reset the heartbeat
    m_HeartBeat = 0U;

    // Set the swerve modules to a known angle.  This addresses an
    // issue with the Neos where setting position during constructors
    // doesn't take effect.
    #ifdef USE_NEO_SWERVE
    // @todo: Check this on TalonFX
    m_pSwerveDrive->HomeModules();
    #endif
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
    
    // Autonomous should have left things in a known state, but
    // just in case clear everything.
    InitialStateSetup();

    // Tele-op won't do detailed processing of the images unless instructed to
    RobotCamera::SetFullProcessing(false);
    RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::DRIVER_CAMERA);
    RobotCamera::SetLimelightLedMode(RobotCamera::LimelightLedMode::PIPELINE);

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

    if (EastTech::Drive::Config::USE_SWERVE_DRIVE)
    {
        SwerveDriveSequence();
    }

    //PneumaticSequence();
    
    CameraSequence();

    UpdateSmartDashboard();
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
    // Give the drive team some state information
    // Nothing to send yet
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
/// RobotCamera class for full details.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::CameraSequence()
{
    if (m_pDriveController->GetButtonState(DRIVE_ALIGN_WITH_CAMERA_BUTTON))
    {
        m_bCameraAlignInProgress = true;
        RobotCamera::SetLimelightPipeline(1);
        RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::VISION_PROCESSOR);
        RobotCamera::AutonomousCamera::AlignToTargetSwerve();
    }
    else
    {
        m_bCameraAlignInProgress = false;
        RobotCamera::SetLimelightPipeline(0);
        RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::DRIVER_CAMERA);
    }
    return;

    // 2024: Go no further

    static bool bFullProcessing = false;
    
    // @note: Use std::chrono if precise time control is needed.
    
    // Check for any change in camera
    if (m_pDriveController->GetButtonState(SELECT_FRONT_CAMERA_BUTTON))
    {
        RobotCamera::SetCamera(RobotCamera::FRONT_USB);
    }
    else if (m_pDriveController->GetButtonState(SELECT_BACK_CAMERA_BUTTON))
    {
        RobotCamera::SetCamera(RobotCamera::BACK_USB);
    }
    else
    {
    }
    
    // Look for full processing to be enabled/disabled
    if (m_pDriveController->DetectButtonChange(CAMERA_TOGGLE_FULL_PROCESSING_BUTTON))
    {
        // Change state first, because the default is set before this code runs
        bFullProcessing = !bFullProcessing;
        RobotCamera::SetFullProcessing(bFullProcessing);
    }
    
    // Look for the displayed processed image to be changed
    if (m_pDriveController->DetectButtonChange(CAMERA_TOGGLE_PROCESSED_IMAGE_BUTTON))
    {
        RobotCamera::ToggleCameraProcessedImage();
    }
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
        m_pSwerveDrive->HomeModules();
    }

    // The GetDriveX() and GetDriveYInput() functions refer to ***controller joystick***
    // x and y axes.  Multiply by -1.0 here to keep the joystick input retrieval code common.
    double translationAxis = RobotUtils::Trim(m_pDriveController->GetDriveYInput() * -1.0, JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
    double strafeAxis = RobotUtils::Trim(m_pDriveController->GetDriveXInput() * -1.0, JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
    double rotationAxis = RobotUtils::Trim(m_pDriveController->GetDriveRotateInput() * -1.0, JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);

    // Override normal control if a fine positioning request is made
    switch (m_pDriveController->GetPovAsDirection())
    {
        case EastTech::Controller::PovDirections::POV_UP:
        {
            translationAxis = SWERVE_DRIVE_SLOW_SPEED;
            strafeAxis = 0.0;
            rotationAxis = 0.0;
            break;
        }
        case EastTech::Controller::PovDirections::POV_DOWN:
        {
            translationAxis = -SWERVE_DRIVE_SLOW_SPEED;
            strafeAxis = 0.0;
            rotationAxis = 0.0;
            break;
        }
        case EastTech::Controller::PovDirections::POV_LEFT:
        {
            translationAxis = 0.0;
            strafeAxis = 0.0;
            rotationAxis = SWERVE_ROTATE_SLOW_SPEED;
            break;
        }
        case EastTech::Controller::PovDirections::POV_RIGHT:
        {
            translationAxis = 0.0;
            strafeAxis = 0.0;
            rotationAxis = -SWERVE_ROTATE_SLOW_SPEED;
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

    // @todo: Shut off the limelight LEDs?
    RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::DRIVER_CAMERA);
    RobotCamera::SetLimelightLedMode(RobotCamera::LimelightLedMode::PIPELINE);
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
