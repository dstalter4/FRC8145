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
    m_pIntakeMotor                      (new TalonFxMotorController(INTAKE_MOTOR_CAN_ID)),
    m_pFeederMotor                      (new TalonFxMotorController(FEEDER_MOTOR_CAN_ID)),
    m_pShooterMotors                    (new TalonMotorGroup<TalonFX>("Shooter", TWO_MOTORS, SHOOTER_MOTORS_CAN_START_ID, MotorGroupControlMode::INVERSE_OFFSET, NeutralModeValue::Coast, false)),
    m_pPivotMotors                      (new TalonMotorGroup<TalonFX>("Pivot", TWO_MOTORS, PIVOT_MOTORS_CAN_START_ID, MotorGroupControlMode::FOLLOW_INVERSE, NeutralModeValue::Brake, false)),
    m_pLiftMotors                       (new EastTech::Talon::EmptyTalonFx("Lift", TWO_MOTORS, LIFT_MOTORS_CAN_START_ID, MotorGroupControlMode::INVERSE_OFFSET, NeutralModeValue::Brake, false)),
    m_pBlinkin                          (new Spark(BLINKIN_PWM_CHANNEL)),
    m_pDebugOutput                      (new DigitalOutput(DEBUG_OUTPUT_DIO_CHANNEL)),
    m_pCompressor                       (new Compressor(PneumaticsModuleType::CTREPCM)),
    m_pMatchModeTimer                   (new Timer()),
    m_pSafetyTimer                      (new Timer()),
    m_CameraThread                      (RobotCamera::LimelightThread),
    m_RobotMode                         (ROBOT_MODE_NOT_SET),
    m_AllianceColor                     (DriverStation::GetAlliance()),
    m_bCameraAlignInProgress            (false),
    m_bShootSpeaker                     (true),
    m_bShootSpeakerClose                (true),
    m_bShotInProgress                   (false),
    m_bIntakeInProgress                 (false),
    m_bPivotTareInProgress              (false),
    m_PivotTargetDegrees                (0.0_deg),
    m_AmpTargetDegrees                  (PIVOT_ANGLE_TOUCHING_AMP),
    m_AmpTargetSpeed                    (SHOOTER_MOTOR_AMP_SPEED),
    m_HeartBeat                         (0U)
{
    RobotUtils::DisplayMessage("Robot constructor.");
    
    // LiveWindow is not used
    LiveWindow::SetEnabled(false);
    
    // Set the autonomous options
    m_AutonomousChooser.SetDefaultOption(AUTO_ROUTINE_1_STRING, AUTO_ROUTINE_1_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_2_STRING, AUTO_ROUTINE_2_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_3_STRING, AUTO_ROUTINE_3_STRING);
    m_AutonomousChooser.AddOption(AUTO_TEST_ROUTINE_STRING, AUTO_TEST_ROUTINE_STRING);
    SmartDashboard::PutData("Autonomous Modes", &m_AutonomousChooser);
    
    RobotUtils::DisplayFormattedMessage("The drive forward axis is: %d\n", EastTech::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.RIGHT_TRIGGER);
    RobotUtils::DisplayFormattedMessage("The drive reverse axis is: %d\n", EastTech::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.LEFT_TRIGGER);
    RobotUtils::DisplayFormattedMessage("The drive left/right axis is: %d\n", EastTech::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.LEFT_X_AXIS);

    ConfigureMotorControllers();

    // Enable an LED display pattern (Ocean Palette Rainbow)
    m_pBlinkin->Set(-0.95);

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

    // Configure mechanism pivot motor controller
    TalonFXConfiguration pivotTalonConfig;
    pivotTalonConfig.Feedback.SensorToMechanismRatio = 25.0;
    pivotTalonConfig.ClosedLoopGeneral.ContinuousWrap = true;

    pivotTalonConfig.CurrentLimits.SupplyCurrentLimit = 25.0;
    pivotTalonConfig.CurrentLimits.SupplyCurrentThreshold = 40.0;
    pivotTalonConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    pivotTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    pivotTalonConfig.Slot0.kP = 18.0;
    pivotTalonConfig.Slot0.kI = 0.0;
    pivotTalonConfig.Slot0.kD = 0.1;

    (void)m_pPivotMotors->GetMotorObject(PIVOT_MOTORS_CAN_START_ID)->GetConfigurator().Apply(pivotTalonConfig);
    (void)m_pPivotMotors->GetMotorObject(PIVOT_MOTORS_CAN_START_ID)->GetConfigurator().SetPosition(0.0_tr);

    // Configure lift motor controller
    TalonFXConfiguration liftTalonConfig;
    liftTalonConfig.Feedback.SensorToMechanismRatio = 25.0;
    liftTalonConfig.ClosedLoopGeneral.ContinuousWrap = true;

    (void)m_pLiftMotors->GetMotorObject(LIFT_MOTORS_CAN_START_ID)->GetConfigurator().Apply(liftTalonConfig);
    (void)m_pLiftMotors->GetMotorObject(LIFT_MOTORS_CAN_START_ID)->GetConfigurator().SetPosition(0.0_tr);
    (void)m_pLiftMotors->GetMotorObject(LIFT_MOTORS_CAN_START_ID + 1)->GetConfigurator().Apply(liftTalonConfig);
    (void)m_pLiftMotors->GetMotorObject(LIFT_MOTORS_CAN_START_ID + 1)->GetConfigurator().SetPosition(0.0_tr);

    m_pIntakeMotor->m_pTalonFx->SetNeutralMode(NeutralModeValue::Coast);
    m_pFeederMotor->m_pTalonFx->SetNeutralMode(NeutralModeValue::Coast);
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

    (void)m_pPivotMotors->GetMotorObject(PIVOT_MOTORS_CAN_START_ID)->GetConfigurator().SetPosition(0.0_tr);
    (void)m_pLiftMotors->GetMotorObject(LIFT_MOTORS_CAN_START_ID)->GetConfigurator().SetPosition(0.0_tr);
    (void)m_pLiftMotors->GetMotorObject(LIFT_MOTORS_CAN_START_ID + 1)->GetConfigurator().SetPosition(0.0_tr);

    // Stop/clear any timers, just in case
    // @todo: Make this a dedicated function.
    m_pMatchModeTimer->Stop();
    m_pMatchModeTimer->Reset();
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
    
    // Just in case constructor was called before these were set (likely the case)
    m_AllianceColor = DriverStation::GetAlliance();

    // Set the LEDs to the alliance color
    double ledPwmValue = (m_AllianceColor.value() == DriverStation::Alliance::kRed) ? 0.61 : 0.87;
    m_pBlinkin->Set(ledPwmValue);

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

    IntakeSequence();
    ShootSequence();
    PivotSequence();
    //LiftSequence();
    CheckAndUpdateAmpValues();

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
/// @method EastTechRobot::IntakeSequence
///
/// Main workflow for controlling the intake.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::IntakeSequence()
{
    if (std::abs(m_pAuxController->GetAxisValue(AUX_INTAKE_AXIS)) > AXIS_INPUT_DEAD_BAND)
    {
        m_pIntakeMotor->SetDutyCycle(INTAKE_MOTOR_SPEED);
        m_pFeederMotor->SetDutyCycle(FEEDER_MOTOR_SPEED);
        m_PivotTargetDegrees = PIVOT_ANGLE_INTAKE_NOTE;
        m_bIntakeInProgress = true;
    }
    else if (m_pAuxController->GetButtonState(AUX_INTAKE_OUT_BUTTON))
    {
        m_pIntakeMotor->SetDutyCycle(-INTAKE_MOTOR_SPEED);
        m_pFeederMotor->SetDutyCycle(-FEEDER_MOTOR_SPEED);
        m_PivotTargetDegrees = PIVOT_ANGLE_INTAKE_NOTE;
        m_bIntakeInProgress = true;
    }
    else if (m_pAuxController->GetButtonState(AUX_INTAKE_AT_SOURCE_BUTTON))
    {
        // Same angle as when touching the amp
        m_pFeederMotor->SetDutyCycle(FEEDER_MOTOR_SPEED);
        m_pShooterMotors->Set(SHOOTER_MOTOR_LOAD_AT_SOURCE_SPEED);
        m_PivotTargetDegrees = m_AmpTargetDegrees;
        m_bIntakeInProgress = true;
    }
    else
    {
        m_pIntakeMotor->SetDutyCycle(0.0);
        m_pFeederMotor->SetDutyCycle(0.0);
        if (!m_bShotInProgress && !m_bPivotTareInProgress && !m_bCameraAlignInProgress)
        {
            m_PivotTargetDegrees = PIVOT_ANGLE_RUNTIME_BASE;
        }
        m_bIntakeInProgress = false;
    }
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::PivotSequence
///
/// Main workflow for pivoting the superstructure mechanism.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::PivotSequence()
{
    static TalonFX * pPivotLeaderTalon = m_pPivotMotors->GetMotorObject(PIVOT_MOTORS_CAN_START_ID);

    // If the tare button is being held
    if (m_pAuxController->GetButtonState(AUX_TARE_PIVOT_ANGLE))
    {
        // Allow manual movement
        double manualPivotInput = m_pAuxController->GetAxisValue(AUX_MANUAL_PIVOT_AXIS);
        if (std::abs(manualPivotInput) > AXIS_INPUT_DEAD_BAND)
        {
            // Limit manual control max speed
            constexpr const double MANUAL_PIVOT_SCALING_FACTOR = 0.15;
            m_pPivotMotors->Set(-(manualPivotInput * MANUAL_PIVOT_SCALING_FACTOR));
        }
        m_bPivotTareInProgress = true;
    }
    // When the tare button is released, set the new zero
    if (m_pAuxController->DetectButtonChange(AUX_TARE_PIVOT_ANGLE, EastTech::Controller::ButtonStateChanges::BUTTON_RELEASED))
    {
        (void)pPivotLeaderTalon->GetConfigurator().SetPosition(0.0_tr);
        m_bPivotTareInProgress = false;
    }

    units::angle::turn_t pivotAngleTurns = pPivotLeaderTalon->GetPosition().GetValue();
    units::angle::degree_t pivotAngleDegrees = pivotAngleTurns;
    SmartDashboard::PutNumber("Pivot angle", pivotAngleDegrees.value());
    SmartDashboard::PutNumber("Target pivot angle", m_PivotTargetDegrees.value());

    if (m_bPivotTareInProgress)
    {
        return;
    }

    // Only update the pivot target if the auto camera align didn't set one
    if (m_bShotInProgress && !m_bCameraAlignInProgress)
    {
        // If an intake is in progress, it will set the target pivot angle.
        // If an intake is not in progress, move to the target position for amp or speaker
        if (m_bShootSpeaker)
        {
            if (m_bShootSpeakerClose)
            {
                m_PivotTargetDegrees = PIVOT_ANGLE_TOUCHING_SPEAKER;
            }
            else
            {
                m_PivotTargetDegrees = PIVOT_ANGLE_FROM_PODIUM;
            }
        }
        else
        {
            m_PivotTargetDegrees = m_AmpTargetDegrees;
        }
    }
    m_pPivotMotors->SetAngle(m_PivotTargetDegrees.value());
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::ShootSequence
///
/// Main workflow for handling shoot requests.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::ShootSequence()
{
    if (m_pAuxController->DetectButtonChange(AUX_TOGGLE_SPEAKER_AMP_SHOOT_BUTTON))
    {
        m_bShootSpeaker = !m_bShootSpeaker;
    }
    if (m_pAuxController->DetectButtonChange(AUX_TOGGLE_SPEAKER_SHOOT_CLOSE) && m_bShootSpeaker)
    {
        m_bShootSpeakerClose = !m_bShootSpeakerClose;
    }

    SmartDashboard::PutBoolean("Shoot speaker", m_bShootSpeaker);
    SmartDashboard::PutBoolean("Speaker close", m_bShootSpeakerClose);

    enum ShootState
    {
        NOT_SHOOTING,
        WAIT_FOR_PIVOT,
        BACK_FEED,
        RAMPING_UP,
        SHOOTING
    };
    static ShootState shootState = NOT_SHOOTING;
    static Timer * pShootTimer = new Timer();

    // Constants used in the cases below
    const double TARGET_SHOOTER_SPEAKER_SPEED = (m_bShootSpeakerClose) ? SHOOTER_MOTOR_SPEAKER_CLOSE_SPEED : SHOOTER_MOTOR_SPEAKER_FAR_SPEED;
    const double TARGET_SHOOTER_SPEED = (m_bShootSpeaker) ? TARGET_SHOOTER_SPEAKER_SPEED : m_AmpTargetSpeed;
    const double TARGET_SHOOTER_OFFSET_SPEED = (m_bShootSpeaker) ? SHOOTER_MOTOR_SPEAKER_OFFSET_SPEED : 0.0;
    const double BACK_FEED_SPEED = 0.2;
    const units::time::second_t TARGET_BACK_FEED_TIME_S = (m_bShootSpeaker) ? 0.15_s : 0.08_s;
    const units::time::second_t WAIT_FOR_PIVOT_MECHANISM_TIME_S = (m_bShootSpeaker) ? 0.5_s : 1.0_s;
    const units::time::second_t RAMP_UP_TIME_S = 1.5_s;

    double feederSpeed = 0.0;
    double shootSpeed = 0.0;
    double shootSpeedOffset = 0.0;
    if (std::abs(m_pAuxController->GetAxisValue(AUX_SHOOT_AXIS)) > AXIS_INPUT_DEAD_BAND)
    {
        switch (shootState)
        {
            case NOT_SHOOTING:
            {
                // Start a timer to control the shooting process.
                // Make sure all motors are off.
                feederSpeed = 0.0;
                shootSpeed = 0.0;
                shootSpeedOffset = 0.0;

                pShootTimer->Reset();
                pShootTimer->Start();
                m_bShotInProgress = true;
                shootState = WAIT_FOR_PIVOT;
                break;
            }
            case WAIT_FOR_PIVOT:
            {
                // A brief delay for the mechanism to move into the
                // appropriate shooting position.  Motors still off.
                feederSpeed = 0.0;
                shootSpeed = 0.0;
                shootSpeedOffset = 0.0;

                if (pShootTimer->Get() > WAIT_FOR_PIVOT_MECHANISM_TIME_S)
                {
                    pShootTimer->Reset();
                    shootState = BACK_FEED;
                }
                break;
            }
            case BACK_FEED:
            {
                // Mechanism in position.  Feeder on reverse, shooter
                // slowly spinning in case the note is touching.
                feederSpeed = -FEEDER_MOTOR_SPEED;
                shootSpeed = BACK_FEED_SPEED;
                shootSpeedOffset = 0.0;

                if (pShootTimer->Get() > TARGET_BACK_FEED_TIME_S)
                {
                    pShootTimer->Reset();
                    shootState = RAMPING_UP;
                }
                break;
            }
            case RAMPING_UP:
            {
                // Mechanism in position.  Feeder off, start ramping
                // up the shooter motors.
                feederSpeed = 0.0;
                shootSpeed = TARGET_SHOOTER_SPEED;
                shootSpeedOffset = TARGET_SHOOTER_OFFSET_SPEED;

                if (pShootTimer->Get() > RAMP_UP_TIME_S)
                {
                    pShootTimer->Stop();
                    shootState = SHOOTING;
                }
                break;
            }
            case SHOOTING:
            {
                // Mechanism in position, shooter motors at speed.
                // Enable feeder to take the shot.
                feederSpeed = FEEDER_MOTOR_SPEED;
                shootSpeed = TARGET_SHOOTER_SPEED;
                shootSpeedOffset = TARGET_SHOOTER_OFFSET_SPEED;
            }
            default:
            {
                break;
            }
        }

        // Sets the motors to the values configured above
        m_pShooterMotors->Set(shootSpeed, shootSpeedOffset);
        m_pFeederMotor->SetDutyCycle(feederSpeed);
    }
    else
    {
        // Feeder motor is not disabled because it is controlled
        // by the intake sequence.
        pShootTimer->Stop();
        if (!m_bIntakeInProgress)
        {
            m_pShooterMotors->Set(0.0, 0.0);
        }
        m_bShotInProgress = false;
        shootState = NOT_SHOOTING;
    }
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::LiftSequence
///
/// Main workflow for handling lift requests.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::LiftSequence()
{
    // CCW is positive rotation, CW is negative rotation.
    // Pigeon orientation is such that robot roll is the reported pitch.
    // 14 inches of total travel, 80:1, need effective rotation radius/diameter.
    // ~4 inch circumference -> 1.2732 diameter => 14 inches = 3.5 turns
    // Measured travel distance is 11.78 in turns.  ~56.55 inches (off by factor of 4?).

    enum TiltDirection
    {
        NO_TILT,
        LEFT_TILT,
        RIGHT_TILT
    };
    static enum TiltDirection tiltDirection = NO_TILT;

    double liftLeaderTurns = std::abs(m_pLiftMotors->GetMotorObject(LIFT_MOTORS_CAN_START_ID)->GetPosition().GetValue().value());
    double liftFollowerTurns = std::abs(m_pLiftMotors->GetMotorObject(LIFT_MOTORS_CAN_START_ID + 1)->GetPosition().GetValue().value());
    double highestTurns = std::max(liftLeaderTurns, liftFollowerTurns);
    double lowestTurns = std::min(liftLeaderTurns, liftFollowerTurns);
    const double LIFT_MIN_TURNS = 1.0;
    const double LIFT_MAX_TURNS = 9.0;

    bool bTravelAllowed = false;
    double liftSpeed = 0.0;
    EastTech::Controller::PovDirections drivePovDirection = m_pDriveController->GetPovAsDirection();
    switch (drivePovDirection)
    {
        // Going up to grab the chain
        case EastTech::Controller::PovDirections::POV_UP:
        {
            if (highestTurns < LIFT_MAX_TURNS)
            {
                bTravelAllowed = true;
                liftSpeed = LIFT_MOTOR_SPEED;
            }
            break;
        }
        // Pulling down to raise the robot
        case EastTech::Controller::PovDirections::POV_DOWN:
        {
            if (lowestTurns > LIFT_MIN_TURNS)
            {
                bTravelAllowed = true;
                liftSpeed = -LIFT_MOTOR_SPEED;
            }
            break;
        }
        default:
        {
            break;
        }
    }

    double liftOffsetSpeed = 0.0;
    double robotRoll = m_pPigeon->GetPitch().GetValue().value();

    switch (tiltDirection)
    {
        case NO_TILT:
        {    
            // Positive angle means left side needs to drive harder.
            // Negative angle means right side needs to drive harder.
            // This needs to be updated to not stop until is passes a threshold (probably zero).
            if ((robotRoll > LIFT_MAX_ROLL_DEGREES) && bTravelAllowed)
            {
                liftOffsetSpeed = LIFT_MOTOR_OFFSET_SPEED;
                tiltDirection = LEFT_TILT;
            }
            else if ((robotRoll < -LIFT_MAX_ROLL_DEGREES) && bTravelAllowed)
            {
                liftOffsetSpeed = -LIFT_MOTOR_OFFSET_SPEED;
                tiltDirection = RIGHT_TILT;
            }
            else
            {
                // The offset is still zero from variable initialization
            }
            break;
        }
        case LEFT_TILT:
        {
            // Robot roll was positive, watch for it to drop back near zero
            if (robotRoll < LIFT_OFFSET_STOP_POINT_DEGREES)
            {
                // The offset is still zero from variable initialization
                tiltDirection = NO_TILT;
            }
            else
            {
                if (bTravelAllowed)
                {
                    liftOffsetSpeed = LIFT_MOTOR_OFFSET_SPEED;
                }
            }
            break;
        }
        case RIGHT_TILT:
        {
            // Robot roll was negative, watch for it to rise back near zero
            if (robotRoll > -LIFT_OFFSET_STOP_POINT_DEGREES)
            {
                // The offset is still zero from variable initialization
                tiltDirection = NO_TILT;
            }
            else
            {
                if (bTravelAllowed)
                {
                    liftOffsetSpeed = -LIFT_MOTOR_OFFSET_SPEED;
                }
            }
            break;
        }
        default:
        {
            break;
        }
    }

    SmartDashboard::PutNumber("Debug A", liftLeaderTurns);
    SmartDashboard::PutNumber("Debug B", liftFollowerTurns);
    SmartDashboard::PutNumber("Debug C", liftSpeed);
    SmartDashboard::PutNumber("Debug D", liftOffsetSpeed);
    SmartDashboard::PutNumber("Debug E", tiltDirection);
    m_pLiftMotors->Set(liftSpeed, liftOffsetSpeed);
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::CheckAndUpdateAmpValues
///
/// Checks for change requests to the amp target angle or
/// shooter motors speed values.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::CheckAndUpdateAmpValues()
{
    static EastTech::Controller::PovDirections lastAuxPovDirection = EastTech::Controller::PovDirections::POV_NOT_PRESSED;
    EastTech::Controller::PovDirections currentAuxPovDirection = m_pAuxController->GetPovAsDirection();
    if (currentAuxPovDirection != lastAuxPovDirection)
    {
        // @todo: Limit these to min/max values
        switch (currentAuxPovDirection)
        {
            case EastTech::Controller::PovDirections::POV_UP:
            {
                // Motor output is negative, so decrease for faster speed
                m_AmpTargetSpeed -= SHOOTER_STEP_SPEED;
                break;
            }
            case EastTech::Controller::PovDirections::POV_DOWN:
            {
                // Motor output is negative, so increase for slower speed
                m_AmpTargetSpeed += SHOOTER_STEP_SPEED;
                break;
            }
            case EastTech::Controller::PovDirections::POV_RIGHT:
            {
                // Motor output is negative, so increase for slower speed
                m_AmpTargetDegrees += SHOOTER_STEP_ANGLE;
                break;
            }
            case EastTech::Controller::PovDirections::POV_LEFT:
            {
                // Motor output is negative, so increase for slower speed
                m_AmpTargetDegrees -= SHOOTER_STEP_ANGLE;
                break;
            }
            default:
            {
                break;
            }
        }

        lastAuxPovDirection = currentAuxPovDirection;
    }
    SmartDashboard::PutNumber("Amp speed", m_AmpTargetSpeed);
    SmartDashboard::PutNumber("Amp angle", m_AmpTargetDegrees.value());
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

    if (m_pDriveController->DetectButtonChange(ZERO_GYRO_YAW_BUTTON))
    {
        m_pSwerveDrive->ZeroGyroYaw();
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

    // Enable an LED display pattern (Ocean Palette Rainbow)
    m_pBlinkin->Set(-0.95);

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
