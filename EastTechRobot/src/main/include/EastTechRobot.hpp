////////////////////////////////////////////////////////////////////////////////
/// @file   EastTechRobot.hpp
/// @author David Stalter
///
/// @details
/// This is the class declaration for a FRC robot derived from the WPI library
/// base classes.  The TimedRobot class is the base of a robot application that
/// will automatically call appropriate Autonomous and Teleop methods at the
/// right time as controlled by the switches on the driver station or the field
/// controls.
///
/// Copyright (c) 2025 East Technical High School
////////////////////////////////////////////////////////////////////////////////

#ifndef EASTTECHROBOT_HPP
#define EASTTECHROBOT_HPP

// SYSTEM INCLUDES
#include <cmath>                                // for M_PI
#include <thread>                               // for std::thread

// C INCLUDES
#include "frc/Compressor.h"                     // for retrieving info on the compressor
#include "frc/DigitalInput.h"                   // for DigitalInput type
#include "frc/DigitalOutput.h"                  // for DigitalOutput type
#include "frc/DoubleSolenoid.h"                 // for DoubleSolenoid type
#include "frc/DriverStation.h"                  // for interacting with the driver station
#include "frc/DutyCycleEncoder.h"               // for interacting with the REV through bore encoders
#include "frc/Relay.h"                          // for Relay type
#include "frc/Solenoid.h"                       // for Solenoid type
#include "frc/TimedRobot.h"                     // for base class decalartion
#include "frc/livewindow/LiveWindow.h"          // for controlling the LiveWindow
#include "frc/motorcontrol/Spark.h"             // for creating an object to interact with the rev blinkin
#include "frc/smartdashboard/SendableChooser.h" // for using the smart dashboard sendable chooser functionality
#include "frc/smartdashboard/SmartDashboard.h"  // for interacting with the smart dashboard

// C++ INCLUDES
#include "DriveConfiguration.hpp"               // for information on the drive config
#include "EastTechController.hpp"               // for controller interaction
#include "EastTechTalon.hpp"                    // for custom Talon control
#include "RobotUtils.hpp"                       // for ASSERT, DEBUG_PRINTS
#include "SwerveDrive.hpp"                      // for using swerve drive
#include "ctre/phoenix/led/CANdle.h"
#include "ctre/phoenix/led/RainbowAnimation.h"
#include "ctre/phoenix6/Pigeon2.hpp"            // for PigeonIMU

using namespace frc;
using namespace ctre::phoenix::led;


////////////////////////////////////////////////////////////////
/// @class EastTechRobot
///
/// Derived class from TimedRobot.  The object that will
/// control all robot functionality.
///
////////////////////////////////////////////////////////////////
class EastTechRobot : public TimedRobot
{
public:
    friend class RobotCamera;
    friend class EastTechRobotTest;

    // MEMBER FUNCTIONS
    
    // Base robot routines
    virtual void RobotInit() override;
    virtual void RobotPeriodic() override;
    
    // Autonomous routines
    virtual void AutonomousInit() override;
    virtual void AutonomousPeriodic() override;
    
    // Teleop routines
    virtual void TeleopInit() override;
    virtual void TeleopPeriodic() override;
    
    // Test mode routines
    virtual void TestInit() override;
    virtual void TestPeriodic() override;
    
    // Robot disabled routines
    virtual void DisabledInit() override;
    virtual void DisabledPeriodic() override;
    
    // Constructor, destructor, copy, assignment
    EastTechRobot();
    virtual ~EastTechRobot() = default;
    EastTechRobot(EastTechRobot&& rhs) = default;
    EastTechRobot& operator=(EastTechRobot&& rhs) = default;
      
private:

    // TYPEDEFS
    typedef EastTech::Talon::MotorGroupControlMode MotorGroupControlMode;
    typedef EastTech::Talon::TalonFxMotorController TalonFxMotorController;
    typedef EastTech::Controller::Config::Models ControllerModels;
    typedef EastTech::Controller::Config::Mappings ControllerMappings;
    typedef EastTechDriveController<EastTechCustomController> DriveControllerType;
    typedef EastTechController<EastTechCustomController> AuxControllerType;
    
    // ENUMS
    enum RobotMode
    {
        ROBOT_MODE_AUTONOMOUS,
        ROBOT_MODE_TELEOP,
        ROBOT_MODE_TEST,
        ROBOT_MODE_DISABLED,
        ROBOT_MODE_NOT_SET
    };

    enum class RobotDirection
    {
        ROBOT_NO_DIRECTION,
        ROBOT_FORWARD,
        ROBOT_REVERSE,
        ROBOT_LEFT,
        ROBOT_RIGHT
    };

    enum class RobotTranslation
    {
        ROBOT_NO_TRANSLATION,
        ROBOT_TRANSLATION_FORWARD,
        ROBOT_TRANSLATION_REVERSE
    };

    enum class RobotStrafe
    {
        ROBOT_NO_STRAFE,
        ROBOT_STRAFE_LEFT,
        ROBOT_STRAFE_RIGHT
    };

    enum class RobotRotation
    {
        ROBOT_NO_ROTATION,
        ROBOT_CLOCKWISE,
        ROBOT_COUNTER_CLOCKWISE
    };

    enum class LiftPosition : uint32_t
    {
        // These must stay in order for the incrementing/decrementing to work right
        LIFT_DOWN,
        LIFT_MIDDLE,
        LIFT_UP
    };

    inline void IncrementLiftPosition(LiftPosition & rLiftPosition)
    {
        if (rLiftPosition != LiftPosition::LIFT_UP)
        {
            uint32_t positionAsUint = static_cast<uint32_t>(rLiftPosition);
            positionAsUint++;
            rLiftPosition = static_cast<LiftPosition>(positionAsUint);
        }
    }

    inline void DecrementLiftPosition(LiftPosition & rLiftPosition)
    {
        if (rLiftPosition != LiftPosition::LIFT_DOWN)
        {
            uint32_t positionAsUint = static_cast<uint32_t>(rLiftPosition);
            positionAsUint--;
            rLiftPosition = static_cast<LiftPosition>(positionAsUint);
        }
    }

    enum class ArmPosition : uint32_t
    {
        // These must stay in order for the incrementing/decrementing to work right
        LOADING,
        NEUTRAL,
        REEF_L1,
        REEF_L2_L3,
        REEF_L4
    };

    inline void IncrementArmPosition(ArmPosition & rArmPosition)
    {
        if (rArmPosition != ArmPosition::REEF_L4)
        {
            uint32_t positionAsUint = static_cast<uint32_t>(rArmPosition);
            positionAsUint++;
            rArmPosition = static_cast<ArmPosition>(positionAsUint);
        }
    }

    inline void DecrementArmPosition(ArmPosition & rArmPosition)
    {
        if (rArmPosition != ArmPosition::LOADING)
        {
            uint32_t positionAsUint = static_cast<uint32_t>(rArmPosition);
            positionAsUint--;
            rArmPosition = static_cast<ArmPosition>(positionAsUint);
        }
    }

    // STRUCTS
    struct RobotSwerveDirections
    {
      public:
        RobotSwerveDirections() : m_Translation(RobotTranslation::ROBOT_NO_TRANSLATION), m_Strafe(RobotStrafe::ROBOT_NO_STRAFE), m_Rotation(RobotRotation::ROBOT_NO_ROTATION) {}
        inline void SetSwerveDirections(RobotTranslation translationDirection, RobotStrafe strafeDirection, RobotRotation rotationDirection)
        {
            m_Translation = translationDirection;
            m_Strafe = strafeDirection;
            m_Rotation = rotationDirection;
        }
        inline RobotTranslation GetTranslation() { return m_Translation; }
        inline RobotStrafe GetStrafe() { return m_Strafe; }
        inline RobotRotation GetRotation() { return m_Rotation; }
      private:
        RobotTranslation m_Translation;
        RobotStrafe m_Strafe;
        RobotRotation m_Rotation;
    };
    
    // This is a hacky way of retrieving a pointer to the robot object
    // outside of the robot class.  The robot object itself is a static
    // variable inside the function StartRobot() in the RobotBase class.
    // This makes retrieving the address difficult.  To work around this,
    // we'll allocate some static storage for a pointer to a robot object.
    // When RobotInit() is called, m_pThis will be filled out.  This works
    // because only one EastTechRobot object is ever constructed.
    static EastTechRobot * m_pThis;
    inline void SetStaticThisInstance() { m_pThis = this; }
    inline static EastTechRobot * GetRobotInstance() { return m_pThis; }

    // Increments a variable to indicate the robot code is successfully running
    inline void HeartBeat();
    
    // Checks for a robot state change and logs a message if so
    inline void CheckAndUpdateRobotMode(RobotMode robotMode);

    // Updates information on the smart dashboard for the drive team
    void UpdateSmartDashboard();

    // Autonomous wait for something to complete delay routine
    inline void AutonomousDelay(units::second_t time);

    // Autonomous drive for a specified time
    inline void AutonomousSwerveDriveSequence(RobotSwerveDirections & rSwerveDirections, double translationSpeed, double strafeSpeed, double rotateSpeed, units::second_t time, bool bFieldRelative);
    
    // Autonomous routines
    // @todo: Make EastTechRobotAutonomous a friend and move these out (requires accessor to *this)!
    void AutonomousRoutine1();
    void AutonomousRoutine2();
    void AutonomousRoutine3();
    void AutonomousTestRoutine();
    void AutonomousTestSwerveRoutine();
    void AutonomousTestTrajectoryRoutine();
    void AutonomousCommon();
    void AutonomousCommonRed();
    void AutonomousCommonBlue();

    // Resets member variables
    void ResetMemberData();

    // Routine to put things in a known state
    void InitialStateSetup();

    // Configure motor controller parameters
    void ConfigureMotorControllers();

    // Main sequence for drive motor control
    void SwerveDriveSequence();

    // Main sequence for controlling pneumatics
    void PneumaticSequence();
    
    // Main sequence for vision processing
    void CameraSequence();

    // LED sequence/support
    inline void SetLedsToAllianceColor();

    // Superstructure sequences
    void LiftSequence();
    void ArmSequence();
    void WristSequence();
    void HangSequence();
    void GamePieceControlSequence();
    void WaitForSensorConfig();
    
    // MEMBER VARIABLES
    
    // Autonomous
    SendableChooser<std::string>    m_AutonomousChooser;                    // Selects from the dashboard which auto routine to run
    RobotSwerveDirections           m_AutoSwerveDirections;                 // Used by autonomous routines to control swerve drive movements
    
    // User Controls
    DriveControllerType *           m_pDriveController;                     // Drive controller
    AuxControllerType *             m_pAuxController;                       // Auxillary input controller
    
    // Swerve Drive
    Pigeon2 *                       m_pPigeon;                              // CTRE Pigeon2 IMU
    SwerveDrive *                   m_pSwerveDrive;                         // Swerve drive control
    
    // Motors
    TalonMotorGroup<TalonFX> *      m_pLiftMotors;                          // Lift motor control
    TalonFxMotorController *        m_pArmPivotMotor;                       // Arm pivot motor control
    TalonFxMotorController *        m_pWristPivotMotor;                     // Wrist pivot motor control
    TalonFxMotorController *        m_pGamePieceMotor;                      // Motor for manipulating game pieces
    TalonFxMotorController *        m_pHangMotor;                           // Hang motor control

    // LEDs
    CANdle *                        m_pCandle;
    RainbowAnimation                m_RainbowAnimation;

    // Digital I/O
    DigitalOutput *                 m_pDebugOutput;                         // Debug assist output
    
    // Analog I/O
    // (none)
    
    // Pneumatics
    Compressor *                    m_pCompressor;                          // Object to get info about the compressor
    
    // Servos
    // (none)
    
    // Encoders
    DutyCycleEncoder *              m_pArmAbsoluteEncoder;                  // REV through bore encoder for the arm position
    DutyCycleEncoder *              m_pWristAbsoluteEncoder;                // REV through bore encoder for the wrist position
    
    // Timers
    Timer *                         m_pMatchModeTimer;                      // Times how long a particular mode (autonomous, teleop) is running
    Timer *                         m_pRobotProgramTimer;                   // Starts at robot program entry, free runs for program life time
    Timer *                         m_pSafetyTimer;                         // Fail safe in case critical operations don't complete

    // Camera
    // Note: Only need to have a thread here and tie it to
    // the RobotCamera class, which handles everything else.
    std::thread                     m_CameraThread;
    
    // Misc
    units::angle::degree_t          m_LiftTargetDegrees;                    // Tracks the desired angle position of the lift
    units::angle::degree_t          m_ArmTargetDegrees;                     // Tracks the desired angle position of the arm
    units::angle::degree_t          m_ArmManualOffsetDegrees;               // Tracks the desired manual offset of the arm angle
    units::angle::degree_t          m_WristTargetDegrees;                   // Tracks the desired angle position of the wrist
    units::angle::degree_t          m_WristManualOffsetDegrees;             // Tracks the desired manual offset of the wrist angle
    LiftPosition                    m_LiftPosition;                         // Keep track of where the lift is currently positioned
    ArmPosition                     m_ArmPosition;                          // Keep track of where the arm is currently positioned
    RobotMode                       m_RobotMode;                            // Keep track of the current robot state
    std::optional
    <DriverStation::Alliance>       m_AllianceColor;                        // Color reported by driver station during a match
    bool                            m_AbsoluteEncodersInitialized;          // Indicates whether the absolute encoders have been initialized
    bool                            m_bCameraAlignInProgress;               // Indicates if an automatic camera align is in progress
    uint32_t                        m_HeartBeat;                            // Incremental counter to indicate the robot code is executing
    
    // CONSTS
    
    // Joysticks/Buttons
    // Note: Don't forget to update the controller object typedefs if
    //       necessary when changing these types!
    static const ControllerModels DRIVE_CONTROLLER_MODEL                        = ControllerModels::CUSTOM_XBOX;
    static const ControllerModels AUX_CONTROLLER_MODEL                          = ControllerModels::CUSTOM_XBOX;
    static constexpr const ControllerMappings * const DRIVE_CONTROLLER_MAPPINGS = EastTech::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL);
    static constexpr const ControllerMappings * const AUX_CONTROLLER_MAPPINGS   = EastTech::Controller::Config::GetControllerMapping(AUX_CONTROLLER_MODEL);
    
    static const int                DRIVE_JOYSTICK_PORT                     = 0;
    static const int                AUX_JOYSTICK_PORT                       = 1;

    // Driver inputs
    static const int                DRIVE_HANG_UP_BUTTON                    = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_BUTTON;
    static const int                DRIVE_HANG_DOWN_BUTTON                  = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.DOWN_BUTTON;
    static const int                FIELD_RELATIVE_TOGGLE_BUTTON            = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.LEFT_BUMPER;
    static const int                REZERO_SWERVE_BUTTON                    = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_BUMPER;
    static const int                DRIVE_ALIGN_WITH_CAMERA_BUTTON          = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_STICK_CLICK;
    static const int                CAMERA_TOGGLE_FULL_PROCESSING_BUTTON    = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    static const int                CAMERA_TOGGLE_PROCESSED_IMAGE_BUTTON    = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    static const int                SELECT_FRONT_CAMERA_BUTTON              = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    static const int                SELECT_BACK_CAMERA_BUTTON               = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;
    
    // Aux inputs
    static const int                AUX_WRIST_IN_AXIS                       = AUX_CONTROLLER_MAPPINGS->AXIS_MAPPINGS.LEFT_TRIGGER;
    static const int                AUX_WRIST_OUT_AXIS                      = AUX_CONTROLLER_MAPPINGS->AXIS_MAPPINGS.RIGHT_TRIGGER;
    static const int                AUX_ARM_TOWARDS_LOAD_BUTTON             = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.LEFT_BUMPER;
    static const int                AUX_ARM_TOWARDS_REEF_BUTTON             = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_BUMPER;
    static const int                AUX_GAME_PIECE_IN_BUTTON                = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.DOWN_BUTTON;
    static const int                AUX_GAME_PIECE_OUT_BUTTON               = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_BUTTON;
    static const int                AUX_TARE_LIFT_BUTTON                    = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.START;
    static const int                AUX_TOGGLE_LIFT_BRAKE_COAST_BUTTON      = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.SELECT;
    static const int                AUX_INCREASE_ANGLE_OFFSET_BUTTON        = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.UP_BUTTON;
    static const int                AUX_DECREASE_ANGLE_OFFSET_BUTTON        = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.LEFT_BUTTON;
    static const int                AUX_TOGGLE_ANGLE_ADJUST_BUTTON          = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.LEFT_STICK_CLICK;
    static const int                ESTOP_BUTTON                            = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;

    static const EastTech::Controller::PovDirections  AUX_CONTROLS_LIFT_UP                  = EastTech::Controller::PovDirections::POV_UP;
    static const EastTech::Controller::PovDirections  AUX_CONTROLS_LIFT_DOWN                = EastTech::Controller::PovDirections::POV_DOWN;

    // CAN Signals
    // Note: Remember to check the CAN IDs in use in SwerveDrive.hpp.
    // Superstructure uses IDs starting at 21
    static const unsigned           LIFT_MOTORS_CAN_START_ID                = 21;
    static const unsigned           ARM_PIVOT_MOTOR_CAN_ID                  = 23;
    static const unsigned           WRIST_PIVOT_MOTOR_CAN_ID                = 24;
    static const unsigned           GAME_PIECE_MOTOR_CAN_ID                 = 25;
    static const unsigned           HANG_MOTOR_CAN_ID                       = 26;

    // CANivore Signals
    // Note: IDs 1-4 are used by the CANcoders (see the
    //       SwerveModuleConfigs in SwerveDrive.hpp).
    static const int                PIGEON_CAN_ID                           = 5;
    static const int                CANDLE_CAN_ID                           = 6;

    // PWM Signals
    // (none)
    
    // Relays
    // (none)
    
    // Digital I/O Signals
    static const int                ARM_ABSOLUTE_ENCODER_DIO_CHANNEL        = 0;
    static const int                WRIST_ABSOLUTE_ENCODER_DIO_CHANNEL      = 1;
    static const int                DEBUG_OUTPUT_DIO_CHANNEL                = 7;
    
    // Analog I/O Signals
    // (none)
    
    // Solenoid Signals
    // (none)

    // Solenoids
    // (none)

    // Motor speeds and angles
    static constexpr const double GAME_PIECE_MOTOR_SPEED_FAST                           = 0.30;
    static constexpr const double GAME_PIECE_MOTOR_SPEED_SLOW                           = 0.15;
    static constexpr const double GAME_PIECE_MOTOR_SPEED_INTAKE                         = 0.10;

    static constexpr const units::angle::degree_t LIFT_DOWN_ANGLE                       = 45.0_deg;
    static constexpr const units::angle::degree_t LIFT_MIDDLE_ANGLE                     = 1000.0_deg;
    static constexpr const units::angle::degree_t LIFT_UP_ANGLE                         = 1890.0_deg;

    static constexpr const units::angle::degree_t ARM_STARTING_POSITION_DEGREES         = 180.0_deg;
    static constexpr const units::angle::degree_t ARM_STARTING_POSITION_ENCODER_VALUE   = units::angle::degree_t(0.0985 * 360.0);
    static constexpr const units::angle::degree_t ARM_ENDING_POSITION_ENCODER_VALUE     = units::angle::degree_t(0.0000 * 360.0);
    static constexpr const units::angle::degree_t ARM_LOADING_TARGET_DEGREES            = 195.0_deg;
    static constexpr const units::angle::degree_t ARM_NEUTRAL_TARGET_DEGREES            = 180.0_deg;
    static constexpr const units::angle::degree_t ARM_REEF_L1_TARGET_DEGREES            = 75.0_deg;
    static constexpr const units::angle::degree_t ARM_REEF_L2_L3_TARGET_DEGREES         = 30.0_deg;
    static constexpr const units::angle::degree_t ARM_REEF_L4_TARGET_DEGREES            = -15.0_deg;

    static constexpr const units::angle::degree_t ARM_WRIST_MANUAL_ADJUST_STEP_DEGREES  = 10.0_deg;

    static constexpr const units::angle::degree_t WRIST_STARTING_POSITION_DEGREES       = 135.0_deg;
    static constexpr const units::angle::degree_t WRIST_STARTING_POSITION_ENCODER_VALUE = units::angle::degree_t(0.0517 * 360.0);
    static constexpr const units::angle::degree_t WRIST_ENDING_POSITION_ENCODER_VALUE   = units::angle::degree_t(0.0000 * 360.0);
    static constexpr const units::angle::degree_t WRIST_LOADING_TARGET_DEGREES          = 35.0_deg;
    static constexpr const units::angle::degree_t WRIST_NEUTRAL_TARGET_DEGREES          = 135.0_deg;
    static constexpr const units::angle::degree_t WRIST_REEF_L1_TARGET_DEGREES          = 135.0_deg;
    static constexpr const units::angle::degree_t WRIST_REEF_L2_L3_TARGET_DEGREES       = 195.0_deg;
    static constexpr const units::angle::degree_t WRIST_REEF_L4_TARGET_DEGREES          = 60.0_deg;

    static constexpr const units::angle::degree_t ENCODER_BOUNDARY_TOLERANCE_DEGREES    = -5.0_deg;

    static constexpr const bool     SUPERSTRUCTURE_MOTION_ENABLED           = true;
    
    // Misc
    const std::string               AUTO_NO_ROUTINE_STRING                  = "No autonomous routine";
    const std::string               AUTO_ROUTINE_1_STRING                   = "Autonomous Routine 1";
    const std::string               AUTO_ROUTINE_2_STRING                   = "Autonomous Routine 2";
    const std::string               AUTO_ROUTINE_3_STRING                   = "Autonomous Routine 3";
    const std::string               AUTO_TEST_ROUTINE_STRING                = "Autonomous Test Routine";
    static const int                OFF                                     = 0;
    static const int                ON                                      = 1;
    static const int                ANGLE_90_DEGREES                        = 90;
    static const int                ANGLE_180_DEGREES                       = 180;
    static const int                ANGLE_360_DEGREES                       = 360;
    static const int                POV_INPUT_TOLERANCE_VALUE               = 30;
    static const int                SCALE_TO_PERCENT                        = 100;
    static const unsigned           SINGLE_MOTOR                            = 1;
    static const unsigned           TWO_MOTORS                              = 2;
    static const unsigned           NUMBER_OF_LEDS                          = 308;
    static const char               NULL_CHARACTER                          = '\0';

    static const unsigned           CAMERA_RUN_INTERVAL_MS                  = 1000U;
    
    static constexpr double         JOYSTICK_TRIM_UPPER_LIMIT               =  0.10;
    static constexpr double         JOYSTICK_TRIM_LOWER_LIMIT               = -0.10;
    static constexpr double         SWERVE_ROTATE_SLOW_JOYSTICK_THRESHOLD   =  0.10;
    static constexpr double         SWERVE_DRIVE_SLOW_SPEED                 =  0.10;
    static constexpr double         SWERVE_ROTATE_SLOW_SPEED                =  0.10;
    static constexpr double         AXIS_INPUT_DEAD_BAND                    =  0.10;

    static constexpr units::second_t    SAFETY_TIMER_MAX_VALUE_S            =  5.00_s;


    // The below code/equations are for arcade drive, not swerve drive.

    // These indicate which motor value (+1/-1) represent
    // forward/reverse in the robot.  They are used to keep
    // autonomous movement code common without yearly updates.

    static constexpr double         LEFT_DRIVE_FORWARD_SCALAR               = -1.00;
    static constexpr double         LEFT_DRIVE_REVERSE_SCALAR               = +1.00;
    static constexpr double         RIGHT_DRIVE_FORWARD_SCALAR              = +1.00;
    static constexpr double         RIGHT_DRIVE_REVERSE_SCALAR              = -1.00;

    ////////////////////////////////////////////////////////////////
    // Inputs from joystick:
    //
    // Forward:     (0, -1)
    // Reverse:     (0, +1)
    // Left:        (-1, 0)
    // Right:       (+1, 0)
    //
    // Equations:
    //
    //     x+y   x-y   -x+y   -x-y
    // F:   -1    +1     -1     +1
    // B:   +1    -1     +1     -1
    // L:   -1    -1     +1     +1
    // R:   +1    +1     -1     -1
    //
    // Output to motors:
    //
    // Left forward/right = +1, Right forward/left  = +1:
    // Left reverse/left  = -1, Right reverse/right = -1:
    // x-y, -x-y
    //
    // Left forward/right = -1, Right forward/left  = -1:
    // Left reverse/left  = +1, Right reverse/right = +1:
    // -x+y, x+y
    //
    // Left forward/right = +1, Right forward/left  = -1:
    // Left reverse/left  = -1, Right reverse/right = +1:
    // x-y, x+y
    //
    // Left forward/right = -1, Right forward/left  = +1:
    // Left reverse/left  = +1, Right reverse/right = -1:
    // -x+y, -x-y
    ////////////////////////////////////////////////////////////////

    inline static constexpr double LeftDriveEquation(double xInput, double yInput)
    {
        double leftValue = 0.0;

        if (static_cast<int>(LEFT_DRIVE_FORWARD_SCALAR) == 1)
        {
            leftValue = xInput - yInput;
        }
        else
        {
            leftValue = -xInput + yInput;
        }
        
        return leftValue;
    }

    inline static constexpr double RightDriveEquation(double xInput, double yInput)
    {
        double rightValue = 0.0;

        if (static_cast<int>(RIGHT_DRIVE_FORWARD_SCALAR) == 1)
        {
            rightValue = -xInput - yInput;
        }
        else
        {
            rightValue = xInput + yInput;
        }
        
        return rightValue;
    }

};  // End class



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::SetLedsToAllianceColor
///
/// Sets the LEDs to the alliance color.
///
////////////////////////////////////////////////////////////////
inline void EastTechRobot::SetLedsToAllianceColor()
{
    switch (m_AllianceColor.value())
    {
        case DriverStation::Alliance::kRed:
        {
            m_pCandle->SetLEDs(255, 0, 0, 0, 0, NUMBER_OF_LEDS);
            break;
        }
        case DriverStation::Alliance::kBlue:
        {
            m_pCandle->SetLEDs(0, 0, 255, 0, 0, NUMBER_OF_LEDS);
            break;
        }
        default:
        {
            break;
        }
    }
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::HeartBeat
///
/// Increments the heartbeat counter.
///
////////////////////////////////////////////////////////////////
inline void EastTechRobot::HeartBeat()
{
    m_HeartBeat++;
    SmartDashboard::PutNumber("Heartbeat", m_HeartBeat);
}



////////////////////////////////////////////////////////////////
/// @method EastTechRobot::CheckAndUpdateRobotMode
///
/// Checks the current robot mode for a state change and updates
/// accordingly, including displaying a message.
///
////////////////////////////////////////////////////////////////
void EastTechRobot::CheckAndUpdateRobotMode(RobotMode robotMode)
{
    // These array messages match the order of the RobotMode enum
    const char * MODE_CHANGE_ENTER_MESSAGES[] = 
                {
                    "Autonomous entered.",
                    "Teleop entered.",
                    "Test entered.",
                    "Disabled entered."
                };

    const char * MODE_CHANGE_EXIT_MESSAGES[] = 
                {
                    "Autonomous exited.",
                    "Teleop exited.",
                    "Test exited.",
                    "Disabled exited."
                };
    
    // Check for the mode to have changed
    if (m_RobotMode != robotMode)
    {
        // First display the exit message for the old mode
        RobotUtils::DisplayMessage(MODE_CHANGE_EXIT_MESSAGES[m_RobotMode]);

        // Enter the new mode and display an enter message
        m_RobotMode = robotMode;
        RobotUtils::DisplayMessage(MODE_CHANGE_ENTER_MESSAGES[m_RobotMode]);
    }
}

#endif // EASTTECHROBOT_HPP
