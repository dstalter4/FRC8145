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
/// Copyright (c) 2026 East Technical High School
////////////////////////////////////////////////////////////////////////////////

#ifndef EASTTECHROBOT_HPP
#define EASTTECHROBOT_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/Compressor.h"                                 // for retrieving info on the compressor
#include "frc/DigitalInput.h"                               // for DigitalInput type
#include "frc/DigitalOutput.h"                              // for DigitalOutput type
#include "frc/DoubleSolenoid.h"                             // for DoubleSolenoid type
#include "frc/DriverStation.h"                              // for interacting with the driver station
#include "frc/DutyCycleEncoder.h"                           // for interacting with PWM based encoders
#include "frc/PWM.h"                                        // for interacting with PWM based sensors (e.g. actuators)
#include "frc/Relay.h"                                      // for Relay type
#include "frc/Solenoid.h"                                   // for Solenoid type
#include "frc/TimedRobot.h"                                 // for base class decalartion
#include "frc/livewindow/LiveWindow.h"                      // for controlling the LiveWindow
#include "frc/smartdashboard/SendableChooser.h"             // for using the smart dashboard sendable chooser functionality
#include "frc/smartdashboard/SmartDashboard.h"              // for interacting with the smart dashboard
#include "frc2/command/CommandPtr.h"                        // for CommandPtr

// C++ INCLUDES
#include "DriveConfiguration.hpp"                           // for information on the drive config
#include "EastTechController.hpp"                           // for controller interaction
#include "EastTechLed.hpp"                                  // for LED interaction
#include "EastTechTalon.hpp"                                // for custom Talon control
#include "LimelightCamera.hpp"                              // for creating and interacting with limelight cameras
#include "RobotUtils.hpp"                                   // for ASSERT, DEBUG_PRINTS
#include "SwerveDrive.hpp"                                  // for using swerve drive
#include "ctre/phoenix6/CANBus.hpp"                         // for creating CANBus objects
#include "ctre/phoenix6/Pigeon2.hpp"                        // for PigeonIMU
#include "ctre/phoenix6/SignalLogger.hpp"                   // for disabling automatic signal logging


using namespace frc;
using namespace frc2;
using namespace ctre::phoenix6::hardware;


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

    // STRUCTS
    // (none)

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

    // Autonomous routines
    // @todo: Make EastTechRobotAutonomous a friend and move these out (requires accessor to *this, or lambdas)!
    void AutonomousPeriodicTimed();
    void AutonomousPeriodicCommand();
    void AutonomousCommon();
    void AutonomousCommonRed();
    void AutonomousCommonBlue();
    void AutonomousRoutine1();
    void AutonomousRoutine2();
    void AutonomousRoutine3();
    void AutonomousTestRoutine();
    void AutonomousTestSwerveRoutine();
    CommandPtr AutonomousTestCommandDashboardRoutine();
    CommandPtr AutonomousTestCommandMotionRoutine();
    CommandPtr AutonomousTestTrajectoryRoutine();

    // Resets member variables
    void ResetMemberData();

    // Routine to put things in a known state
    void InitialStateSetup();

    // Checks for the RIO pin readings to stabilize
    void CheckIfRioPinsAreStable();

    // Configure motor controller parameters
    void ConfigureMotorControllers();

    // Main sequence for drive motor control
    void SwerveDriveSequence();

    // Main sequence for controlling pneumatics
    void PneumaticSequence();

    // Main sequence for vision processing
    void CameraSequence();

    // Superstructure sequences
    // (none)

    // MEMBER VARIABLES

    // Autonomous
    SendableChooser<std::string>    m_AutonomousChooser;                    // Selects from the dashboard which auto routine to run

    // User Controls
    DriveControllerType *           m_pDriveController;                     // Drive controller
    AuxControllerType *             m_pAuxController;                       // Auxillary input controller

    // CAN Bus
    CANBus                          m_RioCanBus;                            // CAN bus object for the RIO
    CANBus                          m_CanivoreBus;                          // CAN bus object for the canivore

    static constexpr const std::string_view RIO_CAN_BUS_NAME = "rio";
    static constexpr const std::string_view CANIVORE_CAN_BUS_NAME = "canivore-8145";

    // GetCanBusReferenceLambda
    // Lambda to retrieve a reference to the CANBus with the specified string name.
    std::function<const CANBus&(std::string_view)> m_GetCanBusReferenceLambda = [this](std::string_view canBusName) -> const CANBus&
    {
        if (canBusName.compare(CANIVORE_CAN_BUS_NAME) == 0)
        {
            return m_CanivoreBus;
        }
        else
        {
            return m_RioCanBus;
        }
    };

    // Swerve Drive
    Pigeon2 *                       m_pPigeon;                              // CTRE Pigeon2 IMU
    SwerveDrive *                   m_pSwerveDrive;                         // Swerve drive control

    // Motors
    // (none)

    // LEDs
    EastTechLedController *         m_pLedController;                       // Interfacing to the LEDs

    // Digital I/O
    DigitalOutput *                 m_pDebugOutput;                         // Debug assist output

    // Analog I/O
    // (none)

    // PWM
    // (none)

    // Pneumatics
    Compressor *                    m_pCompressor;                          // Object to get info about the compressor

    // Solenoids
    // (none)

    // Encoders
    // (none)

    // Timers
    Timer *                         m_pMatchModeTimer;                      // Times how long a particular mode (autonomous, teleop) is running
    Timer *                         m_pRobotProgramTimer;                   // Starts at robot program entry, free runs for program life time

    // Accelerometer
    // (none)

    // Gyro
    // (none)

    // Camera
    LimelightCamera *               m_pLimelightCamera;                     // The limelight camera object
    bool                            m_bLimelightFound;                      // Indicates whether or not the limelight was found

    // LimelightDriveLambda
    // Lambda to drive the robot via swerve.
    LimelightCamera::SwerveDriveLambdaType m_LimelightDriveLambda = [this](Translation2d translation, double rotation)
    {
        m_pSwerveDrive->SetModuleStates(translation, rotation, true, true);
    };

    // Misc
    RobotMode                       m_RobotMode;                            // Keep track of the current robot state
    std::optional
    <DriverStation::Alliance>       m_AllianceColor;                        // Color reported by driver station during a match
    bool                            m_bRioPinsStable;                       // Indicates whether the RIO pin measurements (e.g. PWM) are stable
    bool                            m_bCameraAlignInProgress;               // Indicates if an automatic camera align is in progres
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
    static const int                FIELD_RELATIVE_TOGGLE_BUTTON            = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.LEFT_BUMPER;
    static const int                REZERO_SWERVE_BUTTON                    = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_BUMPER;
    static const int                LOCK_SWERVE_WHEELS_BUTTON               = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.LEFT_BUTTON;
    static const int                DRIVE_ALIGN_WITH_CAMERA_BUTTON          = DRIVE_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.RIGHT_STICK_CLICK;

    static const EastTech::Controller::PovDirections  DRIVE_CONTROLS_SWERVE_FORWARD_SLOW_POV     = EastTech::Controller::PovDirections::POV_UP;
    static const EastTech::Controller::PovDirections  DRIVE_CONTROLS_SWERVE_REVERSE_SLOW_POV     = EastTech::Controller::PovDirections::POV_DOWN;
    static const EastTech::Controller::PovDirections  DRIVE_CONTROLS_SWERVE_LEFT_OR_CCW_SLOW_POV = EastTech::Controller::PovDirections::POV_LEFT;
    static const EastTech::Controller::PovDirections  DRIVE_CONTROLS_SWERVE_RIGHT_OR_CW_SLOW_POV = EastTech::Controller::PovDirections::POV_RIGHT;


    // Aux inputs
    static const int                ESTOP_BUTTON                            = AUX_CONTROLLER_MAPPINGS->BUTTON_MAPPINGS.NO_BUTTON;

    // CAN RIO Signals
    // Note: When using swerve drive, the swerve devices are expected
    //       to be on the CANivore bus (see IDs below).  When using
    //       differential drive, check the IDs in DifferentialDrive.hpp.
    //       In general, unique CAN IDs are used regardless of CAN bus
    //       to avoid confusion.  The super structure IDs start at 31.
    // (none)

    // CANivore Signals
    // Note: IDs 11-18 are used by the swerve module motors.
    //       IDs 21-24 are used by the CANcoders.
    //       See the SwerveModuleConfigs in SwerveConfig.hpp.
    static const int                PIGEON_CAN_ID                           = 25;
    static const int                CANDLE_CAN_ID                           = 26;

    // PWM Signals
    // (none)

    // Relays
    // (none)

    // Digital I/O Signals
    static const int                SENSOR_TEST_CODE_DIO_CHANNEL            = 8;
    static const int                DEBUG_OUTPUT_DIO_CHANNEL                = 9;

    // Analog I/O Signals
    // (none)

    // Solenoid Signals
    // (none)

    // Motor speeds and angles
    // (none)

    // Misc
    const std::string               AUTO_NO_ROUTINE_STRING                  = "No autonomous routine";
    const std::string               AUTO_ROUTINE_1_STRING                   = "Autonomous routine 1";
    const std::string               AUTO_ROUTINE_2_STRING                   = "Autonomous routine 2";
    const std::string               AUTO_ROUTINE_3_STRING                   = "Autonomous routine 3";
    const std::string               AUTO_TEST_ROUTINE_STRING                = "Autonomous test routine";

    static const int                SCALE_TO_PERCENT                        = 100;
    static const unsigned           SINGLE_MOTOR                            = 1;
    static const unsigned           TWO_MOTORS                              = 2;
    static const unsigned           NUMBER_OF_LEDS                          = 0 + 8;

    static constexpr double         JOYSTICK_AXIS_INPUT_DEAD_BAND           =  0.10;
    static constexpr double         DRIVE_TRIM_UPPER_LIMIT                  =  0.05;
    static constexpr double         DRIVE_TRIM_LOWER_LIMIT                  = -0.05;
    static constexpr double         SWERVE_DRIVE_SLOW_SPEED                 =  0.10;
    static constexpr double         SWERVE_ROTATE_SLOW_SPEED                =  0.10;
};  // End class



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
