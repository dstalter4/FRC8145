////////////////////////////////////////////////////////////////////////////////
/// @file   LimelightCamera.hpp
/// @author David Stalter
///
/// @details
/// A class designed to support limelight camera functionality on the robot.
///
/// Copyright (c) 2026 East Technical High School
////////////////////////////////////////////////////////////////////////////////

#ifndef LIMELIGHTCAMERA_HPP
#define LIMELIGHTCAMERA_HPP

// SYSTEM INCLUDES
#include <string>                                   // for std::string

// C INCLUDES
#include "frc/DriverStation.h"                      // for interacting with the driver station
#include "frc/controller/PIDController.h"           // for utilizing WPIlib PID controller
#include "networktables/NetworkTable.h"             // for network tables
#include "networktables/NetworkTableInstance.h"     // for network table instance
#include "units/angle.h"                            // for degree user defined literal

// C++ INCLUDES
#include "RobotUtils.hpp"                           // for DisplayFormattedMessage()

using namespace frc;


////////////////////////////////////////////////////////////////
/// @class LimelightCamera
///
/// Class that provides methods for interacting with limelight
/// cameras.
///
////////////////////////////////////////////////////////////////
class LimelightCamera
{
public:

    typedef std::function<void(Translation2d translation, double rotation)> SwerveDriveLambdaType;

    enum class LimelightModel : uint32_t
    {
        LIMELIGHT_1,
        LIMELIGHT_2,
        LIMELIGHT_2_PLUS,
        LIMELIGHT_3,
        LIMELIGHT_3G,
        LIMELIGHT_3A,
        LIMELIGHT_4,
    };

    enum class TaggedFieldElement
    {
        // This needs to be updated each season to list
        // the field elements with April tags.
        ELEMENT_NONE
    };

    enum class LimelightMode
    {
        // Values taken from the limelight documentation
        VISION_PROCESSOR    = 0,
        DRIVER_CAMERA       = 1
    };

    enum class LimelightLedMode
    {
        // Values taken from the limelight documentation
        PIPELINE            = 0,
        ARRAY_OFF           = 1,
        ARRAY_BLINK         = 2,
        ARRAY_ON            = 3
    };

    // Constructor
    LimelightCamera(LimelightModel limelightModel, std::string cameraName = "limelight");

    // Locates the network table and sets it for use by the object
    inline bool FindAndSetNetworkTable();

    // Set the limelight mode
    inline void SetPipeline(int32_t pipelineNum);

    // Set the limelight mode
    inline void SetMode(LimelightMode mode);

    // Set the state of the limelight LED array
    inline void SetLedMode(LimelightLedMode ledMode);

    // Triggers a limelight rewind capture
    void TriggerRewindCapture(units::time::second_t numberOfSeconds);

    // Set the limelight mode
    void SetPriorityId(TaggedFieldElement fieldElement, DriverStation::Alliance alliance);

    // Use the limelight to align to a target
    void AlignToTargetSwerve(SwerveDriveLambdaType swerveDriveLambda, units::angle::degree_t currentYawDegrees);

    // Display some status information
    void UpdateSmartDashboard();

private:

    struct LimelightConfig
    {
        const LimelightModel LIMELIGHT_MODEL;
        const double HORIZONTAL_FOV_DEGREES;
        const double VERTICAL_FOV_DEGREES;
    };

    static constexpr LimelightConfig LIMELIGHT_CONFIGS[] =
    {
        {LimelightModel::LIMELIGHT_1,       0.0, 0.0},
        {LimelightModel::LIMELIGHT_2,       62.5, 48.9},
        {LimelightModel::LIMELIGHT_2_PLUS,  62.5, 48.9},
        {LimelightModel::LIMELIGHT_3,       62.5, 48.9},
        {LimelightModel::LIMELIGHT_3G,      82.0, 56.2},
        {LimelightModel::LIMELIGHT_3A,      54.5, 42.0},
        {LimelightModel::LIMELIGHT_4,       82.0, 56.2}
    };

    // Enable port forwarding for the limelight
    void EnableLimelightPortForwarding(bool bEnableEthernetForwarding, bool bEnableUsb0Forwarding, bool bEnableUsb1Forwarding);

    // Member variables
    std::string                         m_CameraName;                           // Name of the limelight camera
    std::shared_ptr<nt::NetworkTable>   m_pCameraNetworkTable;                  // Network table for the limelight camera
    PIDController                       m_VisionStrafePid;                      // PID controller for strafe control during automated vision targeting
    PIDController                       m_VisionRotatePid;                      // PID controller for rotation control during automated vision targeting
    int                                 m_TargetAprilTagId;                     // Track which AprilTag to target
    const LimelightConfig               CAMERA_CONFIG;                          // The configuration for this camera instance

    static constexpr double DEFAULT_PID_CONTROLLER_P_VALUE = 0.025;
    static constexpr double DEFAULT_PID_CONTROLLER_I_VALUE = 0.02;
    static constexpr double DEFAULT_PID_CONTROLLER_D_VALUE = 0.002;
};



////////////////////////////////////////////////////////////////
/// @method LimelightCamera::FindAndSetNetworkTable
///
/// This method tries to find the network table for a limelight
/// and will set the member variable.  The robot object might
/// be constructed before the network table for the limelight is
/// available.  This will wait for a certain amount of time to
/// try and locate it.  If the time expires, it assumed no
/// limelight is present and any subsequent call to try and use
/// functionality relying on the network table will fail.  The
/// function returns whether or not a network table was found.
///
////////////////////////////////////////////////////////////////
inline bool LimelightCamera::FindAndSetNetworkTable()
{
    static const uint32_t NETWORK_TABLE_SEARCH_STEP_TIME_MS = 100U;
    static const uint32_t NETWORK_TABLE_MAX_SEARCH_TIME_MS = 5000U;

    bool bTableFound = false;
    for (uint32_t searchTimeMs = 0U; searchTimeMs <= NETWORK_TABLE_MAX_SEARCH_TIME_MS; searchTimeMs += NETWORK_TABLE_MAX_SEARCH_TIME_MS)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(NETWORK_TABLE_SEARCH_STEP_TIME_MS));
        m_pCameraNetworkTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

        if (m_pCameraNetworkTable.get() != nullptr)
        {
            RobotUtils::DisplayFormattedMessage("Limelight network table found after %d milliseconds.", searchTimeMs);
            bTableFound = true;
            break;
        }
    }

    // Enable this here instead of the constructor in case the
    // network table wasn't available when constructors ran.
    m_pCameraNetworkTable->PutNumber("rewind_enable_set", 1);

    return bTableFound;
}



////////////////////////////////////////////////////////////////
/// @method LimelightCamera::SetPipeline
///
/// This method sets the pipeline used by the limelight camera.
///
////////////////////////////////////////////////////////////////
inline void LimelightCamera::SetPipeline(int32_t pipelineNum)
{
    if (m_pCameraNetworkTable.get() != nullptr)
    {
        m_pCameraNetworkTable->PutNumber("pipeline", pipelineNum);
    }
    else
    {
        RobotUtils::DisplayFormattedMessage("Limelight network table unavailble.  Pipeline %d not set!\n", pipelineNum);
    }
}



////////////////////////////////////////////////////////////////
/// @method LimelightCamera::SetMode
///
/// This method sets the mode of the limelight camera.
///
////////////////////////////////////////////////////////////////
inline void LimelightCamera::SetMode(LimelightMode mode)
{
    if (m_pCameraNetworkTable.get() != nullptr)
    {
        m_pCameraNetworkTable->PutNumber("camMode", static_cast<int>(mode));
    }
    else
    {
        RobotUtils::DisplayFormattedMessage("Limelight network table unavailble.  Camera mode %d not set!\n", static_cast<int>(mode));
    }
}



////////////////////////////////////////////////////////////////
/// @method LimelightCamera::SetLedMode
///
/// This method sets the mode of the limelight camera.
///
////////////////////////////////////////////////////////////////
inline void LimelightCamera::SetLedMode(LimelightLedMode ledMode)
{
    if (m_pCameraNetworkTable.get() != nullptr)
    {
        m_pCameraNetworkTable->PutNumber("ledMode", static_cast<int>(ledMode));
    }
    else
    {
        RobotUtils::DisplayFormattedMessage("Limelight network table unavailble.  Led mode %d not set!\n", static_cast<int>(ledMode));
    }
}

#endif // LIMELIGHTCAMERA_HPP
