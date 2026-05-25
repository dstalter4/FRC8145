////////////////////////////////////////////////////////////////////////////////
/// @file   EastTechUsbCamera.hpp
/// @author David Stalter
///
/// @details
/// A class designed to support USB camera functionality on the robot.
///
/// Copyright (c) 2024 East Technical High School
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
#include <thread>                               // for thread management

// C INCLUDES
// (none)

// C++ INCLUDES
#include "EastTechUsbCamera.hpp"                // for class declaration
#include "RobotUtils.hpp"                       // for DisplayMessage(), DisplayFormattedMessage()

// STATIC MEMBER DATA
EastTechUsbCamera::UsbCameraStorage                 EastTechUsbCamera::m_UsbCameras;
EastTechUsbCamera::UsbCameraInfo *                  EastTechUsbCamera::m_pCurrentUsbCamera;
cs::CvSource                                        EastTechUsbCamera::m_CameraOutput;
int                                                 EastTechUsbCamera::m_NumUsbCamerasPresent;
bool                                                EastTechUsbCamera::m_bDoFullProcessing;
unsigned                                            EastTechUsbCamera::m_CameraHeartBeat;
const char *                                        EastTechUsbCamera::CAMERA_OUTPUT_NAME = "Camera Output";

cv::Mat                                             EastTechUsbCamera::m_SourceMat;
cv::Mat                                             EastTechUsbCamera::m_ResizeOutputMat;
cv::Mat                                             EastTechUsbCamera::m_HsvThresholdOutputMat; 
cv::Mat                                             EastTechUsbCamera::m_ErodeOutputMat;
cv::Mat                                             EastTechUsbCamera::m_ContoursMat;
cv::Mat                                             EastTechUsbCamera::m_FilteredContoursMat;
cv::Mat                                             EastTechUsbCamera::m_VisionTargetMat;
cv::Mat *                                           EastTechUsbCamera::m_pDashboardMat;

std::vector<std::vector<cv::Point>>                 EastTechUsbCamera::m_Contours;
std::vector<std::vector<cv::Point>>                 EastTechUsbCamera::m_FilteredContours;
std::vector<EastTechUsbCamera::VisionTargetReport>  EastTechUsbCamera::m_ContourTargetReports;
EastTechUsbCamera::VisionTargetReport               EastTechUsbCamera::m_VisionTargetReport;

Timer                                               EastTechUsbCamera::AutonomousCamera::m_AutoCameraTimer;
double                                              EastTechUsbCamera::AutonomousCamera::m_IntegralSum;


////////////////////////////////////////////////////////////////
/// @method EastTechUsbCamera::AutonomousCamera::AlignToTarget
///
/// This method tries to automatically align the robot to a
/// target based on feedback from the camera.  It will execute
/// until it finds the target or a safety timer expires.
///
////////////////////////////////////////////////////////////////
bool EastTechUsbCamera::AutonomousCamera::AlignToTarget(SeekDirection seekDirection, const bool bEnableMotors)
{
    bool bTargetFound = false;
    m_AutoCameraTimer.Start();

    while (!bTargetFound && (m_AutoCameraTimer.Get() < MAX_CAMERA_SEARCH_TIME_S))
    {
        // These values need to be generated or come from the camera
        double targetX = 0.0;
        bool bTargetValid = false;

        // These are computed to send to the drivetrain.  This has
        // only ever been applied to differential drive, never swerve.
        double steeringAdjust = 0.0;
        double leftCommand = 0.0;
        double rightCommand = 0.0;

        if (!bTargetValid)
        {
            constexpr double STARTING_SEEK_VALUE = 0.2;
            
            // No target - rotate to find target
            if (seekDirection == SEEK_LEFT)
            {
                steeringAdjust = STARTING_SEEK_VALUE;
            }
            else if (seekDirection == SEEK_RIGHT)
            {
                steeringAdjust = -STARTING_SEEK_VALUE;
            }
            else
            {
            }
            
            m_IntegralSum = 0.0;
        }
        else
        {
            // We do see the target, execute aiming code
            double headingError = targetX;

            // (heading error + last heading error);
            m_IntegralSum += headingError;
            m_IntegralSum = RobotUtils::Limit(m_IntegralSum, INTEGRAL_SUM_LIMIT_VALUE, -INTEGRAL_SUM_LIMIT_VALUE);

            // Proportional-Integral controller
            steeringAdjust = (KP * targetX) + (KI * m_IntegralSum);

            // Remember to limit the integration term
            // Reset the integration term when target is out of the frame
        }

        // The left and right commands both use addition here because
        // the scaling constants will correct the direction.
        leftCommand -= steeringAdjust;
        rightCommand += steeringAdjust;
        
        // Make sure we don't spin the motors too fast
        leftCommand = RobotUtils::Limit(leftCommand, MAX_SEEK_MOTOR_SPEED, -MAX_SEEK_MOTOR_SPEED);
        rightCommand = RobotUtils::Limit(rightCommand, MAX_SEEK_MOTOR_SPEED, -MAX_SEEK_MOTOR_SPEED);

        // Need a way to know when the target is found
        if (bTargetValid && (leftCommand == 0.0) && (rightCommand == 0.0))
        {
            bTargetFound = true;
            break;
        }

        // Set motor speed
        if (bEnableMotors)
        {
            // Steer the robot.  This needs a lambda to the motor control.
        }

        // Send useful information to smart dashboard.
        SmartDashboard::PutNumber("Steering adjust", steeringAdjust);
        SmartDashboard::PutNumber("targetX", targetX);
        SmartDashboard::PutNumber("Integral sum", m_IntegralSum);
        SmartDashboard::PutNumber("Target valid", bTargetValid);
    }

    // Clean up the timer
    m_AutoCameraTimer.Stop();
    m_AutoCameraTimer.Reset();

    return bTargetFound;
}



////////////////////////////////////////////////////////////////
/// @method EastTechUsbCamera::UsbCameraInfo::UsbCameraInfo
///
/// Constructor for a UsbCameraInfo object.
///
////////////////////////////////////////////////////////////////
EastTechUsbCamera::UsbCameraInfo::UsbCameraInfo(const CameraType camType, int devNum, const int xRes, const int yRes, const int fps) :
    m_UsbCam(CameraServer::StartAutomaticCapture()),
    m_CamSink(CameraServer::GetVideo(m_UsbCam)),
    m_bIsPresent(true),
    m_DeviceNum(devNum),
    CAM_TYPE(camType),
    X_RESOLUTION(xRes),
    Y_RESOLUTION(yRes),
    FPS(fps)
{
    RobotUtils::DisplayFormattedMessage("Creating camera %d.\n", devNum);

    // Start image capture, set the resolution and connect the sink
    m_UsbCam.SetResolution(xRes, yRes);
    m_UsbCam.SetFPS(fps);
}



////////////////////////////////////////////////////////////////
/// @method EastTechUsbCamera::CreateConfiguredCameras
///
/// This method creates camera objects for configured cameras.
/// It utilizes the static storage buffer in the class and
/// placement new to properly construct the objects.
///
////////////////////////////////////////////////////////////////
bool EastTechUsbCamera::CreateConfiguredCameras()
{
    bool bAnyCameraPresent = false;

    if (FRONT_USB_CAMERA_SUPPORTED)
    {
        // Placement new - storage is statically allocated
        (void) new (&m_UsbCameras.m_CamerasInfo[FRONT_USB]) UsbCameraInfo(FRONT_USB, m_NumUsbCamerasPresent++);
        bAnyCameraPresent = true;
    }

    if (BACK_USB_CAMERA_SUPPORTED)
    {
        // Placement new - storage is statically allocated
        (void) new (&m_UsbCameras.m_CamerasInfo[BACK_USB]) UsbCameraInfo(BACK_USB, m_NumUsbCamerasPresent++);
        bAnyCameraPresent = true;
    }

    return bAnyCameraPresent;
}



////////////////////////////////////////////////////////////////
/// @method EastTechUsbCamera::VisionThread
///
/// This method contains the workflow of the main vision thread.
///
////////////////////////////////////////////////////////////////
void EastTechUsbCamera::UsbVisionThread()
{
    // Indicate the thread has been started
    RobotUtils::DisplayMessage("Vision thread detached.");

    cs::UsbCamera camera = CameraServer::StartAutomaticCapture();
    camera.SetResolution(UsbCameraInfo::DEFAULT_X_RESOLUTION, UsbCameraInfo::DEFAULT_Y_RESOLUTION);
    camera.SetFPS(UsbCameraInfo::DEFAULT_FPS);

    while (true)
    {
        // Be sure to relinquish the CPU when done
        std::this_thread::sleep_for(std::chrono::milliseconds(CAMERA_THREAD_SLEEP_TIME_MS));
    }
    
    /*
    // Me
    cs::UsbCamera usbCam = CameraServer::StartAutomaticCapture();
    usbCam.SetResolution(640, 480);
    usbCam.SetFPS(30);
    cs::CvSink usbSink = CameraServer::GetVideo(usbCam);
    cs::CvSource outputSource = CameraServer::PutVideo(CAMERA_OUTPUT_NAME, 640, 480);
    while (true)
    {
        //usbSink.GrabFrame(m_SourceMat);
        //outputSource.PutFrame(m_SourceMat);
    }
    */
    
    /*
    // Sample WPI code
    cs::UsbCamera camera = CameraServer::StartAutomaticCapture();
    camera.SetResolution(160, 120);
    camera.SetFPS(30);
    cs::CvSink cvSink = CameraServer::GetVideo();
    cs::CvSource outputStreamStd = CameraServer::PutVideo("Camera Output", 160, 120);    // "Gray"
    //cv::Mat source;
    //cv::Mat output;
    //cv::Mat frame;
    while(true) {
        //cvSink.GrabFrame(frame);//source);
        //cvtColor(source, output, cv::COLOR_BGR2GRAY);
        //outputStreamStd.PutFrame(frame);//output);
    }
    */

   // The robot program should never reach this point.
   // Code below here is old and untested in recent years.
   ASSERT(false);
    
    // Clear the memory used for the camera storage
    std::memset(reinterpret_cast<void*>(&m_UsbCameras), 0U, sizeof(UsbCameraStorage));
    
    // Clear the vision target structure
    std::memset(&m_VisionTargetReport, 0U, sizeof(VisionTargetReport));
    
    // Create the configured camera objects (the buffer was cleared during static initialization)
    bool bAnyCameraPresent = CreateConfiguredCameras();
    
    // If there were no properly constructed cameras, just loop indefinitely
    if (!bAnyCameraPresent)
    {
        while (true)
        {
            // Do nothing
        }
    }
    
    // Set the default selected camera
    m_pCurrentUsbCamera = &m_UsbCameras.m_CamerasInfo[FRONT_USB];
    
    // Connect the output
    m_CameraOutput = CameraServer::PutVideo(CAMERA_OUTPUT_NAME, m_pCurrentUsbCamera->X_RESOLUTION, m_pCurrentUsbCamera->Y_RESOLUTION);
    
    // Set the default image to display
    m_pDashboardMat = &m_SourceMat;
    SmartDashboard::PutString("Camera Output", "Default");
    
    // Default to not doing full processing unless the robot code says otherwise
    m_bDoFullProcessing = false;

    while (true)
    {
        // Don't call this in production code - it hogs resources
        UpdateSmartDashboard();
        
        // First, acquire an image from the currently selected camera
        int grabFrameResult = 0;
        grabFrameResult = m_pCurrentUsbCamera->m_CamSink.GrabFrame(m_SourceMat);
        
        // Make sure it was successful before doing more processing
        if (grabFrameResult == 0)
        {
            continue;
        }
        
        if (m_bDoFullProcessing)
        {
            // Filter the image
            ProcessImage();
            
            // Try and identify the reflective tape
            FindReflectiveTapeTarget();
            
            // Calculate some info based on the reflective tape
            CalculateReflectiveTapeValues();
        
        }
        
        // Display the image to the dashboard
        m_CameraOutput.PutFrame(*m_pDashboardMat);
    }
}



////////////////////////////////////////////////////////////////
/// @method EastTechUsbCamera::UpdateSmartDashboard
///
/// This method sends new data to the C++ Smart Dashboard.
///
////////////////////////////////////////////////////////////////
void EastTechUsbCamera::UpdateSmartDashboard()
{
    SmartDashboard::PutNumber("Camera HeartBeat",           m_CameraHeartBeat++);
    
    SmartDashboard::PutNumber("Bounding rect X",            m_VisionTargetReport.m_BoundingRectX);
    SmartDashboard::PutNumber("Bounding rect Y",            m_VisionTargetReport.m_BoundingRectY);
    SmartDashboard::PutNumber("Bounding rect width",        m_VisionTargetReport.m_BoundingRectWidth);
    SmartDashboard::PutNumber("Bounding rect height",       m_VisionTargetReport.m_BoundingRectHeight);
    SmartDashboard::PutNumber("Bounding rect area",         m_VisionTargetReport.m_BoundingRectArea);
    SmartDashboard::PutNumber("Bounding rect aspect ratio", m_VisionTargetReport.m_BoundingRectAspectRatio);
    
    SmartDashboard::PutNumber("Contour area",               m_VisionTargetReport.m_Area);
    SmartDashboard::PutNumber("Contour perimeter",          m_VisionTargetReport.m_Perimeter);
    SmartDashboard::PutNumber("Contour convex hull area",   m_VisionTargetReport.m_ConvexHullArea);
    SmartDashboard::PutNumber("Contour solidity",           m_VisionTargetReport.m_Solidity);
    SmartDashboard::PutNumber("Contour vertices",           m_VisionTargetReport.m_Vertices);
    
    SmartDashboard::PutNumber("Area %",                     m_VisionTargetReport.m_PercentAreaToImageArea);
    SmartDashboard::PutNumber("Trapezoid %",                m_VisionTargetReport.m_TrapezoidPercent);
    SmartDashboard::PutNumber("Camera distance, X",         m_VisionTargetReport.m_CameraDistanceX);
    SmartDashboard::PutNumber("Camera distance, Y",         m_VisionTargetReport.m_CameraDistanceY);
    SmartDashboard::PutNumber("Ground distance",            m_VisionTargetReport.m_GroundDistance);
    SmartDashboard::PutNumber("Target in range",            m_VisionTargetReport.m_bTargetInRange);
    SmartDashboard::PutNumber("Target report valid",        m_VisionTargetReport.m_bIsValid);
}



////////////////////////////////////////////////////////////////
/// @method EastTechUsbCamera::ToggleCameraProcessedImage
///
/// Updates which stage of the image processing is sent to the
/// dashboard.
///
////////////////////////////////////////////////////////////////
void EastTechUsbCamera::ToggleCameraProcessedImage()
{
    if (m_bDoFullProcessing)
    {
        if (m_pDashboardMat == &m_SourceMat)
        {
            // Move on to HSV threshold output
            m_pDashboardMat = &m_HsvThresholdOutputMat;
            SmartDashboard::PutString("Camera Output", "HSV Threshold");
        }
        else if (m_pDashboardMat == &m_HsvThresholdOutputMat)
        {
            // Move on to eroded output
            m_pDashboardMat = &m_ErodeOutputMat;
            SmartDashboard::PutString("Camera Output", "Eroded");
        }
        else if (m_pDashboardMat == &m_ErodeOutputMat)
        {
            // Move on to contours output
            m_pDashboardMat = &m_ContoursMat;
            SmartDashboard::PutString("Camera Output", "Contours");
        }
        else if (m_pDashboardMat == &m_ContoursMat)
        {
            // Move on to filtered contours output
            m_pDashboardMat = &m_FilteredContoursMat;
            SmartDashboard::PutString("Camera Output", "Filtered Contours");
        }
        else if (m_pDashboardMat == &m_FilteredContoursMat)
        {
            // Move on to the best candidate vision target mat
            m_pDashboardMat = &m_VisionTargetMat;
            SmartDashboard::PutString("Camera Output", "Vision Target");
        }
        else if (m_pDashboardMat == &m_VisionTargetMat)
        {
            // Back to the start
            m_pDashboardMat = &m_SourceMat;
            SmartDashboard::PutString("Camera Output", "Default");
        }
        else
        {
        }
    }
    else
    {
        // Default to just the typical source mat
        m_pDashboardMat = &m_SourceMat;
        SmartDashboard::PutString("Camera Output", "Default");
    }
}

    
    
////////////////////////////////////////////////////////////////
/// @method EastTechUsbCamera::ProcessImage
///
/// Processes an image from the camera to try and identify
/// vision targets.
///
////////////////////////////////////////////////////////////////
void EastTechUsbCamera::ProcessImage()
{
    FilterImageHsv();
    ErodeImage();
    FindContours();
    FilterContours();
}

    
    
////////////////////////////////////////////////////////////////
/// @method EastTechUsbCamera::FilterImageHsv
///
/// Applies a color filter to the image based on Hue, Saturation
/// and Value.
///
////////////////////////////////////////////////////////////////
void EastTechUsbCamera::FilterImageHsv()
{
    // min/max values
    static double hsvThresholdHue[] = {0.0, 180.0};
    static double hsvThresholdSaturation[] = {0.0, 150.0};
    static double hsvThresholdValue[] = {220.0, 255.0};
    
    hsvThresholdHue[0]          = SmartDashboard::GetNumber("H min", hsvThresholdHue[0]);
    hsvThresholdHue[1]          = SmartDashboard::GetNumber("H max", hsvThresholdHue[1]);
    hsvThresholdSaturation[0]   = SmartDashboard::GetNumber("S min", hsvThresholdSaturation[0]);
    hsvThresholdSaturation[1]   = SmartDashboard::GetNumber("S max", hsvThresholdSaturation[1]);
    hsvThresholdValue[0]        = SmartDashboard::GetNumber("V min", hsvThresholdValue[0]);
    hsvThresholdValue[1]        = SmartDashboard::GetNumber("V max", hsvThresholdValue[1]);
    
    SmartDashboard::PutNumber("H min", hsvThresholdHue[0]);
    SmartDashboard::PutNumber("H max", hsvThresholdHue[1]);
    SmartDashboard::PutNumber("S min", hsvThresholdSaturation[0]);
    SmartDashboard::PutNumber("S max", hsvThresholdSaturation[1]);
    SmartDashboard::PutNumber("V min", hsvThresholdValue[0]);
    SmartDashboard::PutNumber("V max", hsvThresholdValue[1]);
    
    // Convert to HSV and filter
    cv::cvtColor(m_SourceMat, m_HsvThresholdOutputMat, cv::COLOR_BGR2HSV);
    cv::inRange(m_HsvThresholdOutputMat,
                cv::Scalar(hsvThresholdHue[0], hsvThresholdSaturation[0], hsvThresholdValue[0]),
                cv::Scalar(hsvThresholdHue[1], hsvThresholdSaturation[1], hsvThresholdValue[1]),
                m_HsvThresholdOutputMat);
}

    
    
////////////////////////////////////////////////////////////////
/// @method EastTechUsbCamera::ErodeImage
///
/// Erodes an image.
///
////////////////////////////////////////////////////////////////
void EastTechUsbCamera::ErodeImage()
{
    // Erode image
    // @param src input image
    // @param dst output image
    // @param kernel structuring element used for erosion
    // @param anchor position of the anchor within the element; default value (-1, -1) means that the anchor is at the element center.
    // @param iterations number of times erosion is applied.
    // @param borderType pixel extrapolation method, see cv::BorderTypes
    // @param borderValue border value in case of a constant border
    cv::Mat cvErodeKernel;
    cv::erode(m_HsvThresholdOutputMat, m_ErodeOutputMat, cvErodeKernel, cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, cv::Scalar(-1));
}

    
    
////////////////////////////////////////////////////////////////
/// @method EastTechUsbCamera::FindContours
///
/// Finds the contours in an image.
///
////////////////////////////////////////////////////////////////
void EastTechUsbCamera::FindContours()
{    
    // Find contours
    // @param image Source, an 8-bit single-channel image.
    // @param contours Detected contours. Each contour is stored as a vector of points (e.g. std::vector<std::vector<cv::Point> >).
    // @param hierarchy Optional output vector (e.g. std::vector<cv::Vec4i>), containing information about the image topology.
    // @param mode Contour retrieval mode, see cv::RetrievalModes ( ? cv::RETR_EXTERNAL : cv::RETR_LIST)
    // @param method Contour approximation method, see cv::ContourApproximationModes
    // @param offset Optional offset by which every contour point is shifted.
    std::vector<cv::Vec4i> hierarchy;
    m_Contours.clear();
    cv::findContours(m_ErodeOutputMat, m_Contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    
    // Reset the contour mats by clearing them
    m_ContoursMat = cv::Mat::zeros(m_pDashboardMat->size(), m_pDashboardMat->type());
    m_FilteredContoursMat = cv::Mat::zeros(m_pDashboardMat->size(), m_pDashboardMat->type());
    m_VisionTargetMat = cv::Mat::zeros(m_pDashboardMat->size(), m_pDashboardMat->type());
    
    // @param image Destination image.
    // @param contours All the input contours. Each contour is stored as a point vector.
    // @param contourIdx Parameter indicating a contour to draw. If it is negative, all the contours are drawn.
    // @param color Color of the contours.
    cv::drawContours(m_ContoursMat, m_Contours, -1, cv::Scalar(255, 255, 255));
}

    
    
////////////////////////////////////////////////////////////////
/// @method EastTechUsbCamera::FilterContours
///
/// Filters the contours found by certain criteria.
///
////////////////////////////////////////////////////////////////
void EastTechUsbCamera::FilterContours()
{    
    const double FILTER_CONTOURS_MIN_WIDTH      = 0.0;
    const double FILTER_CONTOURS_MAX_WIDTH      = 1000.0;
    const double FILTER_CONTOURS_MIN_HEIGHT     = 0.0;
    const double FILTER_CONTOURS_MAX_HEIGHT     = 1000.0;
    const double FILTER_CONTOURS_MIN_AREA       = 500.0;
    const double FILTER_CONTOURS_MAX_AREA       = 100000.0;
    const double FILTER_CONTOURS_MIN_PERIMETER  = 0.0;
    const double FILTER_CONTOURS_MAX_PERIMETER  = 10000.0;
    const double FILTER_CONTOURS_SOLIDITY[]     = {85.0, 100.0};
    const double FILTER_CONTOURS_MIN_VERTICES   = 0.0;
    const double FILTER_CONTOURS_MAX_VERTICES   = 1000000.0;
    const double FILTER_CONTOURS_MIN_RATIO      = 0.0;
    const double FILTER_CONTOURS_MAX_RATIO      = 1000.0;
    
    // Filter contours
    m_FilteredContours.clear();
    m_ContourTargetReports.clear();
    for (std::vector<cv::Point> contour : m_Contours)
    {
        VisionTargetReport currentContourReport;
        
        // Bounding rectangle filtering
        cv::Rect boundingRectangle = cv::boundingRect(contour);
        if ((boundingRectangle.width) < FILTER_CONTOURS_MIN_WIDTH || (boundingRectangle.width > FILTER_CONTOURS_MAX_WIDTH))
        {
            continue;
        }
        if ((boundingRectangle.height < FILTER_CONTOURS_MIN_HEIGHT) || (boundingRectangle.height > FILTER_CONTOURS_MAX_HEIGHT))
        {
            continue;
        }
        // Fill out bounding rectangle info (done here so the loop can continue if criteria aren't met)
        currentContourReport.m_BoundingRectX = boundingRectangle.x;
        currentContourReport.m_BoundingRectY = boundingRectangle.y;
        currentContourReport.m_BoundingRectWidth = boundingRectangle.width;
        currentContourReport.m_BoundingRectHeight = boundingRectangle.height;
        currentContourReport.m_BoundingRectArea = boundingRectangle.width * boundingRectangle.height;
        
        // Max area is not a standard filtering technique in GRIP
        double area = cv::contourArea(contour);
        if ((area < FILTER_CONTOURS_MIN_AREA) || (area > FILTER_CONTOURS_MAX_AREA))
        {
            continue;
        }
        currentContourReport.m_Area = area;
        
        // Max perimeter is not a standard filtering technique in GRIP
        double perimeter = cv::arcLength(contour, true);
        if ((perimeter < FILTER_CONTOURS_MIN_PERIMETER) || (perimeter > FILTER_CONTOURS_MAX_PERIMETER))
        {
            continue;
        }
        currentContourReport.m_Perimeter = perimeter;
        
        std::vector<cv::Point> hull;
        cv::convexHull(cv::Mat(contour, true), hull);
        double hullArea = cv::contourArea(hull);
        double solidity = 100.0 * (area / hullArea);
        if ((solidity < FILTER_CONTOURS_SOLIDITY[0]) || (solidity > FILTER_CONTOURS_SOLIDITY[1]))
        {
            continue;
        }
        currentContourReport.m_ConvexHullArea = hullArea;
        currentContourReport.m_Solidity = solidity;
        
        // Number of vertices
        if ((contour.size() < FILTER_CONTOURS_MIN_VERTICES) || (contour.size() > FILTER_CONTOURS_MAX_VERTICES))
        {
            continue;
        }
        currentContourReport.m_Vertices = contour.size();
        
        // Aspect ratio
        double ratio = static_cast<double>(boundingRectangle.width) / static_cast<double>(boundingRectangle.height);
        if ((ratio < FILTER_CONTOURS_MIN_RATIO) || (ratio > FILTER_CONTOURS_MAX_RATIO))
        {
            continue;
        }
        currentContourReport.m_BoundingRectAspectRatio = boundingRectangle.width / boundingRectangle.height;
        
        // All criteria passed, add this contour
        currentContourReport.m_bIsValid = true;
        m_FilteredContours.push_back(contour);
        m_ContourTargetReports.push_back(currentContourReport);
    }
    
    // @param image Destination image.
    // @param contours All the input contours. Each contour is stored as a point vector.
    // @param contourIdx Parameter indicating a contour to draw. If it is negative, all the contours are drawn.
    // @param color Color of the contours.
    cv::drawContours(m_FilteredContoursMat, m_FilteredContours, -1, cv::Scalar(255, 255, 255));
}



////////////////////////////////////////////////////////////////
/// @method EastTechUsbCamera::FindReflectiveTapeTarget
///
/// This method iterates over the filtered contours and tries to
/// identify the reflective tape target.  It will save off the
/// appropriate contour if one that meets the criteria is found.
///
////////////////////////////////////////////////////////////////
void EastTechUsbCamera::FindReflectiveTapeTarget()
{
    if (m_ContourTargetReports.size() > 0)
    {
        int index = 0;
        int candidateIndex = 0;
        double currentMaxArea = 0.0;

        // Iterate through the contour reports, searching for the best candidate
        for (VisionTargetReport report : m_ContourTargetReports)
        {
            if (report.m_Area > currentMaxArea)
            {
                currentMaxArea = report.m_Area;
                m_VisionTargetReport = report;
                candidateIndex = index;
            }
            
            index++;
        }
        
        // Draw the candidate contour
        cv::drawContours(m_VisionTargetMat, m_FilteredContours, candidateIndex, cv::Scalar(255, 255, 255));
    }
    else
    {
        // If no contour met criteria, clear out the target report information
        std::memset(&m_VisionTargetReport, 0, sizeof(VisionTargetReport));
    }
}



////////////////////////////////////////////////////////////////
/// @method EastTechUsbCamera::CalculateReflectiveTapeValues
///
/// This method performs certain calculations on the best found
/// contour.  It will primarily compute distances related to the
/// vision target.
///
////////////////////////////////////////////////////////////////
void EastTechUsbCamera::CalculateReflectiveTapeValues()
{
    // If there is no vision target report available, don't proceed
    if (!m_VisionTargetReport.m_bIsValid)
    {
        return;
    }

    static constexpr double TARGET_WIDTH_INCHES             =  2.0;
    static constexpr double TARGET_HEIGHT_INCHES            = 16.0;
    static constexpr double TARGET_HEIGHT_FROM_GROUND       =  2.0;
    //static constexpr double TARGET_MIN_AREA_PERCENT         = 0.0;
    //static constexpr double TARGET_MAX_AREA_PERCENT         = 100.0;
    //static constexpr double TARGET_RANGE_MIN                = 132.0;
    //static constexpr double TARGET_RANGE_MAX                = 192.0;
    //static constexpr double GROUND_DISTANCE_TOLERANCE       = 6.0;
    static constexpr double CAMERA_FOV_DEGREES              = 50.0;
    //static constexpr double CAMERA_DIAGONAL_FOV_DEGREES     = 78.0;
    //static constexpr double CALIBRATED_CAMERA_ANGLE         = 21.5778173;
    static constexpr double DEGREES_TO_RADIANS              = M_PI / 180.0;
    //static constexpr double DECIMAL_TO_PERCENT              = 100.0;

    // d = (TargetWidthIn * CAMERA_X_RES) / (2 * TargetWidthPix * tan(1/2 * FOVAng))
    // d = (TargetHeightIn * CAMERA_Y_RES) / (2 * TargetHeightPix * tan(1/2 * FOVAng))
    m_VisionTargetReport.m_CameraDistanceX = (TARGET_WIDTH_INCHES * m_pCurrentUsbCamera->X_RESOLUTION) /
                                             (2.0 * (m_VisionTargetReport.m_BoundingRectWidth) * tan(.5 * CAMERA_FOV_DEGREES * DEGREES_TO_RADIANS));
                                             //(2.0 * (m_VisionTargetReport.m_BoundingRectWidth) * tan(.5 * CALIBRATED_CAMERA_ANGLE * DEGREES_TO_RADIANS));
    
    m_VisionTargetReport.m_CameraDistanceY = (TARGET_HEIGHT_INCHES * m_pCurrentUsbCamera->Y_RESOLUTION) /
                                             (2.0 * (m_VisionTargetReport.m_BoundingRectHeight) * tan(.5 * CAMERA_FOV_DEGREES * DEGREES_TO_RADIANS));
                                             //(2.0 * (m_VisionTargetReport.m_BoundingRectHeight) * tan(.5 * CALIBRATED_CAMERA_ANGLE * DEGREES_TO_RADIANS));

    // ground_distance = sqrt((camera_reported_distance^2) - (84^2))
    // sin(camera_angle) = (height_from_ground) / (camera_reported_distance);
    // Use m_CameraDistanceY since the target is taller than wide
    m_VisionTargetReport.m_GroundDistance = sqrt((m_VisionTargetReport.m_CameraDistanceY * m_VisionTargetReport.m_CameraDistanceY) - (TARGET_HEIGHT_FROM_GROUND * TARGET_HEIGHT_FROM_GROUND));

    //m_VisionTargetReport.m_PercentAreaToImageArea = ( ? / m_VisionTargetReport.m_BoundingRectArea) * DECIMAL_TO_PERCENT;
    //m_VisionTargetReport.m_TrapezoidPercent = (m_TargetReport.m_ConvexHullArea / m_VisionTargetReport.m_BoundingRectArea) * DECIMAL_TO_PERCENT;

    // At a distance of 20 feet, the minimum area for the target is about 700 pxl^2
    // Our target range is 11-16 ft. so we will use this as our starting filtering point
    /*
    if (((m_VisionTargetReport.m_GroundDistance + GROUND_DISTANCE_TOLERANCE) >= TARGET_RANGE_MIN)
        && ((m_VisionTargetReport.m_GroundDistance - GROUND_DISTANCE_TOLERANCE) <= TARGET_RANGE_MAX))
    {
        m_VisionTargetReport.m_bTargetInRange = true;
    }
    else
    {
        m_VisionTargetReport.m_bTargetInRange = false;
    }
    */
    
    m_VisionTargetReport.m_bTargetInRange = false;
}
