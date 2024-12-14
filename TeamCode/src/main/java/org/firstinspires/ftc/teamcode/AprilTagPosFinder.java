package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class AprilTagPosFinder {

    private AprilTagProcessor aprilTagProcessor;
    private OpenCvCamera camera; // why?
    private Telemetry telemetry;

    // AprilTag detection results
    public int aprilTagId = -1;
    public double x = 0;
    public double y = 0;
    public double yaw = 0;

    public void Initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .build();

        //region Camera
        // Initialize camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "AprilTagCam"), cameraMonitorViewId);
        //camera = OpenCvCameraFactory.createWebcam(hardwareMap.get(WebcamName.class, "AprilTagCam"), cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline((OpenCvPipeline) camera);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
        //endregion
    }

    public boolean ProcessAprilTagData() {
        // Get the latest AprilTag detections
        ArrayList<AprilTagDetection> detection = aprilTagProcessor.getDetections();


        if (!detection.isEmpty()) {
            // TODO: Multitag localization

            // Access the first detection using index 0
            AprilTagDetection detection1 = detection.get(0);

            // Use the detection object as needed
            aprilTagId = detection1.id;
            telemetry.addData("Detected Tag ID", aprilTagId);

            // Positions
            x = detection1.robotPose.getPosition().x;
            y = detection1.robotPose.getPosition().y;
            // Adjust yaw as needed (e.g., apply offset or coordinate transformation)
            // TODO: Is this (yaw) relivate to the tag, or the field?
            yaw = detection1.robotPose.getOrientation().getYaw(); // Assuming yaw is in radians

            // Update telemetry
            telemetry.addData("AprilTag ID", aprilTagId);
            telemetry.addData("AprilTag X", x);
            telemetry.addData("AprilTag Y", y);
            telemetry.addData("AprilTag Yaw", yaw);
            telemetry.update();

            return true; // AprilTag found
        } else {
            // Handle the case where no detections are available
            telemetry.addData("Status", "No AprilTag detections found");
            telemetry.update();
            return false;
        }
    }

}


//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
/////LECEACY CODE - USE ONLY IN A EMERGEENCY
//
////package org.firstinspires.ftc.teamcode;
////
////import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
////import com.qualcomm.robotcore.hardware.DcMotor;
////import com.qualcomm.robotcore.hardware.IMU;
////
////import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
////import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
////import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
////import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
////import org.firstinspires.ftc.robotcore.external.navigation.Position;
////import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
////import org.firstinspires.ftc.vision.VisionPortal;
////import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
////import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
////import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
////
////import java.util.ArrayList;
////import java.util.List;
////import java.util.OptionalDouble;
////
/////**
//// * This class is used to find the position of the robot on the field. It uses the IMU to find the yaw of the robot, and the AprilTags to find the x and y position of the robot.
//// * It should be used in a separate thread, as it will constantly update the position of the robot.
//// * To initialize, call InitializePositionFinder with the webcam and IMU.
//// * Example: positionFinder.InitializePositionFinder(hardwareMap.get(WebcamName.class, "Webcam 1"), hardwareMap.get(IMU.class, "imu"));
//// * In order to start grabbing our position, call FindBotPosition. MAKE SURE TO CALL IT IN A SEPARATE THREAD.
//// */
////public class PositionFinder {
////    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
////
////    /**
////     * Variables to store the position and orientation of the camera on the robot. Setting these
////     * values requires a definition of the axes of the camera and robot:
////     *
////     * Camera axes:
////     * Origin location: Center of the lens
////     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
////     *
////     * Robot axes (this is typical, but you can define this however you want):
////     * Origin location: Center of the robot at field height
////     * Axes orientation: +x right, +y forward, +z upward
////     *
////     * Position:
////     * If all values are zero (no translation), that implies the camera is at the center of the
////     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
////     * inches above the ground - you would need to set the position to (-5, 7, 12).
////     *
////     * Orientation:
////     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
////     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
////     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
////     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
////     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
////     */
////
////
////
////
////    /**
////     * Our x position in cm relative to the field
////     */
////    public double x;
////    /**
////     * Our y position in cm relative to the field
////     */
////    public double y;
////
////    /**
////     * Our previous yaw in degrees
////     */
////    public double prevYaw = 123456789; // 123456789 is our "we don't got stuff" value
////    /**
////     * Our yaw in degrees
////     */
////    public double yaw = 123456789; // 123456789 is our "we don't got stuff" value
////
////    // Yaw stuff
////    /**
////     * The offset between where the IMU says the yaw is, and what the yaw is relative to the map (thru april tags)
////     */
////    public double imuPosOffset = 123456789;
////    public double firstObtainedAprilYaw = 123456789;
////    public boolean hasObtainedYawLock = false;
////
////    public IMU imu;
////
////    // Some position stuff
////    final private Position cameraPosition = new Position(DistanceUnit.INCH,
////            -6.5, -6, -4.5, 0);
////    final private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
////            0, -90, 0, 0);
////
////    // Hardware
////    private WebcamName hwWebcam;
////
////    /**
////     * The variable to store our instance of the AprilTag processor.
////     */
////    private AprilTagProcessor aprilTag;
////    /**
////     * The variable to store our instance of the vision portal.
////     */
////    private VisionPortal visionPortal;
////
////    /**
////     * This variable stores whether our opmode is running
////     */
////    public boolean isOpmodeRunning = true;
////
////    /**
////     * Not implemented yet
////     */
////    public boolean isPositionFinderReady = false;
////    /**
////     * Initializes the position finder with the camera, and IMU.
////     * The position finder initializes the IMU, so do not initialize the IMU elsewhere.
////     * @param _camera The hardwareMap.get of our webcam
////     * @param _imu The hardwareMap.get of our IMU
////     */
////    public void InitializePositionFinder(WebcamName _camera, IMU _imu) {
////        imu = _imu;
////        hwWebcam = _camera;
//////        initIMU();
////        initAprilTag();
////    }
////
////    /**
////     * Cleans up on things the position finder needs to operate.
////     */
////    public void OnOpmodeStopped() {
////        visionPortal.close();
////    }
////
////
////    /**
////     * Main loop that finds our bot's pos.
////     * Should be ran always in a different thread than the primary one.
////     */
////    public void FindBotPosition() {
////        while (isOpmodeRunning) {
////            boolean hasFoundAprilTag = ProcessAprilTagData();
////            if (!hasObtainedYawLock) {
////                aprilTag.setDecimation(1);
////                // At present moment we have unreliable yaw data, and auto will most likely be unable to start.
////                // We need to figure out our yaw
////                yaw = imu.getRobotYawPitchRollAngles().getYaw();// Get our impersice yaw
////                if (firstObtainedAprilYaw != 123456789) {
////                    // We have our first yaw data, so we can now calculate our offset
////                    imuPosOffset = normalizeRotation(firstObtainedAprilYaw - yaw); // -45 is because the 0 rot is stupid, and is at an angle
////                    hasObtainedYawLock = true;
////                    continue;
////                }
////            } else {
////                // Do our yaw stuff
////                prevYaw = yaw; // used for our odometer position calculations.
////                yaw = normalizeRotation(imu.getRobotYawPitchRollAngles().getYaw() - imuPosOffset);// Get our yaw
////                aprilTag.setDecimation(3);
////            }
////        }
////    }
////
//////    /**
//////     * Initialize the IMU.
//////     */
//////    private void initIMU() {
//////
//////        // Retrieve and initialize the IMU.
//////        IMU.Parameters parameters;
//////
//////        parameters = new IMU.Parameters(
//////                new RevHubOrientationOnRobot(
//////                        // https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html
//////                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//////                        RevHubOrientationOnRobot.UsbFacingDirection.UP
//////                )
//////        );
//////
//////        imu.initialize((parameters));
//////    }
////    /**
////     * Initialize the AprilTag processor.
////     */
////    public void initAprilTag() {
////
////        // Create the AprilTag processor.
////        aprilTag = new AprilTagProcessor.Builder()
////
////                // The following default settings are available to un-comment and edit as needed.
////                .setDrawAxes(false)
////                //.setDrawCubeProjection(false)
////                //.setDrawTagOutline(true)
////                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
////                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
////                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
////                .setCameraPose(cameraPosition, cameraOrientation)
////
////                // == CAMERA CALIBRATION ==
////                // If you do not manually specify calibration parameters, the SDK will attempt
////                // to load a predefined calibration for your camera.
////                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
////                // ... these parameters are fx, fy, cx, cy.
////
////                .build();
////
////        // Adjust Image Decimation to trade-off detection-range for detection-rate.
////        // eg: Some typical detection data using a Logitech C920 WebCam
////        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
////        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
////        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
////        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
////        // Note: Decimation can be changed on-the-fly to adapt during a match.
////        aprilTag.setDecimation(1);
////
////        // Create the vision portal by using a builder.
////        VisionPortal.Builder builder = new VisionPortal.Builder();
////
////        // Set the camera (webcam vs. built-in RC phone camera).
////        if (USE_WEBCAM) {
////            builder.setCamera(hwWebcam);
////        } else {
////            builder.setCamera(BuiltinCameraDirection.BACK);
////        }
////
////        // Choose a camera resolution. Not all cameras support all resolutions.
////        //builder.setCameraResolution(new Size(640, 480));hardwareMap.get(WebcamName.class, "Webcam 1")
////
////        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
////        builder.enableLiveView(true);
////
////        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
////        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
////
////        // Choose whether or not LiveView stops if no processors are enabled.
////        // If set "true", monitor shows solid orange screen if no processors enabled.
////        // If set "false", monitor shows camera view without annotations.
////        //builder.setAutoStopLiveView(false);
////
////        // Set and enable the processor.
////        builder.addProcessor(aprilTag);
////
////        // Build the Vision Portal, using the above settings.
////        visionPortal = builder.build();
////
////        // Disable or re-enable the aprilTag processor at any time.
////        //visionPortal.setProcessorEnabled(aprilTag, true);
////
////    }   // end method initAprilTag()
////
////    /**
////     * This uses data we have from any april tags we have detected, and tries to calculate a position from it. It only calculates our yaw on first launch, because then our IMU can take over, which is waaaaay more accurate.
////     */
////    public boolean ProcessAprilTagData() {
////        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
////        List<Double> aprilsX = new ArrayList<>();
////        List<Double> aprilsY = new ArrayList<>();
////        List<Double> aprilsYaw = new ArrayList<>();
////
////        for (AprilTagDetection detection : currentDetections) {
////            // Get all estimated positions for our location
////            aprilsX.add(detection.robotPose.getPosition().x);
////            aprilsY.add(detection.robotPose.getPosition().y);
//////            if (firstObtainedAprilYaw == 123456789) {
////                aprilsYaw.add(detection.robotPose.getOrientation().getYaw());
//////            }
////        }   // end for() loop
////
////        // Calculate average location (so long as we actually have a position)
////        if (!aprilsX.isEmpty()) {
////            OptionalDouble averageX = aprilsX
////                    .stream()
////                    .mapToDouble(a -> a)
////                    .average();
////            OptionalDouble averageY = aprilsY
////                    .stream()
////                    .mapToDouble(a -> a)
////                    .average();
//////            if (firstObtainedAprilYaw == 123456789) {
////                OptionalDouble averageYaw = aprilsYaw
////                        .stream()
////                        .mapToDouble(a -> a)
////                        .average();
////                firstObtainedAprilYaw = averageYaw.isPresent() ? averageYaw.getAsDouble() : 0;
//////            }
////
////            // The "if it's not present" check is just to get java to shut up.
////            x = averageX.isPresent() ? averageX.getAsDouble() : 0;
////            y = averageY.isPresent() ? averageY.getAsDouble() : 0;
////            return true;
////        }
////        return false;
////    }
////    // Method to normalize angle between -90 and 90 degrees
////    private double normalizeRotation(double angle) {
////        while (angle > 180) angle -= 360;
////        while (angle < -180) angle += 360;
////        return angle;
////    }
////}
