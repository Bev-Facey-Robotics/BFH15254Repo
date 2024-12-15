package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class AprilTagPosFinder {
    final private boolean STREAM_CAMERA = false; // do not enable


    //region Hardware
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private OpenCvCamera camera; // why?
    private Telemetry telemetry;
    //endregion

    //region AprilTag detection results
    public int aprilTagId = -1;
    public double x = 0;
    public double y = 0;
    public double yaw = 0;
    //endregion

    //region Camera Positioning
    final private Position cameraPosition = new Position(DistanceUnit.INCH,
            -6.5, -6, -4.5, 0);
    final private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    //endregion

    /**
     * Initializes the AprilTag processor and camera.
     * @param hardwareMap The hardware map of the robot.
     * @param telemetry The telemetry module (for yelling at you)
     */
    public void Initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        //region AprilTag Builder
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();
        //endregion

        //region April Tag Configuration
        aprilTagProcessor.setDecimation(1); // Detect 2" Tag from 10 feet away at 10 Frames per second
        //endregion

        //region Vision Portal
        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera
        builder.setCamera(hardwareMap.get(WebcamName.class, "AprilTagCam"));
        builder.setCameraResolution(new Size(1920, 1080)); // HD res for our C970

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTagProcessor);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

        //endregion

        //region Camera Streaming
        if (STREAM_CAMERA) {
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
        }
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

            //region Fetching Position Data
            // Positions
            x = detection1.robotPose.getPosition().x;
            y = detection1.robotPose.getPosition().y;
            // Adjust yaw as needed (e.g., apply offset or coordinate transformation)
            // TODO: Is this (yaw) relivate to the tag, or the field?
            yaw = detection1.robotPose.getOrientation().getYaw(); // Assuming yaw is in radians
            //endregion

            //region Telemetry
            // Update telemetry
            telemetry.addData("AprilTag ID", aprilTagId);
            telemetry.addData("AprilTag X", x);
            telemetry.addData("AprilTag Y", y);
            telemetry.addData("AprilTag Yaw", yaw);
            telemetry.update();
            //endregion

            return true; // AprilTag found
        } else {
            // Handle the case where no detections are available
            telemetry.addData("Status", "No AprilTag detections found");
            telemetry.update();
            return false;
        }
    }

    public void StopStreaming() {
        visionPortal.close();
    }
}