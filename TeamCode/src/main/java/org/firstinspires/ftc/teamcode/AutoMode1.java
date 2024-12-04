/* Copyright (c) 2024 Dryw Wade. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.icu.util.Calendar;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/*
 * This OpMode illustrates the basics of AprilTag based localization.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will be used to compute the robot's location and orientation.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the robot, relative to the field origin.
 * This information is provided in the "robotPose" member of the returned "detection".
 *
 * To learn about the Field Coordinate System that is defined for FTC (and used by this OpMode), see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "AutoMode", group = "Concept")
public class AutoMode1 extends DeepHorOpMode {

    // Data relevent to getting our current pos
    private PositionFinder positionFinder = new PositionFinder(); // This may be depricated later
    private MecanumDrive mecanumDrive = null;

    // The threshold for
    private static final double POSITION_THRESHOLD = 0.1;
    private static final double YAW_THRESHOLD = 0.01;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        ConfigureHardware();
        if (!CrossOpModeData.isInitialized) {
            BotInitialization.InitializeRobot(this);
            CrossOpModeData.isInitialized = true;
        }
        // Let's get our position finder ready
        positionFinder.InitializePositionFinder(
                hardwareMap.get(WebcamName.class, "AprilTagCam"),
                hardwareMap.get(IMU.class, "imu")
        );

        boolean hasFoundAprilTag = false;

        while (!hasFoundAprilTag) {
            hasFoundAprilTag = positionFinder.ProcessAprilTagData();
            telemetry.addLine("Looking for April Tag");
            telemetry.update();
        }

        Pose2d initialPose = new Pose2d(positionFinder.x, positionFinder.y, Math.toRadians(positionFinder.firstObtainedAprilYaw));
        mecanumDrive = new MecanumDrive(hardwareMap, initialPose);


        telemetry.addLine("Ready to rumble!");
        telemetry.update();
//        // Wait for the DS start button to be touched.
//        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
//        telemetry.addData(">", "Touch START to start OpMode");
//        telemetry.update();




        waitForStart();

        while (opModeIsActive()) {


//            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch). %s",
//                    mecanumDrive.pose.position.x,
//                    mecanumDrive.pose.position.y,
//                    mecanumDrive.pose.heading.toDouble(),
//                    Calendar.getInstance().getTime()
//            ));

            telemetry.addData("X-Position", mecanumDrive.pose.position.x);
            telemetry.update();

        }
//        if (gamepad1.right_bumper) { // TEMPORARY
//            // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
//            double drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
//            double strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
//            double turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
//            MoveRobot(drive,strafe,turn);
//        }
//        if (gamepad1.left_bumper) {
//            MoveRobot(0,0,-positionFinder.yaw/30);
//        }
//
//        UpdateMoveRobot(1);
//
//            // Save CPU resources; can resume streaming when needed.
//            if (gamepad1.dpad_down) {
//                visionPortal.stopStreaming();
//            } else if (gamepad1.dpad_up) {
//                visionPortal.resumeStreaming();
//            }

//        Actions.runBlocking(
//                mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
//                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
//                        .splineTo(new Vector2d(0, 60), Math.PI)
//                        .build());

//        Actions.runBlocking(
//                mecanumDrive.actionBuilder(mecanumDrive.pose)
//                        .splineTo(new Vector2d(0, 0), Math.PI)
//                        .build());
        positionFinder.isOpmodeRunning = false;
        positionFinder.OnOpmodeStopped();
    }



    private void moveBotTowardsCenter() {
        MoveRobot(-positionFinder.x/3, -positionFinder.y/3, 0);
    }

    private void moveBotToLocation(double targetX, double targetY, double targetYaw) {
        // Step 1: Align the robot to face the target (Yaw)
        double targetAngle = calculateAngle(positionFinder.x, positionFinder.y, targetX, targetY);
        double angleDifference = normalizeAngle(targetAngle - positionFinder.yaw);

        // Step 2: Move towards the target x, y
        double distance = Math.sqrt(Math.pow(targetX - positionFinder.x, 2) + Math.pow(targetY - positionFinder.y, 2));
        while (distance > POSITION_THRESHOLD) {
            double drive = Math.cos(angleDifference);  // Forward/backward movement
            double strafe = Math.sin(angleDifference);  // Left/right strafe movement

            // Adjust yaw based on the angle difference
            double yaw = angleDifference / Math.PI;  // Normalize the yaw to the range [-1, 1]

            MoveRobot(drive, strafe, yaw);

/*
            // Simulate robot movement, update the current position
            currentX += Math.cos(currentYaw) * 0.1 * drive;  // Simulating movement forward/backward
            currentY += Math.sin(currentYaw) * 0.1 * drive;  // Simulating movement forward/backward
            currentYaw += yaw * 0.1;  // Simulating yaw change
*/

            // Recalculate distance and angle after movement
            distance = Math.sqrt(Math.pow(targetX - positionFinder.x, 2) + Math.pow(targetY - positionFinder.y, 2));
            angleDifference = normalizeAngle(targetAngle - positionFinder.yaw);
        }

        // Step 3: Adjust final yaw to match the target yaw
        double finalYawDifference = normalizeAngle(targetYaw - positionFinder.yaw);
        while (Math.abs(finalYawDifference) > YAW_THRESHOLD) {
            MoveRobot(0, 0, finalYawDifference / Math.PI);  // Only rotate

//            currentYaw += (finalYawDifference / Math.PI) * 0.1;  // Simulate yaw rotation
            finalYawDifference = normalizeAngle(targetYaw - positionFinder.yaw);
        }
    }





//    /**
//     * Add telemetry about AprilTag detections.
//     */
//    private void telemetryAprilTag() {
//
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)",
//                        detection.robotPose.getPosition().x,
//                        detection.robotPose.getPosition().y,
//                        detection.robotPose.getPosition().z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
//                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
//                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
//                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }   // end for() loop
//
//        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//
//    }   // end method telemetryAprilTag()

    // MATH!!!!!

    // Method to compute the angle between current and target positions
    private double calculateAngle(double currentX, double currentY, double targetX, double targetY) {
        return Math.atan2(targetY - currentY, targetX - currentX);
    }

    // Method to normalize angle between -PI and PI
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

}   // end class
