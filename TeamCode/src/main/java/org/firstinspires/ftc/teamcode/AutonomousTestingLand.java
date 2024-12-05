package org.firstinspires.ftc.teamcode;

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

import android.annotation.SuppressLint;
import android.icu.util.Calendar;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.acmerobotics.roadrunner.HolonomicController;


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


@Autonomous
(name = "AutoTestFile", group = "Concept")
public class AutonomousTestingLand extends DeepHorOpMode {

    // Data relevent to getting our current pos
    private PositionFinder positionFinder = new PositionFinder(); // This may be depricated later
    private MecanumDrive mecanumDrive = null;

    // The threshold for
    private static final double POSITION_THRESHOLD = 0.1;
    private static final double YAW_THRESHOLD = 0.01;



    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        ConfigureHardware(false);
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
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap, initialPose);

        if (hasFoundAprilTag) {
            telemetry.addLine("Ready to rumble!");
            telemetry.addData("aprilTagX", positionFinder.x);
            telemetry.addData("AprilTagY", positionFinder.y);
            telemetry.addData("AprilTagHeading", positionFinder.yaw);
            telemetry.update();

        } else {
                telemetry.addLine("AprilTag Not Found, Please Reposition and Try Again");
            }





//        // Wait for the DS start button to be touched.
//        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
//        telemetry.addData(">", "Touch START to start OpMode");
//        telemetry.update();

        waitForStart();


        //Stop the robot when we want it to stop.
        if (isStopRequested()) return;


        //Multithread Telemetry
        new Thread(() -> {
            while (opModeIsActive()) {
                mecanumDrive.updatePoseEstimate();
                telemetry.addData("x", mecanumDrive.pose.position.x);
                telemetry.addData("y", mecanumDrive.pose.position.y);
                telemetry.addData("Heading", mecanumDrive.pose.heading.toDouble());
                telemetry.update();
                try {
                    Thread.sleep(100); // Update every 100ms
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }).start();


        //TrajectoryShit
        Actions.runBlocking(
                drivetrain.actionBuilder(new Pose2d(0,0,0))
                        .lineToX(32)
                        .waitSeconds(5)
                        .lineToY(32)
                        .waitSeconds(5)
                        .turnTo(Math.toRadians(120))
                        .build());



        telemetry.addLine("Roadrunner trajectory finished, Program executed succesfully");
        telemetry.update();







    }
}

