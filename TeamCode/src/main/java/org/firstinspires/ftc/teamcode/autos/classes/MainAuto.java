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

package org.firstinspires.ftc.teamcode.autos.classes;

import android.annotation.SuppressLint;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AprilTagPosFinder;
import org.firstinspires.ftc.teamcode.CrossOpModeData;
import org.firstinspires.ftc.teamcode.internal.BaseOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.jetbrains.annotations.NotNull;

public abstract class MainAuto extends BaseOpMode {

    //region Position
    private AprilTagPosFinder aprilTagPosFinder = new AprilTagPosFinder();
    private MecanumDrive mecanumDrive = null; // Road Runner
    //endregion

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        //region Hardware Initialization
        //ConfigureHardware(false);
        if (!CrossOpModeData.isInitialized) {
            //BotInitialization.InitializeRobot(this);
            CrossOpModeData.isInitialized = true;
        }
        //endregion

        //region April Tag Initialization
        // Let's get our position finder ready
        aprilTagPosFinder.Initialize(
                hardwareMap,
                telemetry
        );

        boolean hasFoundAprilTag = false;

        while (!hasFoundAprilTag && !isStopRequested()) {
            telemetry.addLine("Looking for Initial April Tag");
            telemetry.update();
            hasFoundAprilTag = aprilTagPosFinder.ProcessAprilTagData();
        }

        if (isStopRequested()) {
            return;
        }

        while (opModeInInit()) {
//            telemetry.addData("April Tag Found", "ID: %d", positionFinder.firstObtainedAprilTagID);
            aprilTagPosFinder.ProcessAprilTagData();
//            telemetry.addLine("Ready to rumble!");
//            telemetry.addData("April Tag Position", "X: %f, Y: %f", aprilTagPosFinder.x, aprilTagPosFinder.y);
//            telemetry.addData("April Tag Yaw", "Yaw: %f", aprilTagPosFinder.yaw);
            telemetry.update();
        }

        if (isStopRequested()) {
            return;
        }
        //endregion

        Pose2d initialPose = new Pose2d(aprilTagPosFinder.x, aprilTagPosFinder.y, Math.toRadians(aprilTagPosFinder.yaw));
        aprilTagPosFinder.StopStreaming();

        // for debugging
        //Pose2d initialPose = new Pose2d(0,0,Math.toRadians(aprilTagPosFinder.yaw));

        waitForStart(); // We shouldn't need this, but better to be safe than sorry!

        mecanumDrive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder parkingRun = parkingRun(mecanumDrive, initialPose);

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
        });


        Actions.runBlocking(parkingRun.build());
        while (opModeIsActive()) {
            telemetry.addLine("Finished!");
        }
    }

    public abstract TrajectoryActionBuilder parkingRun(MecanumDrive mecanumDrive, Pose2d initialPose);
    public Slide Slideup;

    //Trying out actions

    public class Slide {
        private DcMotor motorSlide;

        public Slide(HardwareMap hardwareMap) {
            motorSlide = hardwareMap.get(DcMotor.class, "slideMotor");

        }

        public class SlideUp implements Action {

            private Boolean initalized = false;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                while (initalized) {
                    if (motorSlide.getCurrentPosition() < -10000); {
                        motorSlide.setPower(0);
                    }


                }



                if (!initalized) {
                    motorSlide.setPower(0.3);
                    initalized = true;
                }

                double vel = motorSlide.getPower();
                packet.put("Slide Power", vel);
                return vel < 0.4;
            }

            public Action slideUp() {
                return new SlideUp();
            }
        }
    }












    public void main() {
        // This is the main method for the auto
    }
    public void initializeHardware() {
        // This is the method to initialize the hardware
    }

    public void calibrateHardware() {
        // This is the method to calibrate the hardware
    }

}   // end class
