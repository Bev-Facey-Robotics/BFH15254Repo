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

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.AprilTagPosFinder;
import org.firstinspires.ftc.teamcode.BotInitialization;
import org.firstinspires.ftc.teamcode.CrossOpModeData;
import org.firstinspires.ftc.teamcode.DeepHorOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;


public abstract class MainAuto extends DeepHorOpMode {

    //region Position
    private AprilTagPosFinder aprilTagPosFinder = new AprilTagPosFinder();
    private MecanumDrive mecanumDrive = null; // Road Runner
    //endregion

    //Feedback loop for the Robot to have a greater degree of accurcy while following RR trajectories



    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        //region Hardware Initialization
        ConfigureHardware(false);
        if (!CrossOpModeData.isInitialized) {
            BotInitialization.InitializeRobot(this);
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
            telemetry.addLine("Ready to rumble!");
            telemetry.addData("April Tag Position", "X: %f, Y: %f", aprilTagPosFinder.x, aprilTagPosFinder.y);
            telemetry.addData("April Tag Yaw", "Yaw: %f", aprilTagPosFinder.yaw);
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


    //Roadrunner Actions, such as raising and lowering slides, implementing bucket states (raised, lowered and dropping), and intake + transfer shit

    //Raising slide action
    public class slideUp implements Action {
        private boolean initialized = false;

        //Actions with telem
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                stage1Arm.setPower(0.8);
                initialized = true;
            }

            //Look to see where the slides are
            double SlidePos = stage1Arm.getCurrentPosition();
            packet.put("SlidePos", SlidePos);

            //Logic to make it go up until telem limits are reached
            if (SlidePos < -10000) {
                return true;
            } else {
                stage1Arm.setPower(0.5);
                return false;
            }
        }
    }

    //Method to make raising the slide easier
    public Action slideUp() {
        return new slideUp();
    }

    public class slideDown implements Action {
        private boolean initialized = false;

        //Actions with telem
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                stage1Arm.setPower(-0.8);
                initialized = true;
            }

            //Look to see where the slides are and put it on the driver station
            double SlidePos = stage1Arm.getCurrentPosition();
            packet.put("SlidePos", SlidePos);

            //Logic to make it go up until telem limits are reached
            if (SlidePos > 0) {
                return true;
            } else {
                stage1Arm.setPower(-0.5);
                return false;
            }
        }
    }

    //Method to lower the slide easier
    public Action slideDown () {
        return new slideDown();
    }

    //Class for raising the bucket
    public class bucketUp implements Action {
        private boolean initialized = false;

        //Actions with telem
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                stage2Swing
                initialized = true;
            }
    }

    //Class for dropping the sample
    public class bucketPlace implements Action {
        private boolean initialized = false;

        //Actions with telem
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                stage2Bucket.setPosition(0.1);
                initialized = true;
            }

        double piecePlace = stage2Bucket.getPosition();
        packet.put("BucketPos", piecePlace);
        if (piecePlace != 0.3) {
            return true;
        } else {
            stage2Bucket.setPosition(0.3);
            return false;
        }
    }

    //Class for lowering the bucket
    public class bucketDown implements Action {
        private boolean initialized = false;

        //Actions with telem
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                stage2Swing.setPower(-0.1);
                initialized = true;
            }

        double bucketArmPos = stage2Swing.getCurrentPosition();
            packet.put("SwingPos", bucketArmPos);

            return false;
        }

    //Class for sample intake
    public class sampleGrab implements Action {
        private boolean initialized = false;
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                stage1Scoop.setPower(1.0);
                sleep(1500);
                return true;
            } else {
                return false;
            }

        }

    //Method for sample intake
    public Action sampleGrab () {
        return new sampleGrab();


    }



    }

    //Class for piece transfer
    public class pieceSwap implements Action {
        private boolean initialized = false;
///*TODO: Borrow Share functionality and put in this action
        //Actions with telem
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                bucketTargetPosition = 0.05;
                MoveSlidePos(-1821);



                initialized = true;

                //moves the intake to transfer to the bucket
                stage1Arm.setTargetPosition(255);
                stage1Arm.setPower(0.3);


            }

    }

}
// end class
