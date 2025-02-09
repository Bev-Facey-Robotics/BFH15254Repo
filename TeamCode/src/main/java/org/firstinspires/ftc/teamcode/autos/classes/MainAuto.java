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
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AprilTagPosFinder;
import org.firstinspires.ftc.teamcode.BotInitialization;
import org.firstinspires.ftc.teamcode.CrossOpModeData;
import org.firstinspires.ftc.teamcode.DeepHorOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;


public abstract class MainAuto extends DeepHorOpMode {

    //region Position
    private AprilTagPosFinder aprilTagPosFinder = new AprilTagPosFinder();
    public MecanumDrive mecanumDrive; // Road Runner

    public Pose2d initialPose;
    //endregion





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


         initialPose = new Pose2d(aprilTagPosFinder.x, aprilTagPosFinder.y, Math.toRadians(aprilTagPosFinder.yaw));
        mecanumDrive = new MecanumDrive(hardwareMap, initialPose);
        aprilTagPosFinder.StopStreaming();




        waitForStart();// We shouldn't need this, but better to be safe than sorry!



        Action trajChosen;
        trajChosen = redThreeYellows(mecanumDrive, initialPose).build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        trajChosen
                )
        );








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



        while (opModeIsActive()) {

            // To make sure that the piece isn't flung out of the basket
//            if (!stage2BucketSync) {
//                StartBucketSync();


            telemetry.addLine("Finished!");
        }
    }

    public abstract TrajectoryActionBuilder redThreeYellows (MecanumDrive mecanumDrive, Pose2d initialPose);

}










