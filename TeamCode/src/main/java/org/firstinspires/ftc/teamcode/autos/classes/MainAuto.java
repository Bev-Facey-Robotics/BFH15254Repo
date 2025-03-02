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
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.AprilTagPosFinder;
import org.firstinspires.ftc.teamcode.CrossOpModeData;
import org.firstinspires.ftc.teamcode.autos.rractions.RRFrontCombine;
import org.firstinspires.ftc.teamcode.autos.rractions.RRSlide;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.internal.BaseOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.internal.HardwareManager;


public abstract class MainAuto extends BaseOpMode {


//
//    public class updatePose implements Action {
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) throws NullPointerException {
//            packet.put("poseUpdate", mecanumDrive.updatePoseEstimate());
//            updatedPose = mecanumDrive.updatePoseEstimate();
//            return true;
//        }
//    }

//    public Action updatePose() {return new updatePose();}



    //region Position
    private MecanumDrive mecanumDrive = null; // Road Runner
    private Pose2d importedinitialPose = importedinitialPose();




    //endregion

    public RRSlide slide = null;
    public RRFrontCombine frontCombine = null;

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
        slide = new RRSlide(hardwareMap);
        frontCombine = new RRFrontCombine(hardwareMap);
        
        //region April Tag Initialization
        // Let's get our position finder ready
//        aprilTagPosFinder.Initialize(
//                hardwareMap,
//                telemetry
//        );
//
//        boolean hasFoundAprilTag = false;
//
//        while (!hasFoundAprilTag && !isStopRequested()) {
//            telemetry.addLine("Looking for Initial April Tag");
//            telemetry.update();
//            hasFoundAprilTag = aprilTagPosFinder.ProcessAprilTagData();
//        }

        if (isStopRequested()) {
            return;
        }

        while (opModeInInit()) {
//            telemetry.addData("April Tag Found", "ID: %d", positionFinder.firstObtainedAprilTagID);
    //        aprilTagPosFinder.ProcessAprilTagData();
//            telemetry.addLine("Ready to rumble!");
//            telemetry.addData("April Tag Position", "X: %f, Y: %f", aprilTagPosFinder.x, aprilTagPosFinder.y);
//            telemetry.addData("April Tag Yaw", "Yaw: %f", aprilTagPosFinder.yaw);
            telemetry.update();
        }

        if (isStopRequested()) {
            return;
        }
        //endregion

        Pose2d initialPose = importedinitialPose;
        //aprilTagPosFinder.StopStreaming();


        // for debugging
        //Pose2d initialPose = new Pose2d(0,0,Math.toRadians(aprilTagPosFinder.yaw));

        waitForStart(); // We shouldn't need this, but better to be safe than sorry!

        mecanumDrive = new MecanumDrive(hardwareMap, initialPose);

        //build all the trajectories here. Yes there is a lot
        TrajectoryActionBuilder aCscoreStartingSpecimenTraj = scoreStartingSpecimenTraj(mecanumDrive, initialPose);
        TrajectoryActionBuilder aCscoreStartingSpecimenTraj2 = scoreStartingSpecimenTraj2(mecanumDrive, aCscoreStartingSpecimenTraj);
        TrajectoryActionBuilder aCscoreSecondSpecimenTraj = scoreSecondSpecimenTraj(mecanumDrive, aCscoreStartingSpecimenTraj);
        TrajectoryActionBuilder aCscoreThirdSpecimenTraj = scoreThirdSpecimenTraj(mecanumDrive, aCscoreSecondSpecimenTraj);
        TrajectoryActionBuilder aCscoreFourthSpecimenTraj = scoreFourthSpecimenTraj(mecanumDrive, aCscoreThirdSpecimenTraj);
        TrajectoryActionBuilder aCscoreFifthSpecimenTraj = scoreFifthSpecimenTraj(mecanumDrive, aCscoreFourthSpecimenTraj);
        TrajectoryActionBuilder aCwallSpecimenTraj = wallSpecimenTraj(mecanumDrive, aCscoreFifthSpecimenTraj);
        TrajectoryActionBuilder aCkickSample1Traj = kickSample1Traj(mecanumDrive, aCwallSpecimenTraj);
        TrajectoryActionBuilder aCkickSample2Traj = kickSample2Traj(mecanumDrive, aCkickSample1Traj);
        TrajectoryActionBuilder aCkickSample3Traj = kickSample3Traj(mecanumDrive, aCkickSample2Traj);
        TrajectoryActionBuilder aCmoveFromWallTraj = moveFromWall(mecanumDrive, aCkickSample3Traj);
        TrajectoryActionBuilder aCparkRunTraj = parkRun(mecanumDrive, aCmoveFromWallTraj);
        TrajectoryActionBuilder aCSleep = sleepHalfSec(mecanumDrive, aCparkRunTraj);
        TrajectoryActionBuilder aCSTwoSleep = sleepTwoSec(mecanumDrive, aCSleep);

        Action aCwallSpecimenActi = aCwallSpecimenTraj.build();
        Action aCkickSample1Acti = aCkickSample1Traj.build();
        Action aCkickSample2Acti = aCkickSample2Traj.build();
        Action aCkickSample3Acti = aCkickSample3Traj.build();
        Action aCscoreStartingSpecimenActi = aCscoreStartingSpecimenTraj.build();
        Action aCscoreStartingSpecimenActi2 = aCscoreStartingSpecimenTraj2.build();
        Action aCscoreSecondSpecimenActi = aCscoreSecondSpecimenTraj.build();
        Action aCscoreThirdSpecimenActi = aCscoreThirdSpecimenTraj.build();
        Action aCscoreFourthSpecimenActi = aCscoreFourthSpecimenTraj.build();
        Action aCscoreFifthSpecimenActi = aCscoreFifthSpecimenTraj.build();
        Action aCmoveFromWallActi = aCmoveFromWallTraj.build();
        Action aCparkRunActi = aCparkRunTraj.build();
        Action aCSleepActi = aCSleep.build(); // something we don't know how to do lol
        Action aCTwoSleepActi = aCSTwoSleep.build();







        waitForStart();

        Actions.runBlocking(new SequentialAction(
//                new ParallelAction(
//                        aCmoveFromWall,
//                        frontCombine.MoveToPickup()
//
//                ),
                aCTwoSleepActi,

                ///Placing the preloaded specimen on high rung
                new ParallelAction(
                        aCscoreStartingSpecimenActi,
                        slide.MoveToHighChamber()
                ),
                aCscoreStartingSpecimenActi,
                aCTwoSleepActi,
                ///Releasing the specimens on high chamber
                slide.ReleaseSpeciminFromHigh(),
                aCTwoSleepActi,
//                aCwallSpecimenActi,
                aCparkRunActi



        ));
///Batting the other specimens into the obv zone + grabbing a specimen
//                new SequentialAction(
//                       aCkickSample1Traj,
//                        aCkickSample2Traj,
//                        aCkickSample3Traj,
//                        aCwallSpecimenTraj),
//
//                ///grabbing the specimen
//                new ParallelAction(
//                        aCscoreSecondSpecimenTraj,
//                        slide.MoveToHighChamber()),
//                ///Releasing the specimen
//                new SequentialAction(
//                        slide.MoveToWall()),
//
//                ///grabbing the specimen
//                new ParallelAction(
//                        aCscoreThirdSpecimenTraj,
//                        slide.MoveToHighChamber()),
//                ///Releasing the specimen
//                new SequentialAction(
//                        slide.MoveToWall()),
//
//                ///grabbing the specimen
//                new ParallelAction(
//                        aCscoreFourthSpecimenTraj,
//                        slide.MoveToHighChamber()),
//                ///Releasing the specimen
//                new SequentialAction(
//                        slide.MoveToWall()),
//                ///grabbing the specimen
//
//                new ParallelAction(
//                        aCscoreFifthSpecimenTraj,
//                        slide.MoveToHighChamber()),
//                ///Releasing the specimen
//                new SequentialAction(
//                        slide.MoveToWall())
//



//


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
            telemetry.addLine("Finished!");
        }
    }



    public abstract TrajectoryActionBuilder wallSpecimenTraj (MecanumDrive mecanumDrive, TrajectoryActionBuilder previousAction);
    public abstract TrajectoryActionBuilder kickSample1Traj (MecanumDrive mecanumDrive, TrajectoryActionBuilder previousAction);
    public abstract TrajectoryActionBuilder kickSample2Traj (MecanumDrive mecanumDrive, TrajectoryActionBuilder previousAction);
    public abstract TrajectoryActionBuilder kickSample3Traj (MecanumDrive mecanumDrive, TrajectoryActionBuilder previousAction);
    public abstract TrajectoryActionBuilder scoreStartingSpecimenTraj (MecanumDrive mecanumDrive, Pose2d initialPose);
    public abstract TrajectoryActionBuilder scoreStartingSpecimenTraj2 (MecanumDrive mecanumDrive, TrajectoryActionBuilder previousAction);
    public abstract TrajectoryActionBuilder scoreSecondSpecimenTraj (MecanumDrive mecanumDrive, TrajectoryActionBuilder previousAction);
    public abstract TrajectoryActionBuilder scoreThirdSpecimenTraj (MecanumDrive mecanumDrive, TrajectoryActionBuilder previousAction);
    public abstract TrajectoryActionBuilder scoreFourthSpecimenTraj (MecanumDrive mecanumDrive, TrajectoryActionBuilder previousAction);
    public abstract TrajectoryActionBuilder scoreFifthSpecimenTraj (MecanumDrive mecanumDrive, TrajectoryActionBuilder previousAction);
    public abstract TrajectoryActionBuilder moveFromWall (MecanumDrive mecanumDrive, TrajectoryActionBuilder previousAction);
    public abstract TrajectoryActionBuilder parkRun (MecanumDrive mecanumDrive, TrajectoryActionBuilder previousAction);

    public abstract Pose2d importedinitialPose();
    public  TrajectoryActionBuilder sleepHalfSec (MecanumDrive mecanumDrive, TrajectoryActionBuilder previousAction) {
        return previousAction.endTrajectory().fresh()
                .waitSeconds(0.5);
    }
    public  TrajectoryActionBuilder sleepTwoSec(MecanumDrive mecanumDrive, TrajectoryActionBuilder previousAction) {
        return previousAction.endTrajectory().fresh()
                .waitSeconds(10);
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
