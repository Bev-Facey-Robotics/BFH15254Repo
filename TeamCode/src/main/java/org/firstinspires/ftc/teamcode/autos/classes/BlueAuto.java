package org.firstinspires.ftc.teamcode.autos.classes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.internal.BaseOpMode;


    public class BlueAuto extends MainAuto {
        /// Parking run
        public TrajectoryActionBuilder parkRun(MecanumDrive mecanumDrive, Pose2d initialPose) {
            return mecanumDrive.actionBuilder(initialPose)
                    .setTangent(0)
                    .waitSeconds(25)
                    .strafeToLinearHeading(new Vector2d(46, -65), Math.toRadians(90));
        }

        //Scores the preloaded specimen on high chamber
        public TrajectoryActionBuilder scoreStartingSpecimenTraj(MecanumDrive mecanumDrive, Pose2d initialPose) {
            return mecanumDrive.actionBuilder(initialPose)
                    .strafeToSplineHeading(new Vector2d(-7.5, 32), Math.toRadians(270));
        }

        //Moves the bot over an inch to score the second specimen. Ditto with 3-5specimenTraj, but I can't think of a way to make it simpler
        public TrajectoryActionBuilder scoreSecondSpecimenTraj(MecanumDrive mecanumDrive, Pose2d initialPose) {
            return mecanumDrive.actionBuilder(initialPose)
                    .strafeToSplineHeading(new Vector2d(-6.5, 32), Math.toRadians(270));
        }

        public TrajectoryActionBuilder scoreThirdSpecimenTraj(MecanumDrive mecanumDrive, Pose2d initialPose) {
            return mecanumDrive.actionBuilder(initialPose)
                    .strafeToSplineHeading(new Vector2d(-5.5, 32), Math.toRadians(270));
        }

        public TrajectoryActionBuilder scoreFourthSpecimenTraj(MecanumDrive mecanumDrive, Pose2d initialPose) {
            return mecanumDrive.actionBuilder(initialPose)
                    .strafeToSplineHeading(new Vector2d(-4.5, 32), Math.toRadians(270));
        }

        public TrajectoryActionBuilder scoreFifthSpecimenTraj(MecanumDrive mecanumDrive, Pose2d initialPose) {
            return mecanumDrive.actionBuilder(initialPose)
                    .strafeToSplineHeading(new Vector2d(-3.5, 32), Math.toRadians(270));
        }

        /// Uses the kicker in the front of the bot to kick the samples into the obv zones
        public TrajectoryActionBuilder kickSample1Traj(MecanumDrive mecanumDrive, Pose2d initialPose) {
            return mecanumDrive.actionBuilder(initialPose)
                    .strafeTo(new Vector2d(-7.5, 42))
                    .strafeToLinearHeading(new Vector2d(-34, 25), Math.toRadians(0))

                    .strafeTo(new Vector2d(-36, 25));
        }

        public TrajectoryActionBuilder kickSample2Traj(MecanumDrive mecanumDrive, Pose2d initialPose) {
            return mecanumDrive.actionBuilder(initialPose)
                    .strafeTo(new Vector2d(-43, 25));
        }

        public TrajectoryActionBuilder kickSample3Traj(MecanumDrive mecanumDrive, Pose2d initialPose) {
            return mecanumDrive.actionBuilder(initialPose)
                    .strafeTo(new Vector2d(-54, 25));
        }

        //The only trajectory that doesn't need to have multiple trajectories, as it needs to constantly go to the same spot
        public TrajectoryActionBuilder wallSpecimenTraj(MecanumDrive mecanumDrive, Pose2d initialPose) {
            return mecanumDrive.actionBuilder(initialPose)

                    .strafeToLinearHeading(new Vector2d(45, -50), Math.toRadians(90))
                    .strafeTo(new Vector2d(-45, -60));
        }



///Deprecated due to design changes
//        public TrajectoryActionBuilder redThreeYellows(MecanumDrive mecanumDrive, Pose2d initialPose) {
//            return  mecanumDrive.actionBuilder(initialPose)
//                    .setTangent(0)
//                    // Line up with first yellow
//                    .splineToConstantHeading(new Vector2d(-34, -25.6), 0)
//                    // First Yellow
//                    .turnTo(Math.PI)
//                    .lineToXLinearHeading(-47, Math.PI)
//                    // Intake Program (Add your intake logic here)
//                    .waitSeconds(1) // Example: Wait for 1 second for intake
//                    // Basket Drive (FIX)
//                    .lineToXConstantHeading(-43)
//                    .splineToLinearHeading(new Pose2d(-45, -45, 8 * Math.PI / 6), 0)
//                    .strafeToConstantHeading(new Vector2d(-60, -60))
//                    // Outtake Program (Add your outtake logic here)
//                    .waitSeconds(1) // Example: Wait for 1 second for outtake
//                    // Second Yellow
//                    .strafeToConstantHeading(new Vector2d(-50, -45))
//                    .splineTo(new Vector2d(-45, -25.6), 180)
//                    .turnTo(Math.PI)
//                    .lineToXLinearHeading(-58.3, Math.PI)
//                    // Intake Program (Add your intake logic here)
//                    .waitSeconds(1) // Example: Wait for 1 second for intake
//                    // Basket Drive
//                    .lineToXConstantHeading(-50)
//                    .splineToLinearHeading(new Pose2d(-45, -45, 8 * Math.PI / 6), 0)
//                    .strafeToConstantHeading(new Vector2d(-60, -60))
//                    // Outtake Program (Add your outtake logic here)
//                    .waitSeconds(1) // Example: Wait for 1 second for outtake
//                    // Third Yellow
//                    .strafeToConstantHeading(new Vector2d(-45, -45))
//                    .splineTo(new Vector2d(-55, -25.6), 0)
//                    .turnTo(Math.PI)
//                    .lineToXLinearHeading(-67.3, Math.PI)
//                    // Intake Program (Add your intake logic here)
//                    .waitSeconds(1) // Example: Wait for 1 second for intake
//                    // Basket Drive
//                    .lineToXConstantHeading(-65)
//                    .splineToLinearHeading(new Pose2d(-45, -45, 8 * Math.PI / 6), 0)
//                    .strafeToConstantHeading(new Vector2d(-60, -60))
//                    // Outtake Program (Add your outtake logic here)
//                    .waitSeconds(1);
//        }

    }

