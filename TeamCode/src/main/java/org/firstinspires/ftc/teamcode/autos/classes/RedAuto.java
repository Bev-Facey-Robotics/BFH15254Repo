package org.firstinspires.ftc.teamcode.autos.classes;

import com.acmerobotics.roadrunner.Pose2d;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.internal.BaseOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.autos.classes.MainAuto;



    public class RedAuto extends MainAuto {


        public TrajectoryActionBuilder parkRun(MecanumDrive mecanumDrive, Pose2d initialPose) {
            return mecanumDrive.actionBuilder(initialPose)
                    .setTangent(0)
                    .waitSeconds(25)
                    .strafeToLinearHeading(new Vector2d(46,-65), Math.toRadians(90));
        }


        public TrajectoryActionBuilder SpeciAuton(MecanumDrive mecanumDrive, Pose2d initialPose) {
            return mecanumDrive.actionBuilder(initialPose)
                    ///When Actions are implemented, this auto should be able to do a 50 point auto on red
                    .setTangent(0)
                    ///Score preloaded speci on high chamber
                    .strafeToSplineHeading(new Vector2d(7.5,-32), Math.toRadians(270))
                    //.Action.SlideSpecScore
                    .waitSeconds(0.5)

                    .strafeTo(new Vector2d(7.5,-36))



                    ///Sample Sweep into obv zone
                    .strafeTo(new Vector2d(7.5,-42))
                    .strafeToLinearHeading(new Vector2d(34, -25), Math.toRadians(0))
                    .turnTo(Math.toRadians(0))
                    .strafeTo(new Vector2d(36, -25)) //Add VelConstraints so piece isn't rammed into
                    //.Action.Sweep
                    .waitSeconds(1)
                    .strafeTo(new Vector2d(43,-25))
                    .waitSeconds(1)
                    //.Action.Sweep
                    .strafeTo(new Vector2d(54,-25))
                    .waitSeconds(1)
                    //.Action.Sweep
                    .waitSeconds(1)
                    ///Obv Run
                    .strafeToLinearHeading(new Vector2d(45,-50), Math.toRadians(90))
                    .strafeTo(new Vector2d(45,-60))

                    ///HighChamber Run
                    .strafeToSplineHeading(new Vector2d(6.5,-32), Math.toRadians(270))
                    //.Action.SlideSpecScore
                    .waitSeconds(0.5)

                    .strafeTo(new Vector2d(6.5,-36))
                    ///Obv Run
                    .strafeToLinearHeading(new Vector2d(45,-50), Math.toRadians(90))
                    .strafeTo(new Vector2d(45,-60))

                    ///HighChamber Run
                    .strafeToSplineHeading(new Vector2d(5.5,-32), Math.toRadians(270))
                    //.Action.SlideSpecScore
                    .waitSeconds(0.5)

                    .strafeTo(new Vector2d(5.5,-36))
                    ///Obv Run
                    .strafeToLinearHeading(new Vector2d(45,-50), Math.toRadians(90))
                    //.Action.SlideSpecGrab
                    .strafeTo(new Vector2d(45,-60))


                    ///HighChamber Run
                    .strafeToSplineHeading(new Vector2d(4.5,-32), Math.toRadians(270))
                    //.Action.SlideSpecScore
                    .waitSeconds(0.5)

                    .strafeTo(new Vector2d(4.5,-36))

                    ///Obv Run
                    .strafeToLinearHeading(new Vector2d(45,-50), Math.toRadians(90))
                    //.Action.SlideSpecGrab
                    .strafeTo(new Vector2d(45,-60))

                    ///HighChamber Run
                    .strafeToSplineHeading(new Vector2d(3.5,-32), Math.toRadians(270))
                    //.Action.SlideSpecScore
                    .waitSeconds(0.5);
        }

        ///Deprecated due to design changes
        public TrajectoryActionBuilder redThreeYellows(MecanumDrive mecanumDrive, Pose2d initialPose) {
            return  mecanumDrive.actionBuilder(initialPose)
                    .setTangent(0)
                    // Line up with first yellow
                    .splineToConstantHeading(new Vector2d(-34, -25.6), 0)
                    // First Yellow
                    .turnTo(Math.PI)
                    .lineToXLinearHeading(-47, Math.PI)
                    // Intake Program (Add your intake logic here)
                    .waitSeconds(1) // Example: Wait for 1 second for intake
                    // Basket Drive (FIX)
                    .lineToXConstantHeading(-43)
                    .splineToLinearHeading(new Pose2d(-45, -45, 8 * Math.PI / 6), 0)
                    .strafeToConstantHeading(new Vector2d(-60, -60))
                    // Outtake Program (Add your outtake logic here)
                    .waitSeconds(1) // Example: Wait for 1 second for outtake
                    // Second Yellow
                    .strafeToConstantHeading(new Vector2d(-50, -45))
                    .splineTo(new Vector2d(-45, -25.6), 180)
                    .turnTo(Math.PI)
                    .lineToXLinearHeading(-58.3, Math.PI)
                    // Intake Program (Add your intake logic here)
                    .waitSeconds(1) // Example: Wait for 1 second for intake
                    // Basket Drive
                    .lineToXConstantHeading(-50)
                    .splineToLinearHeading(new Pose2d(-45, -45, 8 * Math.PI / 6), 0)
                    .strafeToConstantHeading(new Vector2d(-60, -60))
                    // Outtake Program (Add your outtake logic here)
                    .waitSeconds(1) // Example: Wait for 1 second for outtake
                    // Third Yellow
                    .strafeToConstantHeading(new Vector2d(-45, -45))
                    .splineTo(new Vector2d(-55, -25.6), 0)
                    .turnTo(Math.PI)
                    .lineToXLinearHeading(-67.3, Math.PI)
                    // Intake Program (Add your intake logic here)
                    .waitSeconds(1) // Example: Wait for 1 second for intake
                    // Basket Drive
                    .lineToXConstantHeading(-65)
                    .splineToLinearHeading(new Pose2d(-45, -45, 8 * Math.PI / 6), 0)
                    .strafeToConstantHeading(new Vector2d(-60, -60))
                    // Outtake Program (Add your outtake logic here)
                    .waitSeconds(1);
        }






    }

