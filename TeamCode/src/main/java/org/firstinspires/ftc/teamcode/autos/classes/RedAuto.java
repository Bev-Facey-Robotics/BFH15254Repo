package org.firstinspires.ftc.teamcode.autos.classes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.AprilTagPosFinder;
import org.firstinspires.ftc.teamcode.autos.classes.MainAuto;
import org.firstinspires.ftc.teamcode.AprilTagPosFinder;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.apriltag.AprilTagPose;


public class RedAuto extends MainAuto {
        //Go get initialpose, boi
        public Pose2d initialPose = super.initialPose;



     TrajectoryActionBuilder ThreeYellows = mecanumDrive.actionBuilder(initialPose)
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


    public void runOpMode(){


        Action threeYellowActions = ThreeYellows(mecanumDrive, initialPose).build();

        waitForStart();

        Actions.runBlocking(threeYellowActions);
    }

}



