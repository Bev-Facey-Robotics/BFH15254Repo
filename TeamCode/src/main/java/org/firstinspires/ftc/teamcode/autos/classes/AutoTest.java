package org.firstinspires.ftc.teamcode.autos.classes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class AutoTest extends LinearOpMode {


    @Override
    public void runOpMode() {
         Pose2d initialPose = new Pose2d(-10,-50,Math.toRadians(90) );
         MecanumDrive drive = new MecanumDrive(hardwareMap,initialPose);

        TrajectoryActionBuilder testTraj = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);


        waitForStart();// We shouldn't need this, but better to be safe than sorry!

        Action trajChose = testTraj.build();
        Actions.runBlocking(
                new SequentialAction(trajChose)
        );




    }
}
