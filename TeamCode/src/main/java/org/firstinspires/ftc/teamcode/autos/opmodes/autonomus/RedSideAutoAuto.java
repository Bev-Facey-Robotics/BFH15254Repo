package org.firstinspires.ftc.teamcode.autos.opmodes.autonomus;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.sun.tools.javac.main.Main;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Trajectory;


import org.firstinspires.ftc.teamcode.BotInitialization;
import org.firstinspires.ftc.teamcode.CrossOpModeData;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.AprilTagPosFinder;
import org.firstinspires.ftc.teamcode.autos.classes.RedAuto;

@Autonomous(name="Auto: Red Side", group = "Competition Ready")
public class RedSideAutoAuto extends RedAuto {
    private AprilTagPosFinder aprilTagPosFinder = new AprilTagPosFinder();
    private MecanumDrive mecanumDrive = null; // Road Runner
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


        Pose2d initialPose = new Pose2d(aprilTagPosFinder.x, aprilTagPosFinder.y, Math.toRadians(aprilTagPosFinder.yaw));
        aprilTagPosFinder.StopStreaming();

        // for debugging
        //Pose2d initialPose = new Pose2d(0,0,Math.toRadians(aprilTagPosFinder.yaw));

        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        Action ThreeYellows = drive.actionBuilder(initialPose)
                .setTangent(0)
                //Line up with first yellow

                .splineToConstantHeading(new Vector2d(-34, -25.6), 0)
                //First Yellow
                .turnTo(Math.PI)
                .lineToXLinearHeading(-47, Math.PI)
                //Intake Program

                //Basket Drive (FIX)
                .lineToXConstantHeading(-43)
                .splineToLinearHeading(new Pose2d(-45, -45, 8 * Math.PI / 6), 0)
                .strafeToConstantHeading(new Vector2d(-60, -60))
                //Outake Program

                //Second Yellow
                .strafeToConstantHeading(new Vector2d(-50, -45))
                .splineTo(new Vector2d(-45, -25.6), 180)
                .turnTo(Math.PI)
                .lineToXLinearHeading(-58.3, Math.PI)

                //Intake Program

                //Basket Drive
                .lineToXConstantHeading(-50)
                .splineToLinearHeading(new Pose2d(-45, -45, 8 * Math.PI / 6), 0)
                .strafeToConstantHeading(new Vector2d(-60, -60))
                //Outake Program

                //Third Yellow
                .strafeToConstantHeading(new Vector2d(-45, -45))
                .splineTo(new Vector2d(-55, -25.6), 0)
                .turnTo(Math.PI)
                .lineToXLinearHeading(-67.3, Math.PI)

                //Intake Program

                //Basket Drive
                .lineToXConstantHeading(-65)
                .splineToLinearHeading(new Pose2d(-45, -45, 8 * Math.PI / 6), 0)
                .strafeToConstantHeading(new Vector2d(-60, -60))

        //Outake Program

        //Parking Run


        .build();


        Actions.runBlocking(
                new SequentialAction(
                        ThreeYellows
                )
        );
    }
}