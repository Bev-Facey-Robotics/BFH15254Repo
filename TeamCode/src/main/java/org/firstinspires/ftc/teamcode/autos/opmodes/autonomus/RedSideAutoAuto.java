package org.firstinspires.ftc.teamcode.autos.opmodes.autonomus;

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



import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.autos.classes.RedAuto;

@Autonomous(name="Auto: Red Side", group = "Competition Ready")
public class RedSideAutoAuto extends RedAuto {

    private Pose2d myInitialPose;
    public static Pose2d initialPose = new Pose2d(0,0,0);

    public RedSideAutoAuto() {
        initialPose =  (Pose2d) recivePose(initialPose);
    }

    public Pose2d recivePose(Pose2d initialPose) {
        return super.recivePose(initialPose);
    }


    public void runOpMode() {
        // Access the initialPose from the parent class

        myInitialPose = (Pose2d) recivePose(initialPose);

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
        waitForStart();


        Actions.runBlocking(
                new SequentialAction(
                        ThreeYellows
                )
        );
    }
}