package org.firstinspires.ftc.teamcode.autos.opmodes.autonomus;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.classes.RedAuto;

@Autonomous(name="Auto: Red Side", group = "Competition Ready")
public class RedSideAutoAuto extends RedAuto {

    public Pose2d getInitialPose = ;
    public RedSideAutoAuto (Pose2d initialPose) {
       Pose2d getInitialPose = initialPose;

    }

    public void runOpMode()  {
        // Access the initialPose from the parent class
        Pose2d myInitialPose = getInitialPose

    }



    //            .setTangent(0)
//                    //Line up with first yellow
//
//                    .splineToConstantHeading(new Vector2d(-34,-25.6), 0)
//                    //First Yellow
//                    .turnTo(Math.PI)
//                    .lineToXLinearHeading(-47, Math.PI)
//                    //Intake Program
//
//                    //Basket Drive (FIX)
//                    .lineToXConstantHeading(-43)
//                    .splineToLinearHeading(new Pose2d(-45,-45,8*Math.PI/6),0)
//                    .strafeToConstantHeading(new Vector2d(-60,-60))
//                    //Outake Program
//
//                    //Second Yellow
//                        .strafeToConstantHeading(new Vector2d(-50,-45))
//                    .splineTo(new Vector2d(-45,-25.6), 180)
//                    .turnTo(Math.PI)
//                    .lineToXLinearHeading(-58.3, Math.PI)
//
//                    //Intake Program
//
//                    //Basket Drive
//                .lineToXConstantHeading(-50)
//                .splineToLinearHeading(new Pose2d(-45,-45,8*Math.PI/6),0)
//                .strafeToConstantHeading(new Vector2d(-60,-60))
//                    //Outake Program
//
//                    //Third Yellow
//                .strafeToConstantHeading(new Vector2d(-45,-45))
//                .splineTo(new Vector2d(-55,-25.6), 0)
//                .turnTo(Math.PI)
//                .lineToXLinearHeading(-67.3, Math.PI)
//
//                    //Intake Program
//
//                    //Basket Drive
//                .lineToXConstantHeading(-65)
//                .splineToLinearHeading(new Pose2d(-45,-45,8*Math.PI/6),0)
//                .strafeToConstantHeading(new Vector2d(-60,-60))
//                    //Outake Program
//
//                    //Parking Run


