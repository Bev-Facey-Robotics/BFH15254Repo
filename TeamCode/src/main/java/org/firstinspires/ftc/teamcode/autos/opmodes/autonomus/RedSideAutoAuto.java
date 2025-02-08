package org.firstinspires.ftc.teamcode.autos.opmodes.autonomus;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.sun.tools.javac.Main;

import org.firstinspires.ftc.teamcode.autos.classes.MainAuto;
import org.firstinspires.ftc.teamcode.autos.classes.RedAuto;
import org.firstinspires.ftc.teamcode.AprilTagPosFinder;
import org.firstinspires.ftc.teamcode.BotInitialization;
import org.firstinspires.ftc.teamcode.CrossOpModeData;
import org.firstinspires.ftc.teamcode.DeepHorOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.security.cert.CertPathBuilder;


@Autonomous(name = "Auto: Red Side", group = "Competition Ready")
public class RedSideAutoAuto extends RedAuto {

    Pose2d initialPose = super.initialPose;


    public void runOpMode() {


        Action threeYellowActions = ThreeYellows(mecanumDrive, initialPose).build();
        waitForStart();

        Actions.runBlocking(threeYellowActions);
    }


}