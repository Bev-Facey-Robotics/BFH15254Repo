package org.firstinspires.ftc.teamcode.autos.opmodes.autonomus;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.classes.BlueAuto;

@Autonomous(name="Auto: Blue Side", group = "Competition Ready")
public class BlueSideAutoAuto extends BlueAuto {

    private Pose2d myInitialPose;

    public Object recivePose(Pose2d initialPose) {
        return super.recivePose(initialPose);
    }

    public void runOpMode() {
        // Access the initialPose from the parent class
        Pose2d initialPose = null;
        myInitialPose = (Pose2d) recivePose(initialPose);

    }
}

