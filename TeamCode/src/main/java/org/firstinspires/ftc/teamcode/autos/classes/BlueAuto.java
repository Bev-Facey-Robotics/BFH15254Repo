package org.firstinspires.ftc.teamcode.autos.classes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class BlueAuto extends MainAuto {
    public TrajectoryActionBuilder parkingRun(MecanumDrive mecanumDrive, Pose2d initialPose) {
        return mecanumDrive.actionBuilder(initialPose)
                .splineTo(new Vector2d(33.74, -47.38), Math.toRadians(18.36))
                .splineTo(new Vector2d(52.28, -62.51), Math.toRadians(-39.21));
    }
}
