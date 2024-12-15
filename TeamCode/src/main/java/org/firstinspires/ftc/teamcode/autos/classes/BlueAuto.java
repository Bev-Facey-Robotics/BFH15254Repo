package org.firstinspires.ftc.teamcode.autos.classes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class BlueAuto extends MainAuto {
    public TrajectoryActionBuilder parkingRun(MecanumDrive mecanumDrive, Pose2d initialPose) {
        return mecanumDrive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-14.34, 43.90), Math.toRadians(269.57))
                .splineToConstantHeading(new Vector2d(-50.16, 57.73), Math.toRadians(-83.83));

    }
}
