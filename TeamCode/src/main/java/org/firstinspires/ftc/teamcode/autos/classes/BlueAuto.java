package org.firstinspires.ftc.teamcode.autos.classes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.internal.BaseOpMode;


    public class BlueAuto extends RedAuto {

        @Override
        public Pose2d importedinitialPose() {
            return new Pose2d(23,-55, Math.toRadians(90));}

    }

