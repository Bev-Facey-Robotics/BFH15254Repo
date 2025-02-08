package org.firstinspires.ftc.teamcode.autos.classes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "RedTest")
public class RedAuto extends MainAuto {
        //Go get initialpose, boi
        public Pose2d initialPose = new Pose2d(-10.9,-50,90);
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initialPose);



    public void runOpMode() {



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


        if (initialPose == null) {
            telemetry.addLine("Null, boi");
            telemetry.update();
        }

        Pose2d initialPose = new Pose2d(-10.9,-50,90);
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initialPose);

        if (isStopRequested()) return;

        Action trajChosen;
                trajChosen = ThreeYellows.build();

                waitForStart();

                Actions.runBlocking(
                        new SequentialAction(
                                trajChosen
                        )
                );





    }







    //Raising slide action
    public class slideUp implements Action {
        private boolean initialized = false;

        //Actions with telem
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                stage1Arm.setPower(0.8);
                initialized = true;
            }

            //Look to see where the slides are
            double SlidePos = stage1Arm.getCurrentPosition();
            packet.put("SlidePos", SlidePos);

            //Logic to make it go up until telem limits are reached
            if (SlidePos < -10000) {
                return true;
            } else {
                stage1Arm.setPower(0.5);
                return false;
            }
        }
    }

    //Method to make raising the slide easier
    public Action slideUp() {
        return new MainAuto.slideUp();
    }


    //Lowering slide action
    public class slideDown implements Action {
        private boolean initialized = false;

        //Actions with telem
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                stage1Arm.setPower(-0.8);
                initialized = true;
            }

            //Look to see where the slides are and put it on the driver station
            double SlidePos = stage1Arm.getCurrentPosition();
            packet.put("SlidePos", SlidePos);

            //Logic to make it go up until telem limits are reached
            if (SlidePos > 50) {
                return true;
            } else {
                stage1Arm.setPower(-0.5);
                return false;
            }
        }
    }

    //Method to lower the slide easier
    public Action slideDown () {
        return new MainAuto.slideDown();
    }


    //Class for raising the second stage arm + bucket, then dropping the sample
    public class bucketPlace implements Action {
        private boolean initialized = false;

        //Actions with telem
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                //Raise stage 2 arm
                double verticalTarget = -130;
                stage2Swing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                stage2Swing.setTargetPosition((int) verticalTarget);
                stage2Swing.setPower(0.5);
                //Raise stage 2 bucket
                bucketTargetPosition = 0.05;
                stage2Bucket.setPosition(bucketTargetPosition);
                sleep(500);

                //Wait for sample to be dropped into basket
                bucketTargetPosition = 0.25;
                stage2Bucket.setPosition(bucketTargetPosition);
                sleep(1500);
                initialized = true;
            }

            double piecePlace = stage2Bucket.getPosition();
            packet.put("BucketPos", piecePlace);
            if (piecePlace == 0.25) {
                return true;
            } else {
                bucketTargetPosition = 0.25;
                stage2Swing.setTargetPosition(-11);
                return false;
            }
        }
    }

    //Class for lowering the bucket
    public class bucketDown implements Action {
        private boolean initialized = false;

        //Actions with telem
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                stage2Swing.setPower(-0.1);
                initialized = true;
            }

            double bucketArmPos = stage2Swing.getCurrentPosition();
            packet.put("SwingPos", bucketArmPos);

            return false;
        }
    }


    //Class for sample intake
    public class sampleGrab implements Action {
        private boolean initialized = false;

        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                stage1Scoop.setPower(1.0);
                sleep(1500);
                stage1Scoop.setPower(0.0);
                initialized = true;
                return true;
            } else {
                return false;
            }

        }
    }

    //Method for sample intake
    public Action sampleGrab() {
        return new MainAuto.sampleGrab();
    }


    //Class for piece transfer
    public class pieceSwap implements Action {
        private boolean initialized = false;

        /// *TODO: Borrow Share functionality and put in this action
        //Actions with telem
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {


                //Bunch of telemetry info
                //Slide Pos
                double slidePos = motorSlide.getCurrentPosition();
                //S2Bucket Pos
                double S2BPos = stage2Bucket.getPosition();
                //S1Scoop Pwr
                double S1ScoopPwr = stage1Scoop.getPower();
                //S1 Arm
                double S1Arm = stage1Arm.getCurrentPosition();
                //S2Arm
                double S2Swing = stage2Swing.getCurrentPosition();

                //Moves bucket and slides down
                bucketTargetPosition = 0.05;
                stage2Bucket.setPosition(0.05);
                sleep(250);
                motorSlide.setTargetPosition(-1821);
                motorSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlide.setPower(-0.3);
                sleep(250);


                //moves the intake to transfer to the bucket
                stage1Arm.setTargetPosition(255);
                stage1Arm.setPower(0.3);

                //Transfers piece to bucket
                stage1Scoop.setPower(1.0);
                sleep(1000);
                stage1Scoop.setPower(0);

                initialized = true;

                packet.put("SlidePos", slidePos);
                packet.put("S2SwingPos", S2Swing);
                packet.put("S2BucketPos", S2BPos);
                packet.put("S1ScoopPwr", S1ScoopPwr);
                packet.put("S1ArmPos", S1Arm);

                return true;
            } else {
                return false;
            }


        }
    }



}



