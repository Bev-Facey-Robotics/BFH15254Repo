package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Robot Manual Control", group = "Competition Ready")
public class RobotManualControl extends DeepHorOpMode {
    //region Hardware

    public double armSmallHorizontal = 0.52;
    public double armVertical = 0.57;
    //endregion

    // Targets
    public double verticalTarget = -4;

    // Held down checks
    private boolean isBackheld = false;

    // Bot Config
    double deadzone = 0.1;

    @Override
    public void runOpMode() {
        initOpMode();
        waitForStart();
        while (opModeIsActive()) {
            loopOpMode();
        }
    }

    //region Initialization
    public void initOpMode() {
        ConfigureHardware(true);
        if (!CrossOpModeData.isInitialized) {
            BotInitialization.InitializeRobot(this);
            CrossOpModeData.isInitialized = true;
        }

        telemetry.addLine("Ready to Start");
        telemetry.update();
    }
    //endregion

    //region Primary Loop
    public void loopOpMode() {
        //region Drive
        double speed = 1.0;
        if (gamepad1.left_trigger > 0.15) {
            speed = 0.5;
        }
        if (gamepad1.right_trigger > 0.15) {
            speed = 0.25;
        }


        // Apply deadzone to joystick inputs
        double drive = Math.abs(gamepad1.left_stick_y) > deadzone ? -gamepad1.left_stick_y : 0;
        double strafe = Math.abs(gamepad1.left_stick_x) > deadzone ? -gamepad1.left_stick_x : 0;
        double turn = Math.abs(gamepad1.right_stick_x) > deadzone ? -gamepad1.right_stick_x : 0;

        MoveRobot(drive, strafe, turn);
        UpdateMoveRobot(speed);
        //endregion

        //region Controller 2
        boolean isSlowModeActive = !gamepad2.left_bumper;

        //region Collection Arm / First Stage

        if (!AssistRunning) {
            // Scoop
            double armScoop = (gamepad2.b ? 1 : 0) + (gamepad2.a ? -1 : 0);
            MoveArmScoop(armScoop);

            // First stage bucket
            if (gamepad2.x) {
                stage1Arm.setTargetPosition(5);
                stage1Arm.setPower(0.1);
            }
            if (gamepad2.y) {
                stage1Arm.setTargetPosition(255);
                stage1Arm.setPower(0.3);
            }

            // First stage bucket power control
            if (stage1Arm.getTargetPosition() == 5) {
                if (stage1Arm.getCurrentPosition() < 30) {
                    stage1Arm.setPower(0);
                } else {
                    stage1Arm.setPower(0.1);
                }
            }
        }
        //endregion

        //region Slide / Second Stage
        // Slide

        if (!AssistRunning) {
            double slidePower = gamepad2.right_trigger - gamepad2.left_trigger;
            MoveSlidePwr(slidePower);

            // Second stage arm
            if (gamepad2.dpad_down) {
                verticalTarget = -11;
            } else if (gamepad2.dpad_up) {
                verticalTarget = -130;
            }
            stage2Swing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            stage2Swing.setTargetPosition((int) verticalTarget);
            stage2Swing.setPower(0.5);

            // Second stage bucket
            bucketTargetPosition = -0.1;
            if (gamepad2.right_bumper) {
                bucketTargetPosition = 0.2;
            }
            if (gamepad2.left_bumper) {
                bucketTargetPosition = 0;
            }


        }
        // Bucket syncing (since this moves!)
        if (!stage2BucketSync) {
            StartBucketSync();
        }

        //endregion

        //region Arm Assist
        if (gamepad2.share || gamepad2.back) {
            if (!isBackheld) {
                isBackheld = true;
                if (AssistRunning) {
                    StopPieceAssist();
                } else {
                    StartPieceAssist();
                }
            }
        } else {
            isBackheld = false;
        }
        //endregion

        //endregion

        //region Telemetry
        telemetry.addLine("Slide Position " + motorSlide.getCurrentPosition());
        telemetry.addLine("Swing Position " + stage2Swing.getCurrentPosition());
        telemetry.addData("Bucket Position", stage2Bucket.getPosition());
        telemetry.addData("Assist", AssistRunning);
//        telemetry.addLine("Top Bucket Position (servo) " + stage2Bucket.getPosition());
//        telemetry.addLine("Bucket Target Position (servo)" + bucketTargetPosition);

        telemetry.addLine("Arm Vertical: " + stage1Arm.getCurrentPosition());

        telemetry.update();
        //endregion
    }
    //endregion

}

