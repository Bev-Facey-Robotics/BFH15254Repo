package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Robot Manual Control", group = "Competition Ready")
public class RobotManualControl extends DeepHorOpMode {
    //region Hardware

    public double armSmallHorizontal = 0.52;
    public double armVertical = 0.57;
    //endregion
    //endregion


    // Targets
    public double verticalTarget = -4;
    private boolean lastControllerStateX = false;
    private boolean altSwingMode = false;

    // Bot Config
    double deadzone = 0.1;

//region Initialization
    @Override
    public void init() {
        ConfigureHardware();
        if (!CrossOpModeData.isInitialized) {
            BotInitialization.InitializeRobot(this);
            CrossOpModeData.isInitialized = true;
        }

        telemetry.addLine("Ready to Start");
        telemetry.update();


    }
    //endregion

    //region Primary Loop
    @Override
    public void loop() {
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

        boolean isSlowModeActive = !gamepad2.left_bumper;

        //region Arm
        // -131 to -10
//
//
//        if (gamepad2.back) {
//            if (!lastControllerState2) {
//                altArmMode = !altArmMode;
//                lastControllerState2 = true;
//            }
//        } else {
//            lastControllerState2 = false;
//        }
//
//        if (altArmMode) {
//            double armPower = gamepad2.left_stick_x * (isSlowModeActive ? 0.2 : 0.5);
//            arm_BigHorizontal.setPower(armPower);
//            if (arm_BigHorizontal.getMode() != DcMotor.RunMode.RUN_USING_ENCODER ||
//                    arm_BigHorizontal.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE) {
//
//                arm_BigHorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                arm_BigHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            }
//        } else {
//            if (gamepad2.left_stick_x != 0) {
//                armTarget = (arm_BigHorizontal.getCurrentPosition() + (int) (gamepad2.left_stick_x * (isSlowModeActive ? 10 : 20)));
//            }
//            // Adjust target based on position limits
//            if (armTarget <= -130) {
//                // Prevent the motor from moving further negative
//                armTarget = -125;
//            } else if (armTarget >= -10) {
//                // Prevent the motor from moving further positive
//                armTarget = -15;
//            }
//
//            if (arm_BigHorizontal.getMode() != DcMotor.RunMode.RUN_TO_POSITION ||
//                    arm_BigHorizontal.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE) {
//
//                arm_BigHorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                arm_BigHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            }
//            arm_BigHorizontal.setPower(1);
//            arm_BigHorizontal.setTargetPosition(armTarget);
//        }
//
//        armSmallHorizontal += -gamepad2.right_stick_x * (isSlowModeActive ? .01 : .03);
//        armSmallHorizontal = Math.max(0.35, Math.min(0.85, armSmallHorizontal));
//        arm_SmallHorizontal.setPosition(armSmallHorizontal);
//
//        armVertical += -gamepad2.right_stick_y * (isSlowModeActive ? .01 : .03);
//        armVertical = Math.max(0.15, Math.min(0.75, armVertical));
//        arm_VerticalServo.setPosition(armVertical);

         double armScoop = (gamepad2.b ? 1 : 0) + (gamepad2.a ? -1 : 0);
         MoveArmScoop(armScoop);

        // To be repurposed for new scoop on thingyamajing
        verticalTarget += ((gamepad2.dpad_down ? 1 : 0) - (gamepad2.dpad_up ? 1 : 0)) * (isSlowModeActive ? 1 : 3);

            // Adjust target based on position limits
            if (verticalTarget < -130) {
                // Prevent the motor from moving further negative
                verticalTarget = -129;
            } else
            if (verticalTarget > 1) {
                // Prevent the motor from moving further positive
                verticalTarget = 0;
            }
//        } else {
            // Normal mode
            if (gamepad2.dpad_left) {
                verticalTarget = -11;
            } else if (gamepad2.dpad_right) {
                verticalTarget = -130;

            }
            arm_Vertical.setTargetPosition((int) verticalTarget);

//
//        // Arm telemetry
//        telemetry.addLine("Arm data");
//        telemetry.addLine("Big Horizontal " + arm_BigHorizontal.getCurrentPosition());
//        telemetry.addLine("Small Horizonal " + arm_SmallHorizontal.getPosition());
//        telemetry.addLine("Vertical " + arm_VerticalServo.getPosition());

        //endregion




        //region Slide
        double slidePower = gamepad2.right_trigger - gamepad2.left_trigger;
        MoveSlidePwr(slidePower);

        if (gamepad2.start) {
            if (!lastControllerStateX) {
                altSwingMode = !altSwingMode;
                lastControllerStateX = true;
            }
        } else {
            lastControllerStateX = false;
        }

//        if (altSwingMode) {
            // Swing
            // swing 0 to -113



//
//        motorSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorSwing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        motorSwing.setPower(1);
//
//        telemetry.addLine("Slide Position " + motorSlide.getCurrentPosition());
//        telemetry.addLine("Swing Target" + swingTarget);
//        telemetry.addLine("Swing Position " + motorSwing.getCurrentPosition());


        //endregion
        telemetry.addLine("X Odometer: " + motorFR.getCurrentPosition());
        telemetry.addLine("Y Odometer: " + motorFL.getCurrentPosition());

        telemetry.update();
    }
    //endregion

}

