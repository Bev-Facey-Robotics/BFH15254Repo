package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Robot Manual Drive (mod)", group = "Competition Ready")
public class RobotManualControl extends OpMode {
    //region Hardware
    //region Drive Motors
    public DcMotor motorFL = null; // Front Left
    public DcMotor motorFR = null; // Front Right
    public DcMotor motorBL = null; // Back Left
    public DcMotor motorBR = null; // Back Right
    //endregion

    //region Slide
    public DcMotor motorSlide = null;
    public DcMotor motorSwing = null;
    public Servo clawServo = null;
    //endregion
    //region Arm
    /**
     * Motor attached to the bot to move the arm containing the main arm mechanism.
     */
    public DcMotor arm_BigHorizontal = null;
    /**
     * The first servo on the arm. Swivels the main scoop mechanism off the arm.
     */
    public Servo arm_SmallHorizontal = null;
    /**
     * The second servo on the arm. Moves the rest of the scoop up and down.
     */
    public Servo arm_VerticalServo = null;
    /**
     * The third servo on the arm. Controls the wheel to scoop the game pieces.
     */
    public CRServo arm_Scoop = null;

    public double armSmallHorizontal = 0.52;
    public double armVertical = 0.57;
    //endregion
    //endregion

    // Current Drive setting
    // This is here so  I can be lazy and reuse some automode code
    // TODO: Later implement this in a different class to practice DRY, and make some things easier
    private double drivePwr = 0;
    private double strafePwr = 0;
    private double yawPwr = 0;

    // Targets
    public double swingTarget = -4;
    public int armTarget = 0;
    private boolean isClipped = false;
    private boolean altArmMode = false;
    private boolean lastControllerState = false;
    private boolean lastControllerState2 = false;
    private boolean lastControllerStateX = false;
    private boolean altSwingMode = false;

    // Bot Config
    double deadzone = 0.1;

//region Initialization
    @Override
    public void init() {
        BotInitialization.InitializeRobot(this);
        telemetry.addLine("Ready to Start");
        telemetry.update();


    }

    private void ConfigureHardware() {
        //region Drive Motors
        motorFL = hardwareMap.get(DcMotor.class, "leftFront");
        motorFR = hardwareMap.get(DcMotor.class, "rightFront");
        motorBL = hardwareMap.get(DcMotor.class, "leftBack");
        motorBR = hardwareMap.get(DcMotor.class, "rightBack");

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);

        //endregion

        //region Slide
        motorSlide = hardwareMap.get(DcMotor.class, "slideMotor");

        motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorSwing = hardwareMap.get(DcMotor.class, "swing");
        motorSwing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        clawServo = hardwareMap.get(Servo.class, "claw");


        //endregion

        //region Arm Hardware
        arm_BigHorizontal = hardwareMap.get(DcMotor.class, "bigHorizontal");
        arm_SmallHorizontal = hardwareMap.get(Servo.class, "RotHori");
        arm_VerticalServo = hardwareMap.get(Servo.class, "RotVert");
        arm_Scoop = hardwareMap.get(CRServo.class, "Spinny");


        //endregion


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
        moveRobotInternal(speed);
        //endregion

        boolean isSlowModeActive = !gamepad2.left_bumper;

        //region Arm
        // -131 to -10


        if (gamepad2.back) {
            if (!lastControllerState2) {
                altArmMode = !altArmMode;
                lastControllerState2 = true;
            }
        } else {
            lastControllerState2 = false;
        }

        if (altArmMode) {
            double armPower = gamepad2.left_stick_x * (isSlowModeActive ? 0.2 : 0.5);
            arm_BigHorizontal.setPower(armPower);
            if (arm_BigHorizontal.getMode() != DcMotor.RunMode.RUN_USING_ENCODER ||
                    arm_BigHorizontal.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE) {

                arm_BigHorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm_BigHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } else {
            if (gamepad2.left_stick_x != 0) {
                armTarget = (arm_BigHorizontal.getCurrentPosition() + (int) (gamepad2.left_stick_x * (isSlowModeActive ? 10 : 20)));
            }
            // Adjust target based on position limits
            if (armTarget <= -130) {
                // Prevent the motor from moving further negative
                armTarget = -125;
            } else if (armTarget >= -10) {
                // Prevent the motor from moving further positive
                armTarget = -15;
            }

            if (arm_BigHorizontal.getMode() != DcMotor.RunMode.RUN_TO_POSITION ||
                    arm_BigHorizontal.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE) {

                arm_BigHorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_BigHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            arm_BigHorizontal.setPower(1);
            arm_BigHorizontal.setTargetPosition(armTarget);
        }

        armSmallHorizontal += -gamepad2.right_stick_x * (isSlowModeActive ? .01 : .03);
        armSmallHorizontal = Math.max(0.35, Math.min(0.85, armSmallHorizontal));
        arm_SmallHorizontal.setPosition(armSmallHorizontal);

        armVertical += -gamepad2.right_stick_y * (isSlowModeActive ? .01 : .03);
        armVertical = Math.max(0.15, Math.min(0.75, armVertical));
        arm_VerticalServo.setPosition(armVertical);

        double armScoop = (gamepad2.b ? 1 : 0) + (gamepad2.a ? -1 : 0);
        arm_Scoop.setPower(armScoop);

        // Arm telemetry
        telemetry.addLine("Arm data");
        telemetry.addLine("Big Horizontal " + arm_BigHorizontal.getCurrentPosition());
        telemetry.addLine("Small Horizonal " + arm_SmallHorizontal.getPosition());
        telemetry.addLine("Vertical " + arm_VerticalServo.getPosition());

        //endregion




        //region Slide
        double slidePower = gamepad2.right_trigger - gamepad2.left_trigger;
        // Adjust slidePower based on position limits
        if (motorSlide.getCurrentPosition() <= -9240 && slidePower < 0) { // Limit is -8950
            // Prevent the motor from moving further negative
            slidePower = 0;
        } else
            if (motorSlide.getCurrentPosition() >= 0 && slidePower > 0) {
            // Prevent the motor from moving further positive
            slidePower = 0;
        }
        motorSlide.setPower(slidePower);

        if (gamepad2.start) {
            if (!lastControllerStateX) {
                altSwingMode = !altSwingMode;
                lastControllerStateX = true;
            }
        } else {
            lastControllerStateX = false;
        }

        if (altSwingMode) {
            // Swing
            // swing 0 to -113
            swingTarget += ((gamepad2.dpad_down ? 1 : 0) - (gamepad2.dpad_up ? 1 : 0)) * (isSlowModeActive ? 1 : 3);

            // Adjust target based on position limits
            if (swingTarget < -130) {
                // Prevent the motor from moving further negative
                swingTarget = -129;
            } else
            if (swingTarget > 1) {
                // Prevent the motor from moving further positive
                swingTarget = 0;
            }
            if (gamepad2.y) {
                swingTarget = -11;
            }
            motorSwing.setTargetPosition((int)swingTarget);
        } else {
            // Normal mode
            if (gamepad2.dpad_down) {
                swingTarget = -11;
            } else if (gamepad2.dpad_up) {
                swingTarget = -130;

            }
            motorSwing.setTargetPosition((int)swingTarget);
        }


        motorSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSwing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorSwing.setPower(1);

        telemetry.addLine("Slide Position " + motorSlide.getCurrentPosition());
        telemetry.addLine("Swing Target" + swingTarget);
        telemetry.addLine("Swing Position " + motorSwing.getCurrentPosition());

        // Clip

        if (gamepad2.right_bumper) {
            if (!lastControllerState) {
                isClipped = !isClipped;
                lastControllerState = true;
            }
        } else {
            lastControllerState = false;
        }

        clawServo.setPosition(isClipped ? 1 : 0);

        //endregion
        telemetry.update();
    }
    //endregion
    public void MoveRobot(double x, double y, double yaw) {
        drivePwr = x;
        strafePwr = y;
        yawPwr = yaw;
    }

    private void moveRobotInternal(double speedLimit) {
        // Calculate wheel powers.
        double leftFrontPower    =   drivePwr -strafePwr -yawPwr;
        double rightFrontPower   =   drivePwr +strafePwr +yawPwr;
        double leftBackPower     =  -drivePwr -strafePwr +yawPwr;
        double rightBackPower    =  -drivePwr +strafePwr -yawPwr;

        // Normalize wheel powers to be less than 1.0
        double max = (Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        motorFL.setPower(leftFrontPower*speedLimit);
        motorFR.setPower(rightFrontPower*speedLimit);
        motorBL.setPower(leftBackPower*speedLimit);
        motorBR.setPower(rightBackPower*speedLimit);
    }
}

