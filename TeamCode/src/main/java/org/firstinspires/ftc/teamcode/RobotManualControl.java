package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Robot Manual Drive", group = "Concept")
public class RobotManualControl extends OpMode {
    //region Hardware
    //region Drive Motors
    private DcMotor motorFL = null; // Front Left
    private DcMotor motorFR = null; // Front Right
    private DcMotor motorBL = null; // Back Left
    private DcMotor motorBR = null; // Back Right
    //endregion

    //region Slide
    private DcMotor motorSlide = null;
    private DcMotor motorSwing = null;
    private Servo clawServo = null;
    //endregion
    //region Arm
    /**
     * Motor attached to the bot to move the arm containing the main arm mechanism.
     */
    private DcMotor arm_BigHorizontal = null;
    /**
     * The first servo on the arm. Swivels the main scoop mechanism off the arm.
     */
    private Servo arm_SmallHorizontal = null;
    /**
     * The second servo on the arm. Moves the rest of the scoop up and down.
     */
    private Servo arm_VerticalServo = null;
    /**
     * The third servo on the arm. Controls the wheel to scoop the game pieces.
     */
    private CRServo arm_Scoop = null;

    private double armSmallHorizontal = 0.5;
    private double armVertical = 0.5;
    //endregion
    //endregion

    // Current Drive setting
    // This is here so  I can be lazy and reuse some automode code
    // TODO: Later implement this in a different class to practice DRY, and make some things easier
    private double drivePwr = 0;
    private double strafePwr = 0;
    private double yawPwr = 0;

    // Targets
    private int swingTarget = 0;
    private boolean isClipped = false;
    private boolean lastControllerState = false;

//region Initialization
    @Override
    public void init() {
        ConfigureHardware();
        arm_SmallHorizontal.setPosition(0.8);
        arm_VerticalServo.setPosition(0.5);
        telemetry.addLine("Getting Calibration Data...");
        telemetry.update();
        arm_BigHorizontal.setPower(0.2);
        try {
            Thread.sleep(4000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        arm_BigHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_BigHorizontal.setTargetPosition(-140);
        arm_BigHorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_BigHorizontal.setPower(1);
        while (arm_BigHorizontal.isBusy()) {
            telemetry.addLine("Calibrating Arm");
            telemetry.addData("Position", arm_BigHorizontal.getCurrentPosition());
            telemetry.update();
        }
        arm_BigHorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        motorSwing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSwing.setTargetPosition(0);
        motorSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        if (gamepad1.right_bumper) {
            speed = 0.5;
        }
        MoveRobot(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        moveRobotInternal(speed);
        //endregion

        //region Arm
        // -131 to -10
        double armPower = gamepad2.left_stick_x * 0.2;
        arm_BigHorizontal.setPower(armPower);

        armSmallHorizontal += -gamepad2.right_stick_x * 0.01;
        armSmallHorizontal = Math.max(-1, Math.min(1, armSmallHorizontal));
        arm_SmallHorizontal.setPosition(armSmallHorizontal);

        armVertical += -gamepad2.right_stick_y * 0.01;
        armVertical = Math.max(-1, Math.min(1, armVertical));
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
        if (motorSlide.getCurrentPosition() <= -8750 && slidePower < 0) { // Limit is -8950
            // Prevent the motor from moving further negative
            slidePower = 0;
        } else if (motorSlide.getCurrentPosition() >= 0 && slidePower > 0) {
            // Prevent the motor from moving further positive
            slidePower = 0;
        }
        motorSlide.setPower(slidePower);

        // Swing
        // swing 0 to -113
        swingTarget += ((gamepad2.dpad_up ? 1 : 0) - (gamepad2.dpad_down ? 1 : 0)) * 2;

        // Adjust target based on position limits
        if (swingTarget <= -110) {
            // Prevent the motor from moving further negative
            swingTarget = -109;
        } else if (swingTarget >= 0) {
            // Prevent the motor from moving further positive
            swingTarget = -1;
        }

        motorSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSwing.setTargetPosition(swingTarget);
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

        // Swing
        //motorSwing.setTargetPosition(gamepad2.left_stick_y * );
        telemetry.addLine("Swing: " + motorSwing.getCurrentPosition());

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
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        motorFL.setPower(leftFrontPower);
        motorFR.setPower(rightFrontPower);
        motorBL.setPower(leftBackPower);
        motorBR.setPower(rightBackPower);
    }
}

