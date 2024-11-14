package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Robot Manual Drive", group = "Concept")
public class RobotManualControl extends OpMode {
    //region Hardware
    //region Drive Motors
    private DcMotor motorFL = null; // Front Left
    private DcMotor motorFR = null; // Front Right
    private DcMotor motorBL = null; // Back Left
    private DcMotor motorBR = null; // Back Right
    //endregion
    private DcMotor motorSlide = null;
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
    //endregion
    //endregion

    // Current Drive setting
    // This is here so I can be lazy and reuse some automode code
    // TODO: Later implement this in a different class to practice DRY, and make some things easier
    private double drivePwr = 0;
    private double strafePwr = 0;
    private double yawPwr = 0;

//region Initialization
    @Override
    public void init() {
        ConfigureHardware();
        arm_SmallHorizontal.setPosition(0.8);
        arm_VerticalServo.setPosition(0.5);
        telemetry.addLine("hello world!");
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

        motorSlide = hardwareMap.get(DcMotor.class, "slideMotor");

        motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        MoveRobot(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        moveRobotInternal();
        //endregion

        //region Arm
        double armPower = gamepad2.left_stick_x * 0.5;
        arm_BigHorizontal.setPower(armPower);

        double armSmallHorizontal = -gamepad2.right_stick_x + 0.5;
        arm_SmallHorizontal.setPosition(armSmallHorizontal);

        double armVertical = gamepad2.right_stick_y + 0.5;
        arm_VerticalServo.setPosition(armVertical);

        double armScoop = gamepad2.left_stick_y;
        arm_Scoop.setPower(armScoop);

        // Get our arm data

        telemetry.addLine("Arm data");
        telemetry.addLine("Small Horizonal " + arm_SmallHorizontal.getPosition());
        telemetry.addLine("Vertical " + arm_VerticalServo.getPosition());

        telemetry.addLine("Slide Position " + motorSlide.getCurrentPosition());
//        telemetry.addLine("Scoop " + a;

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

        //endregion
        telemetry.update();
    }
    //endregion
    public void MoveRobot(double x, double y, double yaw) {
        drivePwr = x;
        strafePwr = y;
        yawPwr = yaw;
    }

    private void moveRobotInternal() {
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

