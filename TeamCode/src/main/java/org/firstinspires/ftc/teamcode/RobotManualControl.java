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
//        //region Drive
//        double leftStickY = -gamepad1.left_stick_y;
//        double leftStickX = gamepad1.left_stick_x;
//        double rightStickX = gamepad1.right_stick_x;
//
//        double fl = leftStickY + leftStickX + rightStickX;
//        double fr = leftStickY - leftStickX - rightStickX;
//        double bl = leftStickY - leftStickX + rightStickX;
//        double br = leftStickY + leftStickX - rightStickX;
//
//        motorFL.setPower(fl);
//        motorFR.setPower(fr);
//        motorBL.setPower(bl);
//        motorBR.setPower(br);
//        //endregion

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
}
