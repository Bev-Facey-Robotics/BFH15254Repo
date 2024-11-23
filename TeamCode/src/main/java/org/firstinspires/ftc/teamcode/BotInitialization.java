package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class BotInitialization {

    public static void InitializeRobot(RobotManualControl robot) {
        RunInitialization(robot);
    }
    private static void RunInitialization(RobotManualControl robot) {
        ConfigureHardware(robot);
        robot.arm_SmallHorizontal.setPosition(0.8);
        robot.arm_VerticalServo.setPosition(0.5);
        robot.telemetry.addLine("Getting Calibration Data...");
        robot.telemetry.update();
        robot.arm_BigHorizontal.setPower(0.2);
        robot.motorSwing.setPower(0.2);
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot.arm_SmallHorizontal.setPosition(0.52);
        robot.arm_VerticalServo.setPosition(0.57);
        robot.arm_BigHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorSwing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armTarget = -127;
        robot.arm_BigHorizontal.setTargetPosition(robot.armTarget);
        robot.arm_BigHorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm_BigHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.arm_BigHorizontal.setPower(0.2);

        robot.motorSwing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorSwing.setTargetPosition(-4);
        robot.motorSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.arm_BigHorizontal.isBusy()) {
            robot.telemetry.addLine("Calibrating Arm");
            robot.telemetry.addData("Position", robot.arm_BigHorizontal.getCurrentPosition());
            robot.telemetry.update();
        }
        robot.telemetry.addLine("Ready to Start");
        robot.telemetry.update();
    }

    private static void ConfigureHardware(RobotManualControl robot) {
        //region Drive Motors

        robot.motorFL = robot.hardwareMap.get(DcMotor.class, "leftFront");
        robot.motorFR = robot.hardwareMap.get(DcMotor.class, "rightFront");
        robot.motorBL = robot.hardwareMap.get(DcMotor.class, "leftBack");
        robot.motorBR = robot.hardwareMap.get(DcMotor.class, "rightBack");

        robot.motorFL.setDirection(DcMotor.Direction.REVERSE);
        robot.motorBL.setDirection(DcMotor.Direction.REVERSE);
        robot.motorFR.setDirection(DcMotor.Direction.FORWARD);
        robot.motorBR.setDirection(DcMotor.Direction.FORWARD);

        //endregion

        //region Slide
        robot.motorSlide = robot.hardwareMap.get(DcMotor.class, "slideMotor");

        robot.motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorSwing = robot.hardwareMap.get(DcMotor.class, "swing");
        robot.motorSwing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        robot.clawServo = robot.hardwareMap.get(Servo.class, "claw");


        //endregion

        //region Arm Hardware
        robot.arm_BigHorizontal = robot.hardwareMap.get(DcMotor.class, "bigHorizontal");
        robot.arm_SmallHorizontal = robot.hardwareMap.get(Servo.class, "RotHori");
        robot.arm_VerticalServo = robot.hardwareMap.get(Servo.class, "RotVert");
        robot.arm_Scoop = robot.hardwareMap.get(CRServo.class, "Spinny");


        //endregion


    }
}
