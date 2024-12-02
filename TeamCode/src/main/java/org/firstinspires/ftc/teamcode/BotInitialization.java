package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class BotInitialization {

    public static void InitializeRobot(DeepHorOpMode robot) {
        RunInitialization(robot);
    }
    private static void RunInitialization(DeepHorOpMode robot) {
        robot.motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        robot.arm_SmallHorizontal.setPosition(0.8);
//        robot.arm_VerticalServo.setPosition(0.5);
//        robot.telemetry.addLine("Getting Calibration Data...");
//        robot.telemetry.update();
//        robot.arm_BigHorizontal.setPower(0.2);
//        robot.motorSwing.setPower(0.2);
//        try {
//            Thread.sleep(3000);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        robot.arm_SmallHorizontal.setPosition(0.52);
//        robot.arm_VerticalServo.setPosition(0.57);
//        robot.arm_BigHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.motorSwing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.armTarget = -127;
//        robot.arm_BigHorizontal.setTargetPosition(robot.armTarget);
//        robot.arm_BigHorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.arm_BigHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.arm_BigHorizontal.setPower(0.2);
//
        robot.arm_Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm_Vertical.setTargetPosition(-4);
        robot.arm_Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        while (robot.arm_BigHorizontal.isBusy()) {
//            robot.telemetry.addLine("Calibrating Arm");
//            robot.telemetry.addData("Position", robot.arm_BigHorizontal.getCurrentPosition());
//            robot.telemetry.update();
//        }
        robot.telemetry.addLine("Ready to Start");
        robot.telemetry.update();
    }
}
