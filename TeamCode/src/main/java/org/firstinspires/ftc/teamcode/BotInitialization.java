package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class BotInitialization {

    public static void InitializeRobot(DeepHorOpMode robot) {
        RunInitialization(robot);
    }
    private static void RunInitialization(DeepHorOpMode robot) {

        robot.stage1Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.stage1Arm.setPower(-0.8);

        robot.stage2Swing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.stage1Arm.setPower(-0.3);
        robot.stage2Swing.setPower(0.2);
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        // vertical arm
        robot.stage1Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.stage1Arm.setPower(0);
        robot.stage1Arm.setTargetPosition(0);
        robot.stage1Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // swing
        robot.stage2Swing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.stage2Swing.setPower(0);
        robot.stage2Swing.setTargetPosition(0);
        robot.stage2Swing.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //slide
        robot.motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorSlide.setTargetPosition(0);
        robot.motorSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        if (robot.slideLimit1.isPressed() || robot.slideLimit2.isPressed()) {
//            robot.motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        } else {
//            robot.motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.motorSlide.setPower(1);
//            while (!(robot.slideLimit1.isPressed() || robot.slideLimit2.isPressed())) {
//                robot.telemetry.addLine("Calibrating Slide");
//                robot.telemetry.addData("Position", robot.motorSlide.getCurrentPosition());
//                robot.telemetry.addData("Limit 1", robot.slideLimit1.isPressed());
//                robot.telemetry.addData("Limit 2", robot.slideLimit2.isPressed());
//                robot.telemetry.update();
//            }
            robot.motorSlide.setPower(0);
            robot.motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

