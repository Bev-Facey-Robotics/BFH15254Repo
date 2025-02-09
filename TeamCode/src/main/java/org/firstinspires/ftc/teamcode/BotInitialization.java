package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class BotInitialization {

    public static void InitializeRobot(DeepHorOpMode robot) {
        RunInitialization(robot);
    }
    private static void RunInitialization(DeepHorOpMode robot) {
        // Calibration
        //region Stage 1 Arm & Stage 2 Swing
        // Modes
//        robot.stage1Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.stage2Swing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        // Calibration powers
//        robot.stage1Arm.setPower(-0.3);
//        robot.stage2Swing.setPower(0.2);

        // Sleep for 3 second to wait for everything to reach the limits
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        //endregion

        //region Clean up after calibration

        //region Stage 1 Arm
//        robot.stage1Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.stage1Arm.setPower(0);
//        robot.stage1Arm.setTargetPosition(0);
//        robot.stage1Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //endregion
//
//        //region Stage 2 Swing
//        robot.stage2Swing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.stage2Swing.setPower(0);
//        robot.stage2Swing.setTargetPosition(0);
//        robot.stage2Swing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //endregion

        //endregion

        // The slide initialization must be done at the end to avoid conflicts with the arms.
        //region Slide
//        robot.motorSlide.setPower(0);
//        robot.motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.motorSlide.setTargetPosition(0);
//        robot.motorSlide.setPower(0);
//        robot.motorSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // The following is for our slide. Currently our limit switch detection code is
        // commented out due to us having to remove it for height restrictions. When
        // re-enabling, uncomment this block of code, and the hardware that can be found
        // in the DeepHorOpMode class.

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
//        robot.motorSlide.setPower(0);
//        robot.motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //endregion

    }

}

