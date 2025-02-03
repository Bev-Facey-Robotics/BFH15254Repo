package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.internal.BaseOpMode;

@TeleOp(name = "Robot Manual Control", group = "Competition Ready")
public class RobotManualControl extends BaseOpMode {
    //region Hardware

    public double armSmallHorizontal = 0.52;
    public double armVertical = 0.57;
    //endregion

    // Targets


    // Held down checks
    private boolean isBackheld = false;

    // Bot Config


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
    }
    //endregion

    //region Primary Loop
    public void loopOpMode() {
        //region Controller 2
        boolean isSlowModeActive = !gamepad2.left_bumper;


        //region Slide / Second Stage
        // Slide

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
        telemetry.addData("Assist", AssistRunning);
//        telemetry.addLine("Top Bucket Position (servo) " + stage2Bucket.getPosition());
//        telemetry.addLine("Bucket Target Position (servo)" + bucketTargetPosition);

        telemetry.addLine("Arm Vertical: " + stage1Arm.getCurrentPosition());

        telemetry.update();
        //endregion
    }
    //endregion

}

