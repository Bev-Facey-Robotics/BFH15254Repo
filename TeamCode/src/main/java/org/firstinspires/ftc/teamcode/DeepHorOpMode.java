package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


public abstract class DeepHorOpMode extends LinearOpMode {
    //region Variables
    //region Hardware


    //region First Stage

    // end of first stage hw
    //endregion

    //region Second Stage




    // end of second stage hw
    //endregion
    //endregion

    //region Bucket Sync
    /**
     * The relative location of the bucket.
     * 0 is level (straight facing the wall
     * -0.5 is full up
     * 0.5 is full down
     * This goes from -0.5 to +0.5.
     */
    public double bucketTargetPosition = 0;
    /**
     * Whether bucket sync is on
     */
    public boolean stage2BucketSync = false;
    //endregion

    //region Piece Assist
    /**
     * The current state of piece assist. If true, the robot is currently running piece assist.
     */
    public volatile boolean AssistRunning = false;
    /**
     * The thread that Piece Assist is running in.
     */
    private Thread pieceAssistThread;
    //endregion

    //region Drive



    //endregion
    //endregion

    public void ConfigureHardware(boolean initDriveMotors) {
        //region Drive Motors

        if (initDriveMotors) {

        }
        //endregion

        //region First Stage
        // Arm

        // Scoop

        //endregion

        //region Slide / Second Stage
        // Slide

        // Swing
        //endregion
    }





    //region Bucket Sync
    /**
     * Starts the thread to move the bucket to be aligned with the bucketTargetPosition.




    //endregion

    //region Drive

    /**
     * This executes the movement of the robot previously set.
     * This should not be used when roadrunner is active
     * @param speedLimit How fast the robot should run. 1 is full speed, 0.5 is half speed, etc.
     */
    public void UpdateMoveRobot(double speedLimit) {

    }
    //endregion

    //region Piece Assist
    public void StopPieceAssist() {
        if (!AssistRunning) {
            return;
        }
        AssistRunning = false;
        if (pieceAssistThread != null) {
            pieceAssistThread.interrupt();
        }
        motorSlide.setPower(0);
        motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void StartPieceAssist() {
        if (AssistRunning) {
            return;
        }
        AssistRunning = true;
        pieceAssistThread = new Thread(new Runnable() {
            @Override
            public void run() {
                PieceAssist();
            }
        });
        pieceAssistThread.start();
    }

    public void PieceAssist() {
        try {
            // This automatically moves the slide & arm to the correct position to transfer, then raises the slide & piece bucket to be delivered into the bucket
            bucketTargetPosition = 0.05;
            MoveSlidePos(-1821);
            Thread.sleep(2000);
            // wait for slide to move down
            while (motorSlide.isBusy() && AssistRunning) {
                telemetry.addData("Slide Position", motorSlide.getCurrentPosition());
                telemetry.update();
                Thread.sleep(20);
            }
            if (!AssistRunning) return;

            stage1Arm.setTargetPosition(255);
            stage1Arm.setPower(0.3);
            while (stage1Arm.isBusy() && AssistRunning) {
                telemetry.addData("Arm Position", stage1Arm.getCurrentPosition());
                telemetry.update();
                Thread.sleep(20);
            }
            if (!AssistRunning) return;

            MoveArmScoop(1);
            Thread.sleep(3000);
            if (!AssistRunning) return;

            MoveArmScoop(0);
            stage1Arm.setPower(0.1);
            stage1Arm.setTargetPosition(5);
            Thread.sleep(500);
            bucketTargetPosition = -0.1;

            // Move slide to the top
            MoveSlidePos(-10800);
            while (motorSlide.isBusy() && AssistRunning) {
                telemetry.addData("Slide Position", motorSlide.getCurrentPosition());
                telemetry.update();
                Thread.sleep(20);
            }
            if (!AssistRunning) return;

            // Move the swing to correct
            stage2Swing.setTargetPosition(-130);
            stage2Swing.setPower(0.5);
            Thread.sleep(7000);

            if (!AssistRunning) return;

            // dump the piece in the basket
            bucketTargetPosition = 0.2;
            Thread.sleep(3000);

            stage2Swing.setTargetPosition(-130);
            stage2Swing.setPower(0.3);
            while (stage2Swing.isBusy() && AssistRunning) {
                telemetry.addData("Arm Position", stage2Swing.getCurrentPosition());
                telemetry.update();
                Thread.sleep(20);
            }

            AssistRunning = false;

        } catch (InterruptedException e) {
            // Handle the interruption
            AssistRunning = false;
        }
    }
    //endregion
}
