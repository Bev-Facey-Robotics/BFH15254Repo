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


        } catch (InterruptedException e) {
            // Handle the interruption
            AssistRunning = false;
        }
    }
    //endregion
}
