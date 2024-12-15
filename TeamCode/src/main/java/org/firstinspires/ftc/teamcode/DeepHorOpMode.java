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
    //region Drive Motors
    public DcMotor motorFL = null; // Front Left
    public DcMotor motorFR = null; // Front Right
    public DcMotor motorBL = null; // Back Left
    public DcMotor motorBR = null; // Back Right
    //endregion

    //region First Stage
    /**
     * Motor attached to the bot to move the arm containing the main arm mechanism.
     */
    public DcMotor stage1Arm = null;
    /**
     * The servo that scoops up game pieces from the floor into the first stage basket
     */
    public CRServo stage1Scoop = null;
    // end of first stage hw
    //endregion

    //region Second Stage
    /**
     * Motor attached to the bot to move the slide containing the second stage mechanism.
     */
    public DcMotor motorSlide = null;

    //region Limit Switches
    /**
     * Slide limit switches to prevent the slide from going too far.
     * These are used at the beginning of the match to calibrate the slide before the match starts.
     * TODO: Figure out which limit switch is which
     */
    public TouchSensor slideLimit1 = null;

    /**
     * Slide limit switches to prevent the slide from going too far.
     * These are used at the beginning of the match to calibrate the slide before the match starts.
     * TODO: Figure out which limit switch is which
     */
    public TouchSensor slideLimit2 = null;
    //endregion

    /**
     * The arm that brings the second stage bucket up to the baskets for scoring.
     */
    public DcMotor stage2Swing = null;

    /**
     * The servo that holds the second stage bucket.
     * This serves multiple purposes
     * 1. Preventing the piece from falling out as we bring it up to the basket
     * 2. Drops the piece into the basket
     */
    public Servo stage2Bucket = null;
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

    // Current Drive setting
    private double drivePwr = 0;
    private double strafePwr = 0;
    private double yawPwr = 0;

    //endregion

    private Thread bucketsync;
    //endregion

    public void ConfigureHardware(boolean initDriveMotors) {
        //region Drive Motors

        if (initDriveMotors) {
            this.motorFL = this.hardwareMap.get(DcMotor.class, "leftFront");
            this.motorFR = this.hardwareMap.get(DcMotor.class, "rightFront");
            this.motorBL = this.hardwareMap.get(DcMotor.class, "leftBack");
            this.motorBR = this.hardwareMap.get(DcMotor.class, "rightBack");

            this.motorFL.setDirection(DcMotor.Direction.REVERSE);
            this.motorBL.setDirection(DcMotor.Direction.REVERSE);
            this.motorFR.setDirection(DcMotor.Direction.FORWARD);
            this.motorBR.setDirection(DcMotor.Direction.FORWARD);
        }
        //endregion

        //region First Stage
        // Arm
        this.stage1Arm = this.hardwareMap.get(DcMotor.class, "ArmVertical");
        this.stage1Arm.setTargetPosition(255);
        this.stage1Arm.setPower(0);
        this.stage1Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Scoop
        this.stage1Scoop = this.hardwareMap.get(CRServo.class, "FrontCombine");
        //endregion

        //region Slide / Second Stage
        // Slide
        this.motorSlide = this.hardwareMap.get(DcMotor.class, "slideMotor");
        this.motorSlide.setTargetPosition(0);
        this.motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // The limit switches
        // TODO: Make limit switches respect FIRST high restrictions.
//        this.slideLimit1 = this.hardwareMap.get(TouchSensor.class, "slideLimit1");
//        this.slideLimit2 = this.hardwareMap.get(TouchSensor.class, "slideLimit2");
        // Swing
        this.stage2Swing = this.hardwareMap.get(DcMotor.class, "swing");
        this.stage2Swing.setTargetPosition(0);
        this.stage2Swing.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //bucket
        this.stage2Bucket = this.hardwareMap.get(Servo.class, "SlideServo");
        //endregion
    }


    public void MoveArmScoop(double power) {
        stage1Scoop.setPower(power);
    }

    /**
     * DO NOT USE
     * This method has not been updated to work properly, and using is at your own risk!
     * @param position Position to move the arm
     */
    public void MoveVerticalArm(double position) {
        stage1Arm.setTargetPosition((int) position);
        stage1Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stage1Arm.setPower(0.5);
    }

    //region Slide
    /**
     * Moves the slide from the second stage to a specific position.
     * <p>
     * WARNING: There is no limits on where the slide can go. Please be careful.
     * @param position The position to move the slide to. Should be from 0 to -10800.
     */
    public void MoveSlidePos(double position) {
        motorSlide.setTargetPosition((int) position);
        motorSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlide.setPower(1);
    }

    /**
     * Moves the slide by providing a power to run the motor.
     * This does have limits. You can travel from -10800 (top) to 0 (bottom).
     * @param power
     */
    public void MoveSlidePwr(double power) {
        // Adjust slidePower based on position limits
        if (motorSlide.getCurrentPosition() <= -10800 && power < 0) {
            // Prevent the motor from moving further negative
            power = 0;
        } else
        if (motorSlide.getCurrentPosition() >= 0 && power > 0) {
            // Prevent the motor from moving further positive
            power = 0;
        }
        motorSlide.setPower(power);
    }
    //endregion

    //region Bucket Sync
    /**
     * Starts the thread to move the bucket to be aligned with the bucketTargetPosition.
     *  0 is level`
     * -1 is up
     *  1 is down
     *  Going above 0.5 will most likely not go fully to the desired position.
     */
    public void StartBucketSync() {
        if (stage2BucketSync){
            return;
        }
        stage2BucketSync = true;
        bucketsync = new Thread(new Runnable() {
            @Override
            public void run() {
                while (stage2BucketSync && opModeIsActive()) {
                    UpdateBucketPosition();
                    try {
                        Thread.sleep(20);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
            }
        });
        bucketsync.start();
    }

    /**
     * Internal function for bucket sync.
     * This function should not be called directly.
     * To run a bucket sync, use StartBucketSync.
     */
    private void UpdateBucketPosition() {
        // When the swing is at -11, the bucket should be at 0.237
        // When the swing is at -130, the bucket should be at 0.811
        double swingPosition = stage2Swing.getCurrentPosition();
        double bucketPosition;

        bucketPosition = 0.237 + ((swingPosition + 11) * (0.811 - 0.237) / (-130 + 11));

        stage2Bucket.setPosition(bucketPosition + bucketTargetPosition);
    }
    //endregion

    //region Drive
    /**
     * Tells the robot to move thru power values
     * This should not be used when roadrunner is active
     * This function does not move the bot itself. You must call UpdateMoveRobot to actually preform the actions specified.
     * @param drivePwr The power value to move the robot forward/backward. Positive is forward, negative is backward.
     * @param strafePwr The power value to move the robot left/right. Positive is right, negative is left.
     * @param turnPwr The power value to turn the robot. Positive is right, negative is left.
     */
    public void MoveRobot(double drivePwr, double strafePwr, double turnPwr) {
        this.drivePwr = drivePwr;
        this.strafePwr = strafePwr;
        yawPwr = turnPwr;
    }

    /**
     * This executes the movement of the robot previously set.
     * This should not be used when roadrunner is active
     * @param speedLimit How fast the robot should run. 1 is full speed, 0.5 is half speed, etc.
     */
    public void UpdateMoveRobot(double speedLimit) {
        // Calculate wheel powers.
        double leftFrontPower    =   drivePwr -strafePwr -yawPwr;
        double rightFrontPower   =   drivePwr +strafePwr +yawPwr;
        double leftBackPower     =  -drivePwr -strafePwr +yawPwr;
        double rightBackPower    =  -drivePwr +strafePwr -yawPwr;

        // Normalize wheel powers to be less than 1.0
        double max = (Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        motorFL.setPower(leftFrontPower*speedLimit);
        motorFR.setPower(rightFrontPower*speedLimit);
        motorBL.setPower(leftBackPower*speedLimit);
        motorBR.setPower(rightBackPower*speedLimit);
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
            bucketTargetPosition = -0.1;
            Thread.sleep(500);

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
            return;
        }
    }
    //endregion
}
