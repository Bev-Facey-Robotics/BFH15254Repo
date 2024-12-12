package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


public abstract class DeepHorOpMode extends LinearOpMode {
    //region Hardware
    //region Drive Motors
    public DcMotor motorFL = null; // Front Left
    public DcMotor motorFR = null; // Front Right
    public DcMotor motorBL = null; // Back Left
    public DcMotor motorBR = null; // Back Right
    //endregion

    //region Slide
    public DcMotor motorSlide = null;
    public DcMotor motorSwing = null;
//    public Servo clawServo = null;
//    //endregion
//    //region Arm
//    /**
//     * Motor attached to the bot to move the arm containing the main arm mechanism.
//     */
    public DcMotor arm_Vertical = null;
//    /**
//     * The first servo on the arm. Swivels the main scoop mechanism off the arm.
//     */
    public Servo arm_Pivot = null;
//    /**
//     * The second servo on the arm. Moves the rest of the scoop up and down.
//     */
//    public Servo arm_VerticalServo = null;
    /**
     * The third servo on the arm. Controls the wheel to scoop the game pieces.
     */
    public CRServo arm_Scoop = null;

    public Servo topBucketServo = null;

//    private
//
//    public double armSmallHorizontal = 0.52;
//    public double armVertical = 0.57;
//
//    public int armTarget = 0;
    double bucketTargetPosition = 0;

    //endregion

    public TouchSensor slideLimit1 = null;
    public TouchSensor slideLimit2 = null;

    public boolean isBucketBalencingEnabled = false;

    public IMU imu = null;

    //endregion

    //region Drive

    // Current Drive setting
    private double drivePwr = 0;
    private double strafePwr = 0;
    private double yawPwr = 0;

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

        //region Slide
        this.motorSlide = this.hardwareMap.get(DcMotor.class, "slideMotor");
        this.slideLimit1 = this.hardwareMap.get(TouchSensor.class, "slideLimit1");
        this.slideLimit2 = this.hardwareMap.get(TouchSensor.class, "slideLimit2");
        this.motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorSwing = this.hardwareMap.get(DcMotor.class, "swing");
        this.motorSwing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.topBucketServo = this.hardwareMap.get(Servo.class, "SlideServo");


        //endregion

        //region Arm Hardware
        this.arm_Vertical = this.hardwareMap.get(DcMotor.class, "ArmVertical");
//        this.arm_SmallHorizontal = this.hardwareMap.get(Servo.class, "RotHori");
//        this.arm_VerticalServo = this.hardwareMap.get(Servo.class, "RotVert");
        this.arm_Scoop = this.hardwareMap.get(CRServo.class, "FrontCombine");


        //endregion

    }

    public void MoveRobot(double x, double y, double yaw) {
        drivePwr = x;
        strafePwr = y;
        yawPwr = yaw;
    }

    public void MoveArmScoop(double power) {
        arm_Scoop.setPower(power);
    }

    public void MoveArmPivot(double position) {
        arm_Pivot.setPosition(position);
    }

    public void MoveVerticalArm(double position) {
        arm_Vertical.setTargetPosition((int) position);
        arm_Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_Vertical.setPower(0.5);
    }

    public void MoveSlidePos(double position) {
        motorSlide.setTargetPosition((int) position);
        motorSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlide.setPower(0.5);
    }

    public void MoveSlidePwr(double power) {
        // Adjust slidePower based on position limits
        if (motorSlide.getCurrentPosition() <= -10800 && power < 0) { // Limit is -8950
            // Prevent the motor from moving further negative
            power = 0;
        } else
        if (motorSlide.getCurrentPosition() >= 0 && power > 0) {
            // Prevent the motor from moving further positive
            power = 0;
        }
        motorSlide.setPower(power);

    }

    public void StartBucketSync() {
        isBucketBalencingEnabled = true;
        new Thread(new Runnable() {
            @Override
            public void run() {
                while (isBucketBalencingEnabled && opModeIsActive()) {
                    UpdateBucketPosition();
                    try {
                        Thread.sleep(20);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
            }
        }).start();
    }

    public void UpdateBucketPosition() {
        // When the swing is at -11, the bucket should be at 0.237
        // When the swing is at -130, the bucket should be at 0.811
        double swingPosition = motorSwing.getCurrentPosition();
        double bucketPosition;

        bucketPosition = 0.237 + ((swingPosition + 11) * (0.811 - 0.237) / (-130 + 11));

        topBucketServo.setPosition(bucketPosition + bucketTargetPosition);
    }

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
}
