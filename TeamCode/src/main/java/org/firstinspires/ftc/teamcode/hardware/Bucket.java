package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.internal.HardwareElement;

/**
 * This hardware will not be present in newer versions of the robot, and is thus deprecated.
 * Usage is not recommended!!!
 */
@Deprecated
public class Bucket extends HardwareElement {
    public DcMotor arm = null;
    public Servo bucket = null;

    public double bucketTargetPosition = 0;

    public boolean isBucketSyncRunning = true;

    public void init(HardwareMap hardwareMap) {
        this.arm = hardwareMap.get(DcMotor.class, "swing");
        this.arm.setPower(0);
        this.arm.setTargetPosition(0);
        this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.bucket = hardwareMap.get(Servo.class, "SlideServo");
    }

    public void calibrate() {
        this.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.arm.setPower(0.2);
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        this.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.arm.setPower(0);
        this.arm.setTargetPosition(0);
        this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void MoveArmToPosition(double position, double power) {
        this.arm.setTargetPosition((int) position);
        this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.arm.setPower(power);
    }

    /**
     * Internal function for bucket sync.
     * This function should not be called directly.
     * To run a bucket sync, use StartBucketSync.
     */
    private void UpdateBucketPosition() {
        // When the swing is at -11, the bucket should be at 0.237
        // When the swing is at -130, the bucket should be at 0.811
        double swingPosition = this.arm.getCurrentPosition();
        double bucketPosition;

        bucketPosition = 0.237 + ((swingPosition + 11) * (0.811 - 0.237) / (-130 + 11));

        this.bucket.setPosition(bucketPosition + bucketTargetPosition);
    }


    public void self_test() {

    }

    /**
     *  0 is level`
     * -1 is up
     *  1 is down
     *  Going above 0.5 will most likely not go fully to the desired position.
     */
    public void update() {
        if (isBucketSyncRunning) {
            UpdateBucketPosition();
        }
    }

    public void stop() {
        // Stop the arm
        this.arm.setPower(0);
        this.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.arm.setTargetPosition(0);
        this.arm = null;
        // Stop the bucket
        this.bucket.setPosition(0.5);
        this.bucket = null;
    }
}
