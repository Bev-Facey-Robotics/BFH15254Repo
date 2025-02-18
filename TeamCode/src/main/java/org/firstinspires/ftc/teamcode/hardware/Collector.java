package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.internal.HardwareElement;

/**
 * This hardware will not be present in newer versions of the robot, and is thus deprecated.
 * Usage is not recommended!!!
 */
@Deprecated
public class Collector extends HardwareElement {
    public DcMotor arm = null;
    public CRServo scoop = null;

    public void init(HardwareMap hardwareMap) {
        this.scoop = hardwareMap.get(CRServo.class, "FrontCombine");
        this.arm = hardwareMap.get(DcMotor.class, "ArmVertical");
        this.arm.setTargetPosition(255);
        this.arm.setPower(0);
        this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void calibrate() {
        this.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.arm.setPower(-0.3);
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

    /**
     * Moves the first stage arm.
     * The bottom is 0, and the top is 255.
     * @param position The position to move the arm, generally from 0 to 255
     * @param power How fast the arm should go
     */
    public void MoveArmToPosition(double position, double power) {
        this.arm.setTargetPosition((int) position);
        this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.arm.setPower(power);
    }

    /**
     * Rotates the scoop attached to the arm.
     * @param power Power it will run at. -1 to 1.
     */
    public void RotateScoop(double power) {
        scoop.setPower(power);
    }

    public void self_test() {

    }

    public void stop() {
        this.arm.setPower(0);
        this.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.arm.setTargetPosition(0);
        this.arm = null;
        this.scoop = null;
    }
}
