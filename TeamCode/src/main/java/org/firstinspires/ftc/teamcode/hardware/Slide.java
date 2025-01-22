package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * The mechanism that brings game pieces from the first stage bucket, up to the top basket
 */
public class Slide extends HardwareElement {
    final private int slideNegativeLimit = -10800;
    final private int slidePositiveLimit = 0;

    public boolean areLimitsEnabled = true;

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

    public void init(HardwareMap hardwareMap) {
        this.motorSlide = hardwareMap.get(DcMotor.class, "slideMotor");
        this.motorSlide.setPower(0);
        this.motorSlide.setTargetPosition(0);
        this.motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Limit Switches
        // TODO: Make limit switches respect FIRST high restrictions.
        // this.slideLimit1 = this.hardwareMap.get(TouchSensor.class, "slideLimit1");
        // this.slideLimit2 = this.hardwareMap.get(TouchSensor.class, "slideLimit2");
    }

    public void calibrate() {
        this.motorSlide.setPower(0);
        this.motorSlide.setTargetPosition(0);
        this.motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorSlide.setPower(0);
        this.motorSlide.setTargetPosition(0);
        this.motorSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void self_test() {
    }

    public void update() {
        // Later, detection if the slide is about to kill itself should be added.
    }

    /**
     * Move the slide mechanism with a given power. This does have limits enabled by default.
     * @param power The power to move the slide with. (-1 to 1)
     */
    public void MovePower (double power) {
        if (motorSlide.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (areLimitsEnabled) {
            // Adjust slidePower based on position limits
            if (motorSlide.getCurrentPosition() <= slideNegativeLimit && power < 0) {
                // Prevent the motor from moving further negative
                power = 0;
            } else
            if (motorSlide.getCurrentPosition() >= slidePositiveLimit && power > 0) {
                // Prevent the motor from moving further positive
                power = 0;
            }
        }
        motorSlide.setPower(power);
    }

    /**
     * Move the slide mechanism to a given position. This does not have limits, use caution.
     * @param position The position to move the slide to.
     */
    public void MovePosition(int position) {
        if (motorSlide.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            motorSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        motorSlide.setTargetPosition(position);
        motorSlide.setPower(1);
    }


    public void stop() {
        this.motorSlide.setPower(0);
        this.motorSlide.setTargetPosition(0);
        this.motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorSlide = null;
    }
}
