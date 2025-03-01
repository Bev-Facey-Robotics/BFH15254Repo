package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.internal.HardwareElement;
import org.firstinspires.ftc.teamcode.internal.TelemetryManager;

public class Slide extends HardwareElement {
    final private int slideNegativeLimit = 0;
    final private int slidePositiveLimit = 3050;

    public boolean areLimitsEnabled = true;
    public DcMotor motorSlide = null;


    public void init(HardwareMap hardwareMap) {
        this.motorSlide = hardwareMap.get(DcMotor.class, "slideMotor");
        this.motorSlide.setPower(0);
        this.motorSlide.setTargetPosition(0);
        this.motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TelemetryManager.instance.AddFunctionToLogging("Slide Pos", () -> motorSlide.getCurrentPosition());
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

//    public void update() {
//        // Later, detection if the slide is about to kill itself should be added.
//    }

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

    public void MovePowerNoLimits (double power) {
        if (motorSlide.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    public void MovePositionWithPower(int position, double power) {
        if (motorSlide.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            motorSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        motorSlide.setTargetPosition(position);
        motorSlide.setPower(power);
    }


    public void stop() {
        this.motorSlide.setPower(0);
        this.motorSlide.setTargetPosition(0);
        this.motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorSlide = null;
    }

    public boolean change = false;

    public void update() {
        if (this.motorSlide.getTargetPosition() <= 5) { // This is here because when it was fully down, it was still using a lot of power, and turning that power into heat.
            if (this.motorSlide.getCurrentPosition() <= 10) {
                if (change)
                    return;
                change = true;
                this.motorSlide.setPower(0);
                return;
            }
        }
        if (!change)
            return;
        change = false;
        this.motorSlide.setPower(1);
    }
}
