package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.internal.HardwareElement;

import java.util.concurrent.TimeUnit;

public class FrontCombine extends HardwareElement {
    // Hardware
    public Servo unitRotate = null;
    private CRServo leftIntake = null;
    private CRServo rightIntake = null;

    public DcMotor intakeActuator = null;

    public boolean isIntakeActive = false;

    public void init(HardwareMap hardwareMap) {
        this.unitRotate = hardwareMap.get(Servo.class, "UnitRotate");
        this.leftIntake = hardwareMap.get(CRServo.class, "LeftIntake");
        this.rightIntake = hardwareMap.get(CRServo.class, "RightIntake");

        this.intakeActuator = hardwareMap.get(DcMotor.class, "IntakeActuator");
        //this.intakeActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // We stop giving the arm power at a certain point when the arm is on the floor.
        this.intakeActuator.setTargetPosition(0);
        this.intakeActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.intakeActuator.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Rotates the servos at the front to take in a piece.
     * @param power The power to rotate the servos at. 1 is take in, -1 is push out.
     */
    public void TakePiece(double power) {
        this.leftIntake.setPower(power);
        this.rightIntake.setPower(-power);
    }

    /**
     * Rotates the main intake assembly to a certain position.
     * Used for aligning with the samples.
     */
    public void RotateAssembly(double position) {
        this.unitRotate.setPosition(position);
    }

    /**
     * This will move the whole intake assembly down to the ground if true, and up if false.
     * P.S. If you have a better name, make a github issue!
     */
    public void SetIntakeActive(boolean active) {
        isIntakeActive = active;
        if (active) {
            this.intakeActuator.setTargetPosition(107);
        } else {
            this.intakeActuator.setTargetPosition(0); // TODO: Make it not complete BS numbers.
        }

    }

    public void calibrate() {
        // Calibration will be figured out once the hardware is actually on the robot
        this.intakeActuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.intakeActuator.setPower(-0.5);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            return;
        }
        this.intakeActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.intakeActuator.setTargetPosition(0);
        this.intakeActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void update() {
        // TODO: Revamp this to reset calibration if we are confirmed to be at the lowest position
        if (!isIntakeActive) { //intake is down
            if (this.intakeActuator.getCurrentPosition() < 30) {
                this.intakeActuator.setPower(0.01); // this makes sure that if we encounter a bump we will at least stay somewhat down.
            } else {
                this.intakeActuator.setPower(1);
            }
        } else {
            if (this.intakeActuator.getCurrentPosition() > 80) {
                this.intakeActuator.setPower(0.01);
            } else {
                this.intakeActuator.setPower(0.25);
            }
        }
    }

    public void self_test() {

    }

    public void stop() {
        this.intakeActuator.setPower(0);

        this.leftIntake = null;
        this.rightIntake = null;
        this.unitRotate = null;
        this.intakeActuator = null;
    }

}
