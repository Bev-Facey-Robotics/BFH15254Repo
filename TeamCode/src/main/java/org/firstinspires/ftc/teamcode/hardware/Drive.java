package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.internal.HardwareElement;

public class Drive extends HardwareElement {
    public DcMotor motorFL = null; // Front Left
    public DcMotor motorFR = null; // Front Right
    public DcMotor motorBL = null; // Back Left
    public DcMotor motorBR = null; // Back Right

    // Current Drive setting
    private double drivePwr = 0;
    private double strafePwr = 0;
    private double yawPwr = 0;

    // Speed Limit
    public double speedLimit = 1.0;

    public void init(HardwareMap hardwareMap) {
        this.motorFL = hardwareMap.get(DcMotor.class, "leftFront");
        this.motorFR = hardwareMap.get(DcMotor.class, "rightFront");
        this.motorBL = hardwareMap.get(DcMotor.class, "leftBack");
        this.motorBR = hardwareMap.get(DcMotor.class, "rightBack");

        this.motorFL.setDirection(DcMotor.Direction.REVERSE);
        this.motorBL.setDirection(DcMotor.Direction.REVERSE);
        this.motorFR.setDirection(DcMotor.Direction.FORWARD);
        this.motorBR.setDirection(DcMotor.Direction.FORWARD);
    }

    public void update() {
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

    // stop our motors
    public void stop() {
        this.motorFL.setPower(0);
        this.motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorFL = null;

        this.motorFR.setPower(0);
        this.motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorFR = null;

        this.motorBL.setPower(0);
        this.motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorBL = null;

        this.motorBR.setPower(0);
        this.motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorBR = null;
    }

    public void move(double drivePwr, double strafePwr, double yawPwr) {
        this.drivePwr = drivePwr;
        this.strafePwr = strafePwr;
        this.yawPwr = yawPwr;
    }

    public void self_test() {
    }

    public void calibrate() {
    }
}
