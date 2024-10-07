package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Vector;

@TeleOp(name = "Logan's Terrible Bot", group = "testing")

public class LoganTestBot extends OpMode {

    private DcMotor motorFL = null; // Front Left
    private DcMotor motorFR = null; // Front Right
    private DcMotor motorBL = null; // Back Left
    private DcMotor motorBR = null; // Back Right


    @Override
    public void init() {
        configureMotors();
        telemetry.addLine("hello world!");
        telemetry.update();

    }

    private void configureMotors() {
        motorFL = hardwareMap.get(DcMotor.class, "leftFront");
        motorFR = hardwareMap.get(DcMotor.class, "rightFront");
        motorBL = hardwareMap.get(DcMotor.class, "leftBack");
        motorBR = hardwareMap.get(DcMotor.class, "rightBack");
    }

    private void driveBot(Vector<Double> driveParams) {
        motorFL.setPower(driveParams.get(0));
        motorFR.setPower(driveParams.get(1));
        motorBL.setPower(driveParams.get(2));
        motorBR.setPower(driveParams.get(3));
    }

    private Vector<Double> addDriveVectors(Vector<Double> a, Vector<Double> b) {
        Vector<Double> c = new Vector<Double>();
        c.add(a.get(0) + b.get(0));
        c.add(a.get(1) + b.get(1));
        c.add(a.get(2) + b.get(2));
        c.add(a.get(3) + b.get(3));
        return c;
    }

    private Vector<Double> subtractDriveVectors(Vector<Double> a, Vector<Double> b) {
        Vector<Double> c = new Vector<Double>();
        c.add(a.get(0) - b.get(0));
        c.add(a.get(1) - b.get(1));
        c.add(a.get(2) - b.get(2));
        c.add(a.get(3) - b.get(3));
        return c;
    }

    private Vector<Double> multiplyDriveVectors(Vector<Double> a, Vector<Double> b) {
        Vector<Double> c = new Vector<Double>();
        c.add(a.get(0) * b.get(0));
        c.add(a.get(1) * b.get(1));
        c.add(a.get(2) * b.get(2));
        c.add(a.get(3) * b.get(3));
        return c;
    }

    // Drive Functions
    private Vector<Double> calculateDriveGamepadMotorSpeed() {
        Vector<Double> speeds = new Vector<Double>();
        speeds.add((double) gamepad1.left_stick_y);
        speeds.add((double) gamepad1.left_stick_y * -1);
        speeds.add((double) gamepad1.left_stick_y * -1);
        speeds.add((double) gamepad1.left_stick_y);
        return speeds;
    }

    private Vector<Double> calculateTurnGamepadMotorSpeed() {
        Vector<Double> speeds = new Vector<Double>();
        speeds.add((double) gamepad1.right_stick_x * -1);
        speeds.add((double) gamepad1.right_stick_x * -1);
        speeds.add((double) gamepad1.right_stick_x);
        speeds.add((double) gamepad1.right_stick_x);
        return speeds;
    }

    private Vector<Double> calculateStrafeGamepadMotorSpeed() {
        Vector<Double> speeds = new Vector<Double>();
        speeds.add((double) gamepad1.left_stick_x * -1);
        speeds.add((double) gamepad1.left_stick_x * -1);
        speeds.add((double) gamepad1.left_stick_x * -1);
        speeds.add((double) gamepad1.left_stick_x * -1);
        return speeds;
    }

    private void DriveRobotWithController() {
        Vector<Double> drive = new Vector<Double>();
        drive = calculateDriveGamepadMotorSpeed();
        drive = addDriveVectors(drive, calculateTurnGamepadMotorSpeed());
        drive = addDriveVectors(drive, calculateStrafeGamepadMotorSpeed());

        driveBot(drive);
    }

    @Override
    public void loop() {
        DriveRobotWithController();
    }
}
