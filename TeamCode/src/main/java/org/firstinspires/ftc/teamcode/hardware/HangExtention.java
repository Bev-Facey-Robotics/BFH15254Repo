package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.internal.HardwareElement;

public class HangExtention extends HardwareElement {
    public DcMotor motorHangExtension1 = null;
    public DcMotor motorHangExtension2 = null;

    public void init(HardwareMap hardwareMap) {
        // motor 1
        this.motorHangExtension1 = hardwareMap.get(DcMotor.class, "motorHangExtension1");
        this.motorHangExtension1.setDirection(DcMotor.Direction.FORWARD);
        this.motorHangExtension1.setPower(0);
        this.motorHangExtension1.setTargetPosition(0);
        this.motorHangExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // motor 2
        this.motorHangExtension2 = hardwareMap.get(DcMotor.class, "motorHangExtension2");
        this.motorHangExtension2.setDirection(DcMotor.Direction.REVERSE);
        this.motorHangExtension2.setPower(0);
        this.motorHangExtension2.setTargetPosition(0);
        this.motorHangExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void calibrate() {
        this.motorHangExtension1.setPower(0);
        this.motorHangExtension1.setTargetPosition(0);
        this.motorHangExtension1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorHangExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorHangExtension1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorHangExtension1.setPower(0);
        this.motorHangExtension1.setTargetPosition(0);

        this.motorHangExtension2.setPower(0);
        this.motorHangExtension2.setTargetPosition(0);
        this.motorHangExtension2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorHangExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorHangExtension2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorHangExtension2.setPower(0);
        this.motorHangExtension2.setTargetPosition(0);
    }

    public void update() {
        // Nothing to do here
    }

    public void self_test() {

    }

    public void MoveToPosition(int position) {
        motorHangExtension1.setTargetPosition(position);
        motorHangExtension2.setTargetPosition(position);
        motorHangExtension1.setPower(1);
        motorHangExtension2.setPower(1);

    }

    public void stop() {
        motorHangExtension1.setPower(0);
        motorHangExtension2.setPower(0);
    }

    public boolean isBusy() {
        return motorHangExtension1.isBusy() || motorHangExtension2.isBusy();
    }
}
