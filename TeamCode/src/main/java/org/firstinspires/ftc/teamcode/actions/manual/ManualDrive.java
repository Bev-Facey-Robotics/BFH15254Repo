package org.firstinspires.ftc.teamcode.actions.manual;

import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.internal.ActionElement;
import org.firstinspires.ftc.teamcode.internal.HardwareManager;

public class ManualDrive extends ActionElement {
    private final double deadzone = 0.1;

    @Override
    public void run() throws InterruptedException, NullPointerException  {
        // Reserve the hardware
        Drive drive = (Drive) HardwareManager.ReserveHardware(this,"Drive");

        while (!Thread.currentThread().isInterrupted()) {

            double speed = 1.0;
            if (HardwareManager.opMode.gamepad1.left_trigger > 0.15) {
                speed = 0.5;
            }
            if (HardwareManager.opMode.gamepad1.right_trigger > 0.15) {
                speed = 0.25;
            }


            // Apply deadzone to joystick inputs
            double driveSpeed = Math.abs(HardwareManager.opMode.gamepad1.left_stick_y) > deadzone ? -HardwareManager.opMode.gamepad1.left_stick_y : 0;
            double strafeSpeed = Math.abs(HardwareManager.opMode.gamepad1.left_stick_x) > deadzone ? -HardwareManager.opMode.gamepad1.left_stick_x : 0;
            double turnSpeed = Math.abs(HardwareManager.opMode.gamepad1.right_stick_x) > deadzone ? -HardwareManager.opMode.gamepad1.right_stick_x : 0;

            // Invert drive & strafe directions
            if (!HardwareManager.opMode.gamepad1.right_bumper) {
                driveSpeed *= -1;
                strafeSpeed *= -1;
            }

            drive.speedLimit = speed;
            drive.move(driveSpeed, strafeSpeed, turnSpeed);

            Thread.sleep(20);
        }

    }
    @Override
    public boolean isAutoRestart() {
        return true;
    }
    @Override
    public int getPriority() {
        return 1;
    }
}
