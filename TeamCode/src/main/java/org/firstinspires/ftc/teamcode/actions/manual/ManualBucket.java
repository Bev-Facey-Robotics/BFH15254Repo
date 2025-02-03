package org.firstinspires.ftc.teamcode.actions.manual;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Bucket;
import org.firstinspires.ftc.teamcode.internal.ActionElement;
import org.firstinspires.ftc.teamcode.internal.HardwareManager;

public class ManualBucket extends ActionElement {
    public double verticalTarget = -4;

    @Override
    public void run() throws InterruptedException {
        // Reserve the hardware
        Bucket bucket = (Bucket) HardwareManager.ReserveHardware(this, "Bucket");

        while (true) {
            if (HardwareManager.opMode.gamepad2.dpad_down) {
                bucket.bucketTargetPosition = -11;
            } else if (HardwareManager.opMode.gamepad2.dpad_up) {
                bucket.bucketTargetPosition = -130;
            }

            bucket.MoveArmToPosition(verticalTarget, 0.5);

            // Second stage bucket
            bucket.bucketTargetPosition = -0.1;
            if (HardwareManager.opMode.gamepad2.right_bumper) {
                bucket.bucketTargetPosition = 0.25;
            }
            if (HardwareManager.opMode.gamepad2.left_bumper) {
                bucket.bucketTargetPosition = 0.05;
            }
        }
    }
}
