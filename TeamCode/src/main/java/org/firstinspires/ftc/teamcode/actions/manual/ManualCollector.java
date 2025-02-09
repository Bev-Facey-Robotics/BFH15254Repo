package org.firstinspires.ftc.teamcode.actions.manual;

import org.firstinspires.ftc.teamcode.hardware.Collector;
import org.firstinspires.ftc.teamcode.internal.ActionElement;
import org.firstinspires.ftc.teamcode.internal.HardwareManager;

@Deprecated
public class ManualCollector extends ActionElement {
    @Override
    public void run() throws InterruptedException, NullPointerException  {
        // Reserve the hardware
        Collector collector = (Collector) HardwareManager.ReserveHardware(this,"Collector");

        while (!Thread.currentThread().isInterrupted()) {
            // Scoop
            double armScoop = (HardwareManager.opMode.gamepad2.b ? 1 : 0) + (HardwareManager.opMode.gamepad2.a ? -1 : 0);
            collector.RotateScoop(armScoop);

            // First stage bucket
            if (HardwareManager.opMode.gamepad2.x) {
                collector.MoveArmToPosition(5, 0.1);
            }
            if (HardwareManager.opMode.gamepad2.y) {
                collector.MoveArmToPosition(255, 0.3);
            }

            // First stage bucket power control
            if (collector.arm.getTargetPosition() == 5) {
                if (collector.arm.getCurrentPosition() < 30) {
                    collector.arm.setPower(0);
                } else {
                    collector.arm.setPower(0.1);
                }
            }
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
