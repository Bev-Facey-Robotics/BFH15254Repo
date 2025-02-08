package org.firstinspires.ftc.teamcode.actions.manual;

import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.internal.ActionElement;
import org.firstinspires.ftc.teamcode.internal.HardwareManager;

public class ManualSlide extends ActionElement {
    @Override
    public void run() throws InterruptedException {
        // Reserve the hardware
        Slide slide = (Slide) HardwareManager.ReserveHardware(this, "Slide");

        while (!Thread.currentThread().isInterrupted()) {
            double slidePower = HardwareManager.opMode.gamepad2.right_trigger - HardwareManager.opMode.gamepad2.left_trigger;
            slide.MovePower(slidePower);
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
