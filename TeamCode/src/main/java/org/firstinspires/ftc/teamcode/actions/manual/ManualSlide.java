package org.firstinspires.ftc.teamcode.actions.manual;

import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.internal.ActionElement;
import org.firstinspires.ftc.teamcode.internal.HardwareManager;

/**
 * Usage of this is highly unrecommended, due to rebuild.
 */
public class ManualSlide extends ActionElement {
    private boolean isPositionBased = false;
    private boolean lastRightShoulderButton = false;

    @Override
    public void run() throws InterruptedException, NullPointerException  {
        // Reserve the hardware
        Slide slide = (Slide) HardwareManager.ReserveHardware(this, "Slide");

        while (!Thread.currentThread().isInterrupted()) {
            if (HardwareManager.opMode.gamepad2.options) {
                slide.calibrate();
                continue;
            }
            if (HardwareManager.opMode.gamepad2.dpad_up) {
                slide.MovePosition(2929);
                isPositionBased = true;
            }
            if (HardwareManager.opMode.gamepad2.dpad_down) {
                slide.MovePosition(0);
                isPositionBased = true;
            }
            if (HardwareManager.opMode.gamepad2.dpad_left) {
                slide.MovePosition(1464);
                isPositionBased = true;
            }
            if (HardwareManager.opMode.gamepad2.dpad_right) {
                slide.MovePosition(645);
                isPositionBased = true;
            }
            if (HardwareManager.opMode.gamepad2.right_trigger - HardwareManager.opMode.gamepad2.left_trigger !=0) {
                isPositionBased = false;
            }
            if (!isPositionBased){
                double slidePower = HardwareManager.opMode.gamepad2.right_trigger - HardwareManager.opMode.gamepad2.left_trigger;
                if (HardwareManager.opMode.gamepad2.left_bumper) {
                    slide.MovePowerNoLimits(slidePower);
                    lastRightShoulderButton = true;
                } else {
                    if (lastRightShoulderButton) {
                        slide.calibrate();
                        lastRightShoulderButton = false;
                    }
                    slide.MovePower(slidePower);
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
