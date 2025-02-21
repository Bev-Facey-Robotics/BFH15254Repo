package org.firstinspires.ftc.teamcode.actions.manual;

import org.firstinspires.ftc.teamcode.hardware.FrontCombine;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.internal.ActionElement;
import org.firstinspires.ftc.teamcode.internal.HardwareManager;

public class ManualFrontCombine extends ActionElement {

    final public double sensitivity = 0.1;

    @Override
    public void run() throws InterruptedException, NullPointerException {
        FrontCombine frontCombine = (FrontCombine) HardwareManager.ReserveHardware(this, "FrontCombine");
        while (!Thread.currentThread().isInterrupted()) {
            // the collecting of the piece
            if (HardwareManager.opMode.gamepad2.a) {
                frontCombine.TakePiece(1);
            } else if (HardwareManager.opMode.gamepad2.b) {
                frontCombine.TakePiece(-1);
            } else {
                frontCombine.TakePiece(0);
            }
            // rotating the assembly to align with the piece
            frontCombine.RotateAssembly(frontCombine.unitRotate.getPosition() + (HardwareManager.opMode.gamepad2.right_stick_y * sensitivity));
            // moving the intake assembly up and down
            if (HardwareManager.opMode.gamepad2.y) {
                frontCombine.SetIntakeActive(false);
            } else if (HardwareManager.opMode.gamepad2.x) {
                frontCombine.SetIntakeActive(true);
            }
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
