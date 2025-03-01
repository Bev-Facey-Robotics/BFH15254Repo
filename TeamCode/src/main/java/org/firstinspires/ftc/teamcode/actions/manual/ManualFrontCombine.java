package org.firstinspires.ftc.teamcode.actions.manual;

import org.firstinspires.ftc.teamcode.hardware.FrontCombine;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.internal.ActionElement;
import org.firstinspires.ftc.teamcode.internal.HardwareManager;
import org.firstinspires.ftc.teamcode.internal.TelemetryManager;

public class ManualFrontCombine extends ActionElement {

    final public double sensitivity = 0.05;

    @Override
    public void run() throws InterruptedException, NullPointerException {
        FrontCombine frontCombine = (FrontCombine) HardwareManager.ReserveHardware(this, "FrontCombine");
        TelemetryManager.instance.AddFunctionToLogging("FrontCombine Rotation", () -> frontCombine.unitRotate.getPosition());
        TelemetryManager.instance.AddFunctionToLogging("Intake Actuator Position", () -> frontCombine.intakeActuator.getCurrentPosition());

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
            frontCombine.RotateAssembly(HardwareManager.opMode.gamepad2.right_stick_x);
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
