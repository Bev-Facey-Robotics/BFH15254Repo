package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.hardware.HangExtention;
import org.firstinspires.ftc.teamcode.internal.ActionElement;
import org.firstinspires.ftc.teamcode.internal.HardwareManager;
import org.firstinspires.ftc.teamcode.internal.TelemetryManager;

public class AutoHang extends ActionElement {
    @Override
    public void run() throws InterruptedException, NullPointerException {
        // Reserve the hardware
        HangExtention hangExtention = (HangExtention) HardwareManager.ReserveHardware(this, "HangExtention");
        TelemetryManager.instance.AddFunctionToLogging("HangExtention Position 1", () -> hangExtention.motorHangExtension1.getCurrentPosition());
        TelemetryManager.instance.AddFunctionToLogging("HangExtention Position 2", () -> hangExtention.motorHangExtension2.getCurrentPosition());
        hangExtention.MoveToPosition(100);
        while (!Thread.currentThread().isInterrupted()) {
            Thread.sleep(20);
        }
    }

    @Override
    public boolean isAutoRestart() {
        return false;
    }

    @Override
    public int getPriority() {
        return 10;
    }
}
