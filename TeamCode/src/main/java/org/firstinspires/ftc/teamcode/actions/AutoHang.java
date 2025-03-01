package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.hardware.HangExtention;
import org.firstinspires.ftc.teamcode.internal.ActionElement;
import org.firstinspires.ftc.teamcode.internal.HardwareManager;

public class AutoHang extends ActionElement {
    @Override
    public void run() throws InterruptedException, NullPointerException {
        // Reserve the hardware
        HangExtention hangExtention = (HangExtention) HardwareManager.ReserveHardware(this, "HangExtention");
        hangExtention.MoveToPosition(100);
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
