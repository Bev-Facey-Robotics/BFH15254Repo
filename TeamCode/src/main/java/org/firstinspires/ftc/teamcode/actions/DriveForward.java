package org.firstinspires.ftc.teamcode.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.internal.ActionElement;
import org.firstinspires.ftc.teamcode.internal.HardwareManager;
import org.firstinspires.ftc.teamcode.internal.TelemetryManager;

/**
 * This just drives the robot forward for 3 seconds
 */
public class DriveForward extends ActionElement {
    @Override
    public void run() throws InterruptedException, NullPointerException  {
        Drive drive = (Drive)HardwareManager.ReserveHardware(this, "Drive");

        ElapsedTime runtime = new ElapsedTime();
        drive.move(1, 0, 0);
        TelemetryManager.instance.AddFunctionToLogging("Time Left", runtime::seconds);
        while (runtime.seconds() < 1.0) {
            // wait for time to be up
            Thread.sleep(60);
        }

        // Stop the robot after 3 seconds
        drive.move(0, 0, 0);
    }

    @Override
    public boolean isAutoRestart() {
        return false;
    }
    @Override
    public int getPriority() {
        return 2;
    }
}