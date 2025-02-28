package org.firstinspires.ftc.teamcode.internal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public abstract class BaseOpMode extends LinearOpMode {
    public TelemetryManager telemetryManager = null;
    public boolean isStopped = false;

    public void runOpMode() {
        HardwareManager.Reset();
        try {
            telemetryManager = new TelemetryManager(telemetry, hardwareMap.appContext);
            telemetryManager.StartTelemetryLoop();
            HardwareManager.opMode = this;
            if (!opModeIsActive() && !opModeInInit()) {
                stopOpMode();
                return;
            }
            telemetryManager.ModifyMessagesLogged("Robot Status", "Initializing Hardware");
            initializeHardware();
            if (!opModeIsActive() && !opModeInInit()) {
                stopOpMode();
                return;
            }
            telemetryManager.ModifyMessagesLogged("Robot Status", "Calibrating Hardware");
            calibrateHardware();
            if (!opModeIsActive() && !opModeInInit()) {
                stopOpMode();
                return;
            }
            telemetryManager.ModifyMessagesLogged("Robot Status", "Waiting for start...");
            waitForStart();
            if (!opModeIsActive() && !opModeInInit()) {
                stopOpMode();
                return;
            }
            HardwareManager.onOpModeStart();
            if (!opModeIsActive() && !opModeInInit()) {
                stopOpMode();
                return;
            }
            telemetryManager.ModifyMessagesLogged("Robot Status", "Running!");
            main();
            telemetryManager.ModifyMessagesLogged("Robot Status", "Stopping...");
        } catch (Exception e) {

        }
        finally {
            stopOpMode();
        }
    }

    private void stopOpMode() {
        isStopped = true;
    }

    /**
     * This class will always be called at the start of the opmode.
     * This is where you should initialize all of your hardware.
     * To initialize your hardware, use HardwareManager.init(hardwareElement, hardwareMap);
     */
    public abstract void initializeHardware();

    /**
     * This class will be called at the start of the opmode.
     * This is where you should calibrate all of your hardware.
     * To calibrate, use HardwareManager.calibrate() or HardwareManager.calibrate_async() with HardwareManager.waitForCalibrations();
     * This will ALWAYS run. Verify that the hardware hasn't been calibrated before.
     */
    public abstract void calibrateHardware();

    public abstract void main();
}
