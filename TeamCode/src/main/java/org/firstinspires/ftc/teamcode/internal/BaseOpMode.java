package org.firstinspires.ftc.teamcode.internal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public abstract class BaseOpMode extends LinearOpMode {
    public TelemetryManager telemetryManager = new TelemetryManager(telemetry, hardwareMap.appContext);

    public void runOpMode() {
        HardwareManager.opMode = this;
        initializeHardware();
        calibrateHardware();
        waitForStart();
        HardwareManager.onOpModeStart();
        main();
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
