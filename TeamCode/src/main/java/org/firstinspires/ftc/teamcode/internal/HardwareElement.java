package org.firstinspires.ftc.teamcode.internal;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class HardwareElement {
    public boolean isReserved = false;
    public boolean isInitialized = false;
    public boolean isCalibrated = false;
    public boolean isBroken = false;
    public int reservedWithPriority = -1;

    public Thread updateThread = null;

    /**
     * Use this to initialize your hardware devices and prepare to run the op mode.
     * @param hardwareMap The FTC hardware map for defining hardware.
     */
    public abstract void init(HardwareMap hardwareMap);

    /**
     * This function will run once to calibrate the hardware.
     * This only occurs on fresh starts of the robot.
     * Do not rely on anything in this function being called for
     * operation.
     */
    public abstract void calibrate();

    public abstract void self_test();

    /**
     * This function will run as fast as possible.
     * This runs in it's own thread, so don't be too conserved about using too much time.
     * If an error occurs, it will disable this bit of hardware.
     */
    public void update() {
        // Default implementation does nothing
    }

    /**
     * If a hardware problem occurs, or when we are stopping the robot, this function will be called.
     * This function should, for example, stop all motors and set all servos to a safe position.
     */
    public abstract void stop();
}
