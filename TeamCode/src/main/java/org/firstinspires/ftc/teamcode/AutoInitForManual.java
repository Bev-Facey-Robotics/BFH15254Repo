package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Bucket;
import org.firstinspires.ftc.teamcode.hardware.Collector;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.internal.BaseOpMode;
import org.firstinspires.ftc.teamcode.internal.HardwareManager;

@Autonomous(name = "Initialize Manual", group = "Competition Ready")
public class AutoInitForManual extends BaseOpMode {
    // Hardware
    private final Bucket HW_Bucket = new Bucket();
    private final Collector HW_Collector = new Collector();
    private final Drive HW_Drive = new Drive();
    private final Slide HW_Slide = new Slide();

    @Override
    public void initializeHardware() {
        // Initialize the hardware
        HardwareManager.init(HW_Bucket, hardwareMap);
        HardwareManager.init(HW_Collector, hardwareMap);
        HardwareManager.init(HW_Drive, hardwareMap);
        HardwareManager.init(HW_Slide, hardwareMap);

        // We aren't supposed to do this here, but we will, since this isn't a normal opmode
        // Calibrate the hardware
        HardwareManager.calibrate_async(HW_Bucket);
        HardwareManager.calibrate_async(HW_Collector);
        try {
            HardwareManager.waitForCalibrations();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        HardwareManager.calibrate(HW_Slide);
        // unsure if this is needed with my lib, but better safe than sorry.
        HardwareManager.calibrate(HW_Drive);

        // Now stop the opmode
        requestOpModeStop();
    }

    public void calibrateHardware() {
        // blank
    }

    public void main() {
        // blank
    }
}
