package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.FrontCombine;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.internal.BaseOpMode;
import org.firstinspires.ftc.teamcode.internal.HardwareManager;

@Autonomous(name = "Initialize Manual", group = "Competition Ready")
public class AutoInitForManual extends BaseOpMode {
    // Hardware
    private final Drive HW_Drive = new Drive();
    private final Slide HW_Slide = new Slide();
    private final FrontCombine HW_FrontCombine = new FrontCombine();

    @Override
    public void initializeHardware() {
        // Initialize the hardware

        HardwareManager.init(HW_Drive, hardwareMap);
        HardwareManager.init(HW_Slide, hardwareMap);
        HardwareManager.init(HW_FrontCombine, hardwareMap);

        // We aren't supposed to do this here, but we will, since this isn't a normal opmode
        // Calibrate the hardware
//        try {
//            HardwareManager.waitForCalibrations();
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
        HardwareManager.calibrate(HW_FrontCombine);
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
