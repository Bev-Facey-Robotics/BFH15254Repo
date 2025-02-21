package org.firstinspires.ftc.teamcode.autos.opmodes.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.actions.DriveForward;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.internal.BaseOpMode;
import org.firstinspires.ftc.teamcode.internal.HardwareManager;

/**
 * This drives the robot forwards for 1 second to get into the zone for some points.
 * This also acts as a basic calibrator.
 */
@Autonomous(name="the most basic auto mode", group = "Competition Ready")
public class DriveForwardAuto extends BaseOpMode {
    private final Drive HW_Drive = new Drive();
    private final Slide HW_Slide = new Slide();

    @Override
    public void initializeHardware() {
        // Initialize the hardware
        HardwareManager.init(HW_Drive, hardwareMap);
        HardwareManager.init(HW_Slide, hardwareMap);
    }

    @Override
    public void calibrateHardware() {
        // Calibrate the bare minimum hardware hardware
        HardwareManager.calibrate(HW_Drive);
    }

    @Override
    public void main() {
        // Drive forward for 1 second
        DriveForward driveForward = new DriveForward();
        HardwareManager.StartAction(driveForward);
        try {
            Thread.sleep(3000); // TODO: This should be replaced with an actual wait for the action to finish.
        } catch (InterruptedException e) {
            return;
        }
    }

    private void calibrateRest() {
        HardwareManager.calibrate(HW_Slide);
    }
}
