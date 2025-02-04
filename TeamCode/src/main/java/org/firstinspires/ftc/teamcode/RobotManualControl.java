package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actions.AutoPieceDelivery;
import org.firstinspires.ftc.teamcode.actions.manual.ManualBucket;
import org.firstinspires.ftc.teamcode.actions.manual.ManualCollector;
import org.firstinspires.ftc.teamcode.actions.manual.ManualDrive;
import org.firstinspires.ftc.teamcode.actions.manual.ManualSlide;
import org.firstinspires.ftc.teamcode.hardware.Bucket;
import org.firstinspires.ftc.teamcode.hardware.Collector;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.internal.BaseOpMode;
import org.firstinspires.ftc.teamcode.internal.HardwareManager;

@TeleOp(name = "Robot Manual Control", group = "Competition Ready")
public class RobotManualControl extends BaseOpMode {
    // Hardwarez
    private final Bucket HW_Bucket = new Bucket();
    private final Collector HW_Collector = new Collector();
    private final Drive HW_Drive = new Drive();
    private final Slide HW_Slide = new Slide();

    // Manual Actions
    private final ManualBucket AC_Bucket = new ManualBucket();
    private final ManualCollector AC_Collector = new ManualCollector();
    private final ManualDrive AC_Drive = new ManualDrive();
    private final ManualSlide AC_Slide = new ManualSlide();

    private final AutoPieceDelivery AC_PieceDelivery = new AutoPieceDelivery();

    private boolean isShareButtonPressed = false;

    @Override
    public void initializeHardware() {
        // Initialize the hardware
        HardwareManager.init(HW_Bucket, hardwareMap);
        HardwareManager.init(HW_Collector, hardwareMap);
        HardwareManager.init(HW_Drive, hardwareMap);
        HardwareManager.init(HW_Slide, hardwareMap);
    }

    @Override
    public void calibrateHardware() {
        // Calibrate the hardware
        HardwareManager.calibrate_async(HW_Bucket);
        HardwareManager.calibrate_async(HW_Collector);
        try {
            HardwareManager.waitForCalibrations();
        } catch (InterruptedException e) {
            return;
        }
        HardwareManager.calibrate(HW_Slide);
        // unsure if this is needed with my lib, but better safe than sorry.
        HardwareManager.calibrate(HW_Drive);
    }

    @Override
    public void main() {
        HardwareManager.StartAction(AC_Drive);
        HardwareManager.StartAction(AC_Slide);
        HardwareManager.StartAction(AC_Collector);
        HardwareManager.StartAction(AC_Bucket);

        while (opModeIsActive()) {
            try {
                if (gamepad1.share && !isShareButtonPressed) {
                    isShareButtonPressed = true;
                    HardwareManager.StartAction(AC_PieceDelivery);
                } else if (!gamepad1.share) {
                    isShareButtonPressed = false;
                }
                Thread.sleep(20);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

}

