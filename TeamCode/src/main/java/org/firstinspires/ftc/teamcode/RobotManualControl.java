package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actions.AutoHang;
import org.firstinspires.ftc.teamcode.actions.AutoPieceDelivery;
import org.firstinspires.ftc.teamcode.actions.manual.ManualDrive;
import org.firstinspires.ftc.teamcode.actions.manual.ManualFrontCombine;
import org.firstinspires.ftc.teamcode.actions.manual.ManualSlide;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.FrontCombine;
import org.firstinspires.ftc.teamcode.hardware.HangExtention;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.internal.BaseOpMode;
import org.firstinspires.ftc.teamcode.internal.HardwareManager;

@TeleOp(name = "Robot Manual Control", group = "Competition Ready")
public class RobotManualControl extends BaseOpMode {
    // Hardwarez
    private final Drive HW_Drive = new Drive();
    private final Slide HW_Slide = new Slide();
    private final FrontCombine HW_FrontCombine = new FrontCombine();
    private final HangExtention HW_HangExtension = new HangExtention();

    // Manual Actions
    private final ManualDrive AC_Drive = new ManualDrive();
    private final ManualSlide AC_Slide = new ManualSlide();
    private final ManualFrontCombine AC_FrontCombine = new ManualFrontCombine();

    private final AutoPieceDelivery AC_PieceDelivery = new AutoPieceDelivery();

    //private final AutoHang AC_HangExtension = new AutoHang();

    private boolean isShareButtonPressed = false;

    @Override
    public void initializeHardware() {
        // Initialize the hardware
        HardwareManager.init(HW_Drive, hardwareMap);
        HardwareManager.init(HW_Slide, hardwareMap);
        HardwareManager.init(HW_FrontCombine, hardwareMap);
      //  HardwareManager.init(HW_HangExtension, hardwareMap);
    }

    @Override
    public void calibrateHardware() {
        // Calibrate the hardware
        HardwareManager.calibrate(HW_Slide);
        // unsure if this is needed with my lib, but better safe than sorry.
        HardwareManager.calibrate(HW_Drive);
        HardwareManager.calibrate(HW_FrontCombine);

       // HardwareManager.calibrate(HW_HangExtension);
    }

    @Override
    public void main() {
        HardwareManager.StartAction(AC_Drive);
        HardwareManager.StartAction(AC_Slide);
        HardwareManager.StartAction(AC_FrontCombine);
       // HardwareManager.StartAction(AC_HangExtension);
//        HardwareManager.StartAction(AC_HangExtension);


        while (opModeIsActive()) {


//                if (gamepad1.share && !isShareButtonPressed) {
//                    isShareButtonPressed = true;
//                    //HardwareManager.StartAction(AC_HangExtension);
//                } else if (!gamepad1.share) {
//                    isShareButtonPressed = false;
                }
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
        }
    }



