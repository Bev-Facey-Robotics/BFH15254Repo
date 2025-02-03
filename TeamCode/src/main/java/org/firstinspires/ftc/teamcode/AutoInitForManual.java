package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.internal.BaseOpMode;

@Autonomous(name = "Initialize Manual", group = "Competition Ready")
public class AutoInitForManual extends BaseOpMode {
    @Override
    public void runOpMode() {
        ConfigureHardware(true);
        BotInitialization.InitializeRobot(this);
        telemetry.addLine("Ready to Start Manual");

        CrossOpModeData.isInitialized = true;

        telemetry.update();
    }
}
