package org.firstinspires.ftc.teamcode;

import android.os.Environment;
import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

@Autonomous(name = "Initialize Manual", group = "Competition Ready")
public class AutoInitForManual extends DeepHorOpMode {
    @Override
    public void runOpMode() {
        ConfigureHardware();
        BotInitialization.InitializeRobot(this);
        telemetry.addLine("Ready to Start Manual");

        CrossOpModeData.isInitialized = true;

        telemetry.update();
    }
}
