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
    public void init() {
        ConfigureHardware();
        BotInitialization.InitializeRobot(this);
        telemetry.addLine("Ready to Start Manual");

        long bootTime = System.currentTimeMillis() - SystemClock.elapsedRealtime();

        try {
            FileUtils.writeToFile(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS) + "AutoInitForManual.txt", "" + bootTime);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        telemetry.update();
    }
    @Override
    public void loop() {

    }

}
