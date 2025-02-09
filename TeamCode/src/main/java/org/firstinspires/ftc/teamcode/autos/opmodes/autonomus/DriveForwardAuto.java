package org.firstinspires.ftc.teamcode.autos.opmodes.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BotInitialization;
import org.firstinspires.ftc.teamcode.CrossOpModeData;
import org.firstinspires.ftc.teamcode.DeepHorOpMode;

@Autonomous(name="the most basic auto mode", group = "Competition Ready")
public class DriveForwardAuto extends DeepHorOpMode {
    @Override
    public void runOpMode() {
        //region Hardware Initialization
        ConfigureHardware(true);
        if (!CrossOpModeData.isInitialized) {
            BotInitialization.InitializeRobot(this);
            CrossOpModeData.isInitialized = true;
        }
        //endregion

//        stage1Arm.setTargetPosition(255);
//        stage1Arm.setPower(0.3);

        waitForStart();

        ElapsedTime runtime = new ElapsedTime();
        MoveRobot(1, 0, 0);

        while (opModeIsActive() && runtime.seconds() < 1.0) {
            UpdateMoveRobot(0.5);
            telemetry.addData("Status", "Running");
            telemetry.addData("Time", runtime.seconds());
            telemetry.update();
        }

        // Stop the robot after 3 seconds
        MoveRobot(0, 0, 0);
    }
}
