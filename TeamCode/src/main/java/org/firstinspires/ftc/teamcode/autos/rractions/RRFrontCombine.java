package org.firstinspires.ftc.teamcode.autos.rractions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.FrontCombine;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.internal.HardwareManager;

public class RRFrontCombine {
    private FrontCombine frontCombine = new FrontCombine();

    public RRFrontCombine(HardwareMap hardwareMap) {
        HardwareManager.init(frontCombine, hardwareMap);
        HardwareManager.calibrate(frontCombine);
        HardwareManager.ReserveHardwareForRoadRunner("FrontCombine");
    }

    public class MoveToPickup implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) throws NullPointerException {
            //Logic for the slide to check where it is and return either true or false
            frontCombine.SetIntakeActive(true);
            return frontCombine.intakeActuator.isBusy();

        }
    }
    public Action MoveToPickup() {return new RRFrontCombine.MoveToPickup();}
}
