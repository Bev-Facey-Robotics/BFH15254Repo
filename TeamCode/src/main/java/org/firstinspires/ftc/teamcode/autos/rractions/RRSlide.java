package org.firstinspires.ftc.teamcode.autos.rractions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.internal.HardwareManager;

public class RRSlide {

    private final Slide slide = new Slide();

    public RRSlide(HardwareMap hardwareMap) {
        HardwareManager.init(slide, hardwareMap);
        slide.calibrate();
        slide.isCalibrated = true;
        HardwareManager.ReserveHardwareForRoadRunner("Slide");
    }

    public class MoveToHighChamber implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) throws NullPointerException {
            packet.put("SlidePos", slide.motorSlide.getCurrentPosition());
            //Logic for the slide to check where it is and return either true or false
            slide.MovePosition(1335);
            return slide.motorSlide.isBusy();

        }
    }
    public Action MoveToHighChamber() {return new MoveToHighChamber();}

    public class ReleaseSpeciminFromHigh implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) throws NullPointerException {
            packet.put("SlidePos", slide.motorSlide.getCurrentPosition());
            //Logic for the slide to check where it is and return either true or false

            slide.MovePositionWithPower(561, 0.1);
            //Thread.sleep();
            return slide.motorSlide.isBusy();

        }
    }
    public Action ReleaseSpeciminFromHigh() {return new ReleaseSpeciminFromHigh();}

    public class MoveToWall implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) throws NullPointerException {
            packet.put("SlidePos", slide.motorSlide.getCurrentPosition());
            slide.MovePosition(0);
            return slide.motorSlide.isBusy();
        }
    }
    public Action MoveToWall() {return new MoveToWall();}

    public class ReleaseSpeciFromWall implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) throws NullPointerException {
            packet.put("SlidePos", slide.motorSlide.getCurrentPosition());
            slide.MovePosition(0);
            return slide.motorSlide.isBusy();
        }
    }
    public Action ReleaseSpeciFromWall() {return new ReleaseSpeciFromWall();}
}
