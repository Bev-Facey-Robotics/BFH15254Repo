package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.hardware.Bucket;
import org.firstinspires.ftc.teamcode.hardware.Collector;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.internal.ActionElement;
import org.firstinspires.ftc.teamcode.internal.HardwareManager;
import org.firstinspires.ftc.teamcode.internal.TelemetryManager;

public class AutoPieceDelivery extends ActionElement {
    @Override
    public void run() throws InterruptedException {
        // Reserve the hardware
        Collector collector = (Collector) HardwareManager.ReserveHardware(this,"Collector");
        Slide slide = (Slide) HardwareManager.ReserveHardware(this,"Slide");
        Bucket bucket = (Bucket) HardwareManager.ReserveHardware(this,"Bucket");

        bucket.isBucketSyncRunning = true;

        // This automatically moves the slide & arm to the correct position to transfer, then raises the slide & piece bucket to be delivered into the bucket
        bucket.bucketTargetPosition = 0.05;
        slide.MovePosition(-1821);
        Thread.sleep(2000);
        // wait for slide to move down
        TelemetryManager.instance.AddFunctionToLogging("Slide Position", () -> slide.motorSlide.getCurrentPosition());
        while (slide.motorSlide.isBusy()) {
            // wait for slide to move down
            Thread.sleep(20);
        }

        collector.MoveArmToPosition(255, 0.3);
        TelemetryManager.instance.AddFunctionToLogging("Arm Position", () -> collector.arm.getCurrentPosition());

        while (collector.arm.isBusy()) {
            // wait for the arm to move down
            Thread.sleep(20);
        }

        collector.RotateScoop(1);
        Thread.sleep(3000);

        collector.RotateScoop(0);
        collector.MoveArmToPosition(5, 0.1);
        Thread.sleep(500);
        bucket.bucketTargetPosition = -0.1;

        // Move slide to the top
        slide.MovePosition(-10800);
        TelemetryManager.instance.AddFunctionToLogging("Slide Position", () -> slide.motorSlide.getCurrentPosition());
        while (slide.motorSlide.isBusy()) {
            // wait for slide to move down
            Thread.sleep(20);
        }

        // Move the swing to correct
        bucket.MoveArmToPosition(-130, 0.5);
        Thread.sleep(7000);

        // dump the piece in the basket
        bucket.bucketTargetPosition = 0.2;
        Thread.sleep(3000);

        bucket.MoveArmToPosition(-130, 0.3);
        TelemetryManager.instance.AddFunctionToLogging("Arm #2 Position", () -> bucket.arm.getCurrentPosition());
        while (bucket.arm.isBusy()) {
            // wait for the arm to down
            Thread.sleep(20);
        }
    }
    @Override
    public boolean isAutoRestart() {
        return false;
    }
    @Override
    public int getPriority() {
        return 9;
    }
}
