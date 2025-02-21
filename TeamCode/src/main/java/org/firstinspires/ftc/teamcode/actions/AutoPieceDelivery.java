package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.internal.ActionElement;
import org.firstinspires.ftc.teamcode.internal.HardwareManager;
import org.firstinspires.ftc.teamcode.internal.TelemetryManager;

public class AutoPieceDelivery extends ActionElement {
    @Override
    public void run() throws InterruptedException, NullPointerException {
        // Reserve the hardware
//        Collector collector = (Collector) HardwareManager.ReserveHardware(this, "Collector");
//        Slide slide = (Slide) HardwareManager.ReserveHardware(this, "Slide");
//        Bucket bucket = (Bucket) HardwareManager.ReserveHardware(this, "Bucket");
//
//        try {
//            bucket.isBucketSyncRunning = true;
//
//            bucket.bucketTargetPosition = 0.05;
//            slide.MovePosition(-1821);
//
//            // Replace long sleeps with interruptible loops
//            waitMillis(2000);
//
//            TelemetryManager.instance.AddFunctionToLogging("Slide Position", () -> slide.motorSlide.getCurrentPosition());
//            while (slide.motorSlide.isBusy() && !Thread.currentThread().isInterrupted()) {
//                Thread.sleep(20);
//            }
//
//            collector.MoveArmToPosition(255, 0.3);
//            TelemetryManager.instance.AddFunctionToLogging("Arm Position", () -> collector.arm.getCurrentPosition());
//
//            while (collector.arm.isBusy() && !Thread.currentThread().isInterrupted()) {
//                Thread.sleep(20);
//            }
//
//            collector.RotateScoop(1);
//            waitMillis(3000);
//
//            collector.RotateScoop(0);
//            collector.MoveArmToPosition(5, 0.1);
//            waitMillis(500);
//
//            bucket.bucketTargetPosition = -0.1;
//
//            slide.MovePosition(-10800);
//            TelemetryManager.instance.AddFunctionToLogging("Slide Position", () -> slide.motorSlide.getCurrentPosition());
//            while (slide.motorSlide.isBusy() && !Thread.currentThread().isInterrupted()) {
//                Thread.sleep(20);
//            }
//
//            bucket.MoveArmToPosition(-130, 0.5);
//            waitMillis(7000);
//
//            bucket.bucketTargetPosition = 0.2;
//            waitMillis(3000);
//
//            bucket.MoveArmToPosition(-130, 0.3);
//            TelemetryManager.instance.AddFunctionToLogging("Arm #2 Position", () -> bucket.arm.getCurrentPosition());
//            while (bucket.arm.isBusy() && !Thread.currentThread().isInterrupted()) {
//                Thread.sleep(20);
//            }
//        } finally {
//            // Cleanup hardware state
//            bucket.isBucketSyncRunning = false;
//        }
    }

    // Helper method to wait with interruption checks
    private void waitMillis(long millis) throws InterruptedException {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < millis && !Thread.currentThread().isInterrupted()) {
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