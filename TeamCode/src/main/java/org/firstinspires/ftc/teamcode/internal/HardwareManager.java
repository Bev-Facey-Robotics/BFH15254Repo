package org.firstinspires.ftc.teamcode.internal;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

public class HardwareManager {
    public static List<HardwareElement> hardwareElements = new ArrayList<HardwareElement>();
    private static volatile boolean opModeActive = false;

    public static void init_array(HardwareElement[] hardwareElements, HardwareMap hardwareMap) {
        for (HardwareElement hardwareElement : hardwareElements) {
            try {
                hardwareElement.init(hardwareMap);
            } catch (Exception e) {
                hardwareElement.isBroken = true;
            }
        }
    }

    public static void init(HardwareElement hw, HardwareMap hardwareMap) {
        try {
            hw.init(hardwareMap);
        } catch (Exception e) {
            hw.isBroken = true;
        }
    }

    public static void calibrate_array(HardwareElement[] hardwareElements) {
        for (HardwareElement hardwareElement : hardwareElements) {
            try {
                hardwareElement.calibrate();
            } catch (Exception e) {
                hardwareElement.isBroken = true;
            }
        }
    }

    public static void onOpModeStart() {
        opModeActive = true;
        for (HardwareElement hardwareElement : hardwareElements) {
                try {
                    Method updateMethod = hardwareElement.getClass().getMethod("update");
                    if (updateMethod.getDeclaringClass() != HardwareElement.class) {
                        hardwareElement.updateThread = new Thread(new Runnable() {
                            @Override
                            public void run() {
                                while (opModeActive && !Thread.currentThread().isInterrupted()) {
                                    try {
                                        hardwareElement.update();
                                        Thread.sleep(2);
                                    } catch (Exception e) {
                                        new Error(hardwareElement, 2, "An error in hardware update thread", e);
                                        return;
                                    }
                                }
                            }
                        });
                        hardwareElement.updateThread.start();
                    }
                } catch (NoSuchMethodException e) {
                    // Method not found, do nothing
                }

        }
    }

    public static void onOpModeStop() {
        opModeActive = false;
        for (HardwareElement hardwareElement : hardwareElements) {
            if (hardwareElement.updateThread != null) {
                hardwareElement.updateThread.interrupt();
            }
            hardwareElement.stop();
        }
    }
}