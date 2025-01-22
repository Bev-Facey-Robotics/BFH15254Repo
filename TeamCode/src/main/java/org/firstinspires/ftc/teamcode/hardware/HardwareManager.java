package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareManager {
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

    public static void update(HardwareElement[] hardwareElements) {
        for (HardwareElement hardwareElement : hardwareElements) {
            hardwareElement.update();
        }
    }

    public static void stop(HardwareElement[] hardwareElements) {
        for (HardwareElement hardwareElement : hardwareElements) {
            hardwareElement.stop();
        }
    }
}
