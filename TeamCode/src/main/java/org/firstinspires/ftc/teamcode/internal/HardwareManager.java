package org.firstinspires.ftc.teamcode.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.PriorityQueue;
import java.util.concurrent.CountDownLatch;

public class HardwareManager {
    public static List<HardwareElement> hardwareElements = new ArrayList<>();
    public static List<ActionElement> runningActionElements = new ArrayList<>();
    public static HashMap<HardwareElement, ActionElement> HardwareReserves = new HashMap<>();
    private static List<ActionElement> stoppedActions = new ArrayList<>();
    private static HashMap<ActionElement, List<HardwareElement>> actionHardwareMap = new HashMap<>();

    private static volatile boolean opModeActive = false;
    private static CountDownLatch calibrationLatch;
    public static HardwareMap hardwareMap;
    public static OpMode opMode = null;

    //region Initalization & Calibration
    public static Error init(HardwareElement hw, HardwareMap hardwareMap) {
        if (HardwareManager.hardwareMap == null) {
            HardwareManager.hardwareMap = hardwareMap; // im lazy ok?
        }
        if (getElementByClassName(hw.getClass().getSimpleName()) != null) {
            return new Error(hw, 104, "Hardware element already exists: " + hw.getClass().getSimpleName(), null);
        }
        try {
            hw.init(hardwareMap);
            hw.isInitialized = true;
            hardwareElements.add(hw);
            return null;
        } catch (Exception e) {
            return new Error(hw, 103, "An error occurred while initializing hardware", e);
        }
    }

    public static Error init_array(HardwareElement[] elementsToInit, HardwareMap hardwareMap) {
        for (HardwareElement hardwareElement : hardwareElements) {
            Error result = init(hardwareElement, hardwareMap);
            if (result != null) {
                return result; // there's probably a better way to do this.
            }
        }
        return null;
    }

    public static Error calibrate(HardwareElement hw) {
        if (!hw.isInitialized) {
            return new Error(hw, 201, "Could not calibrate hardware element: " + hw.getClass().getSimpleName() + ". Hardware is not initialized.", null);
        }
        try {
            hw.calibrate();
            hw.isCalibrated = true;
            return null;
        } catch (Exception e) {
            return new Error(hw, 202, "An error occurred while calibrating hardware", e);
        }
    }

    public static Error calibrate_async(HardwareElement hw) {
        if (!hw.isInitialized) {
            return new Error(hw, 201, "Could not calibrate hardware element: " + hw.getClass().getSimpleName() + ". Hardware is not initialized.", null);
        }
        calibrationLatch = new CountDownLatch(hardwareElements.size());
        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    hw.calibrate();
                    hw.isCalibrated = true;
                } catch (Exception e) {
                    new Error(hw, 202, "An error occurred while calibrating hardware", e);
                } finally {
                    calibrationLatch.countDown();
                }
            }
        }).start();
        return null;
    }

    public static void waitForCalibrations() throws InterruptedException {
        if (calibrationLatch != null) {
            calibrationLatch.await();
        }
    }

    //endregion

    //region Failure Management
    // i had to name it that

    public static void GracefullyFailHardware(HardwareElement hw) {
        hw.isBroken = true;
        for (ActionElement actionElement : runningActionElements) {
            if (HardwareReserves.get(hw) == actionElement) {
                StopAction(actionElement);
            }
        }

    }

    //endregion

    /**
     * Get a hardware element by its class name.
     * WARNING: This does no checks to see if the device is working.
     * @param className The class name of the hardware element.
     * @return The hardware element, or null if not found.
     */
    private static HardwareElement getElementByClassName(String className) {
        for (HardwareElement hardwareElement : hardwareElements) {
            if (hardwareElement.getClass().getSimpleName().equals(className)) {
                return hardwareElement;
            }
        }
        return null; // Return null if no matching element is found
    }

    //region Actions

    public static void StartAction(ActionElement action) {
        // Run the action in a seperate thread (with a try-catch)
        runningActionElements.add(action);
        action.runThread = new Thread(() -> {
            try {
                action.run();
                FinishAction(action);
            } catch (Exception e) {
                new Error(action, 204, "An error occurred while running action", e);
            }
        });
        action.runThread.start();
    }

    /**
     * Finishes an action and releases all hardware elements reserved by it.
     * This assumes that the thread has already been stopped
     * @param action The action to finish.
     */
    private static void FinishAction(ActionElement action) {
        runningActionElements.remove(action);
        action.runThread = null;

        List<HardwareElement> reservedHardware = new ArrayList<>();
        for (HardwareElement hardwareElement : HardwareReserves.keySet()) {
            if (HardwareReserves.get(hardwareElement) == action) {
                reservedHardware.add(hardwareElement);
                init(hardwareElement, hardwareMap);
                hardwareElement.isReserved = false;
                hardwareElement.reservedWithPriority = -1;
                HardwareReserves.remove(hardwareElement);
            }
        }

        if (action.autoRestart) {
            stoppedActions.add(action);
            actionHardwareMap.put(action, reservedHardware);
        }

        checkAndRestartActions();
    }

//    // Call this method when hardware is released
//    private static void releaseHardware(HardwareElement hardwareElement) {
//        hardwareElement.isReserved = false;
//        hardwareElement.reservedWithPriority = -1;
//        HardwareReserves.remove(hardwareElement);
//        checkAndRestartActions();
//    }

    private static void checkAndRestartActions() {
        PriorityQueue<ActionElement> actionQueue = new PriorityQueue<>(Comparator.comparingInt(a -> -a.priority));
        actionQueue.addAll(stoppedActions);

        while (!actionQueue.isEmpty()) {
            ActionElement action = actionQueue.poll();
            List<HardwareElement> reservedHardware = actionHardwareMap.get(action);
            boolean allAvailable = true;
            for (HardwareElement hardwareElement : reservedHardware) {
                if (hardwareElement.isReserved) {
                    allAvailable = false;
                    break;
                }
            }
            if (allAvailable) {
                StartAction(action);
                stoppedActions.remove(action);
                actionHardwareMap.remove(action);
            }
        }
    }

    /**
     * Forcibly stops an action and releases all hardware elements reserved by it.
     * @param action The action to stop.
     */
    public static void StopAction(ActionElement action) {
        action.runThread.interrupt();
        FinishAction(action);
    }

    /**
     * Reserves a piece of hardware for actions.
     * This will prevent other actions from using the hardware.
     * @param className The class name of the hardware element.
     * @return The hardware element, or an error if not found.
     */
    public static HardwareElement ReserveHardware(ActionElement action, String className) {
        HardwareElement hw = getElementByClassName(className);
        if (hw == null || !hw.isInitialized || hw.isBroken) {
            new Error(action, 203, "Could not reserve hardware element: " + className, null);
            return null;
        }
        if (hw.isReserved) {
            if (action.priority >= hw.reservedWithPriority) {
                ActionElement actionToRelease = HardwareReserves.get(hw);
                if (actionToRelease != null) {
                    StopAction(actionToRelease);
                } else {
                    new Error(action, 203, "Could not reserve hardware element: " + className, null);
                    return null;
                }
            } else {
                new Error(action, 203, "Could not reserve hardware element: " + className, null);
                return null;
            }
        }
        hw.isReserved = true;
        hw.reservedWithPriority = action.priority;
        HardwareReserves.put(hw, action);
        return hw;
    }
    //endregion

    //region OpMode Lifecycle

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
                                        new Error(hardwareElement, 102, "An error in hardware update thread", e);
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
        for (ActionElement actionElement : runningActionElements) {
            StopAction(actionElement);
        }
        for (HardwareElement hardwareElement : hardwareElements) {
            if (hardwareElement.updateThread != null) {
                hardwareElement.updateThread.interrupt();
            }
            hardwareElement.stop();
        }
    }
    //endregion
}