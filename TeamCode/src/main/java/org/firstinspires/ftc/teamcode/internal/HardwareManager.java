package org.firstinspires.ftc.teamcode.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.concurrent.CountDownLatch;

public class HardwareManager {
    public static List<HardwareElement> hardwareElements = new ArrayList<>();
    public static List<ActionElement> runningActionElements = new ArrayList<>();
    public static HashMap<HardwareElement, ActionElement> HardwareReserves = new HashMap<>();
    private static final List<ActionElement> stoppedActions = new ArrayList<>();
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
        if (calibrationLatch == null) {
            calibrationLatch = new CountDownLatch(1);
        } else {
            calibrationLatch = new CountDownLatch((int)calibrationLatch.getCount()+1);
        }

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
        // Run the action in a separate thread (with a try-catch-finally)
        action.runThread = new Thread(() -> {
            try {
                action.run();
            } catch (InterruptedException ignored) {
                // Thread interruption handled by finally block
            } catch (Exception e) {
                new Error(action, 204, "An error occurred while running action", e);
            } finally {
                FinishAction(action);
            }
        });
        action.runThread.start();
        runningActionElements.add(action);
    }

    /**
     * Finishes an action and releases all hardware elements reserved by it.
     * This assumes that the thread has already been stopped
     * @param action The action to finish.
     */
    private static void FinishAction(ActionElement action) {
        synchronized (runningActionElements) {
            runningActionElements.remove(action);
        }

        // Release hardware AFTER ensuring thread is stopped
        List<HardwareElement> reservedHardware = new ArrayList<>();
        synchronized (HardwareReserves) {
            Iterator<Map.Entry<HardwareElement, ActionElement>> iterator = HardwareReserves.entrySet().iterator();
            while (iterator.hasNext()) {
                Map.Entry<HardwareElement, ActionElement> entry = iterator.next();
                if (entry.getValue() == action) {
                    HardwareElement hw = entry.getKey();
                    hw.isReserved = false;
                    hw.reservedWithPriority = -1;
                    reservedHardware.add(hw);
                    iterator.remove();
                }
            }
        }

        // Handle auto-restart logic safely
        if (action.isAutoRestart()) {
            synchronized (stoppedActions) {
                stoppedActions.add(action);
            }
            synchronized (actionHardwareMap) {
                actionHardwareMap.put(action, reservedHardware);
            }
        }

        if (!action.isStoppingDueToPriority) {
            checkAndRestartActions();
        }
    }

//    // Call this method when hardware is released
//    private static void releaseHardware(HardwareElement hardwareElement) {
//        hardwareElement.isReserved = false;
//        hardwareElement.reservedWithPriority = -1;
//        HardwareReserves.remove(hardwareElement);
//        checkAndRestartActions();
//    }

    private static void checkAndRestartActions() {
        List<ActionElement> copyStoppedActions;
        synchronized (stoppedActions) {
            copyStoppedActions = new ArrayList<>(stoppedActions); // Thread-safe copy
        }

        PriorityQueue<ActionElement> actionQueue = new PriorityQueue<>(Comparator.comparingInt(a -> -a.getPriority()));
        actionQueue.addAll(copyStoppedActions);

        for (ActionElement action : copyStoppedActions) {
            List<HardwareElement> reservedHardware;
            synchronized (actionHardwareMap) {
                reservedHardware = actionHardwareMap.get(action);
            }

            boolean allAvailable = true;
            if (reservedHardware != null) {
                for (HardwareElement hardwareElement : reservedHardware) {
                    if (hardwareElement.isReserved) {
                        allAvailable = false;
                        break;
                    }
                }
            } else {
                allAvailable = false;
            }

            if (allAvailable) {
                synchronized (stoppedActions) {
                    stoppedActions.remove(action);
                }
                synchronized (actionHardwareMap) {
                    actionHardwareMap.remove(action);
                }
                action.isStoppingDueToPriority = false;
                StartAction(action);
            }

        }
    }


    /**
     * Forcibly stops an action and releases all hardware elements reserved by it.
     * @param action The action to stop.
     */
    public static void StopAction(ActionElement action) {
        try {
            Thread actionThread = action.runThread;
            if (actionThread != null && actionThread.isAlive()) {
                // Avoid interrupting/joining the current thread
                if (Thread.currentThread() != actionThread) {
                    actionThread.interrupt();
                    try {
                        actionThread.join(3000);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        new Error(action, 205, "Interrupted waiting for action to stop", e);
                    }
                    if (actionThread.isAlive()) {
                        new Error(action, 205, "Action stop timeout", null);
                    }
                }
            }
            FinishAction(action);
        } catch (Exception e) {
            new Error(action, 205, "An error occurred while stopping action", e);
            throw new RuntimeException(e);
        }

    }

    /**
     * Reserves a piece of hardware for actions.
     * This will prevent other actions from using the hardware.
     * @param className The class name of the hardware element.
     * @return The hardware element, or an error if not found.
     */
    public static HardwareElement ReserveHardware(ActionElement action, String className) {
        try {
            HardwareElement hw = getElementByClassName(className);
            if (hw == null || !hw.isInitialized || hw.isBroken) {
                new Error(action, 203, "Could not reserve hardware element: " + className, null);
                return null;
            }
            if (hw.isReserved) {
                if (action.getPriority() >= hw.reservedWithPriority) {
                    ActionElement actionToRelease = HardwareReserves.get(hw);
                    if (actionToRelease != null) {
                        try {
                            actionToRelease.isStoppingDueToPriority = true;
                            StopAction(actionToRelease);
                        } catch (Exception e) {
                            new Error(action, 203, "Could not reserve hardware element: " + className, e);
                            return null;
                        }
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
            hw.reservedWithPriority = action.getPriority();
            HardwareReserves.put(hw, action);
            return hw;
        } catch (Exception e) {
            new Error(action, 203, "Could not reserve hardware element: " + className, null);
            return null;
        }
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