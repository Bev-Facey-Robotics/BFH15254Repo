package org.firstinspires.ftc.teamcode.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Collections;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ConcurrentHashMap;

public class HardwareManager {
    private static final Object initLock = new Object();
    private static final Object calibrationLock = new Object();
    private static final Object actionLock = new Object();

    private static final List<HardwareElement> hardwareElements =
            Collections.synchronizedList(new ArrayList<>());
    private static final List<ActionElement> runningActionElements =
            Collections.synchronizedList(new ArrayList<>());
    private static final Map<HardwareElement, ActionElement> hardwareReserves =
            Collections.synchronizedMap(new HashMap<>());
    private static final List<ActionElement> stoppedActions =
            Collections.synchronizedList(new ArrayList<>());
    private static final Map<ActionElement, List<HardwareElement>> actionHardwareMap =
            Collections.synchronizedMap(new HashMap<>());

    public static volatile boolean opModeActive = false;
    private static volatile CountDownLatch calibrationLatch;
    public static volatile HardwareMap hardwareMap;
    public static volatile BaseOpMode opMode = null;
    public static volatile boolean isCurrentlyStopping = false;

    public static void Reset() {
        synchronized (initLock) {
            hardwareElements.clear();
            runningActionElements.clear();
            hardwareReserves.clear();
            stoppedActions.clear();
            actionHardwareMap.clear();
            calibrationLatch = null;
            hardwareMap = null;
            opMode = null;
            isCurrentlyStopping = false;
        }
    }

    public static Error init(HardwareElement hw, HardwareMap hwMap) {
        synchronized (initLock) {
            if (hardwareMap == null) {
                hardwareMap = hwMap;
            }

            HardwareElement existing = getElementByClassName(hw.getClass().getSimpleName());
            if (existing != null) {
                hardwareElements.remove(existing);
                return init(hw, hwMap);
            }

            try {
                hw.init(hwMap);
                hw.isInitialized = true;
                hardwareElements.add(hw);
                return null;
            } catch (Exception e) {
                return new Error(hw, 103, "An error occurred while initializing hardware", e);
            }
        }
    }

    public static Error init_array(HardwareElement[] elementsToInit, HardwareMap hwMap) {
        synchronized (initLock) {
            for (HardwareElement element : elementsToInit) {
                Error result = init(element, hwMap);
                if (result != null) {
                    return result;
                }
            }
            return null;
        }
    }

    public static Error calibrate(HardwareElement hw) {
        synchronized (calibrationLock) {
            if (!hw.isInitialized) {
                return new Error(hw, 201,
                        "Could not calibrate hardware element: " + hw.getClass().getSimpleName() +
                                ". Hardware is not initialized.", null);
            }
            try {
                hw.calibrate();
                hw.isCalibrated = true;
                return null;
            } catch (Exception e) {
                return new Error(hw, 202, "An error occurred while calibrating hardware", e);
            }
        }
    }

    public static Error calibrate_async(HardwareElement hw) {
        synchronized (calibrationLock) {
            if (!hw.isInitialized) {
                return new Error(hw, 201,
                        "Could not calibrate hardware element: " + hw.getClass().getSimpleName() +
                                ". Hardware is not initialized.", null);
            }

            if (calibrationLatch == null) {
                calibrationLatch = new CountDownLatch(1);
            } else {
                calibrationLatch = new CountDownLatch((int)calibrationLatch.getCount() + 1);
            }

            new Thread(() -> {
                try {
                    hw.calibrate();
                    hw.isCalibrated = true;
                } catch (Exception e) {
                    new Error(hw, 202, "An error occurred while calibrating hardware", e);
                } finally {
                    calibrationLatch.countDown();
                }
            }).start();
            return null;
        }
    }

    public static void waitForCalibrations() throws InterruptedException {
        CountDownLatch latch = calibrationLatch;
        if (latch != null) {
            latch.await();
        }
    }

    public static void GracefullyFailHardware(HardwareElement hw) {
        synchronized (actionLock) {
            hw.isBroken = true;
            for (ActionElement action : runningActionElements) {
                if (hardwareReserves.get(hw) == action) {
                    StopAction(action);
                }
            }
        }
    }

    private static HardwareElement getElementByClassName(String className) {
        synchronized (hardwareElements) {
            for (HardwareElement element : hardwareElements) {
                if (element.getClass().getSimpleName().equals(className)) {
                    return element;
                }
            }
            return null;
        }
    }

    public static void StartAction(ActionElement action) {
        synchronized (actionLock) {
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
    }

    private static void FinishAction(ActionElement action) {
        synchronized (actionLock) {
            runningActionElements.remove(action);

            List<HardwareElement> hardwareToRelease = new ArrayList<>();
            for (Map.Entry<HardwareElement, ActionElement> entry :
                    new HashMap<>(hardwareReserves).entrySet()) {
                if (entry.getValue() == action) {
                    hardwareToRelease.add(entry.getKey());
                }
            }

            for (HardwareElement hw : hardwareToRelease) {
                hw.isReserved = false;
                hw.reservedWithPriority = -1;
                hardwareReserves.remove(hw);
            }

            if (action.isAutoRestart() && !action.isStoppingDueToError) {
                stoppedActions.add(action);
                actionHardwareMap.put(action, hardwareToRelease);
            }

            if (!action.isStoppingDueToPriority) {
                checkAndRestartActions();
            }
        }
    }

    private static void checkAndRestartActions() {
        synchronized (actionLock) {
            List<ActionElement> copyStoppedActions = new ArrayList<>(stoppedActions);
            PriorityQueue<ActionElement> actionQueue = new PriorityQueue<>(
                    Comparator.comparingInt(a -> -a.getPriority()));
            actionQueue.addAll(copyStoppedActions);

            for (ActionElement action : copyStoppedActions) {
                List<HardwareElement> reservedHardware = actionHardwareMap.get(action);
                if (reservedHardware == null) continue;

                boolean allAvailable = true;
                for (HardwareElement hw : reservedHardware) {
                    if (hw.isReserved) {
                        allAvailable = false;
                        break;
                    }
                }

                if (allAvailable) {
                    stoppedActions.remove(action);
                    actionHardwareMap.remove(action);
                    action.isStoppingDueToPriority = false;
                    StartAction(action);
                }
            }
        }
    }

    public static void StopAction(ActionElement action) {
        synchronized (actionLock) {
            Thread actionThread = action.runThread;
            if (actionThread != null && actionThread.isAlive() &&
                    Thread.currentThread() != actionThread) {
                actionThread.interrupt();
                try {
                    actionThread.join(3000);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    new Error(action, 205, "Interrupted while waiting for action to stop", e);
                }
                if (actionThread.isAlive()) {
                    new Error(action, 205, "Action stop timeout", null);
                }
            }
            FinishAction(action);
        }
    }

    public static HardwareElement ReserveHardware(ActionElement action, String className) {
        synchronized (actionLock) {
            try {
                HardwareElement hw = getElementByClassName(className);
                if (hw == null || !hw.isInitialized || hw.isBroken) {
                    new Error(action, 203, "Could not reserve hardware element: " + className, null);
                    return null;
                }

                synchronized (hw) {
                    if (hw.isReserved) {
                        if (action.getPriority() >= hw.reservedWithPriority) {
                            ActionElement actionToRelease = hardwareReserves.get(hw);
                            if (actionToRelease != null) {
                                actionToRelease.isStoppingDueToPriority = true;
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
                    hw.reservedWithPriority = action.getPriority();
                    hardwareReserves.put(hw, action);
                    return hw;
                }
            } catch (Exception e) {
                new Error(action, 203, "Could not reserve hardware element: " + className, null);
                return null;
            }
        }
    }

    public static HardwareElement ReserveHardwareForRoadRunner(String className) {
        synchronized (actionLock) {
            try {
                HardwareElement hw = getElementByClassName(className);
                if (hw == null || !hw.isInitialized || hw.isBroken) {
                    new Error(203, "Could not reserve hardware element: " + className,
                            ErrorTypes.ROAD_RUNNER_ERROR, null);
                    return null;
                }

                synchronized (hw) {
                    if (hw.isReserved) {
                        ActionElement actionToRelease = hardwareReserves.get(hw);
                        if (actionToRelease != null) {
                            actionToRelease.isStoppingDueToPriority = true;
                            StopAction(actionToRelease);
                        } else {
                            new Error(203, "Could not reserve hardware element: " + className,
                                    ErrorTypes.ROAD_RUNNER_ERROR, null);
                            return null;
                        }
                    }
                    hw.isReserved = true;
                    hw.reservedWithPriority = 999999999;
                    return hw;
                }
            } catch (Exception e) {
                new Error(203, "Could not reserve hardware element: " + className,
                        ErrorTypes.ROAD_RUNNER_ERROR, null);
                return null;
            }
        }
    }

    public static void onOpModeStart() {
        synchronized (actionLock) {
            opModeActive = true;
            for (HardwareElement hw : hardwareElements) {
                try {
                    if (hw.isBroken) continue;
                    Method updateMethod = hw.getClass().getMethod("update");
                    if (updateMethod.getDeclaringClass() != HardwareElement.class) {
                        hw.updateThread = new Thread(() -> {
                            while (opModeActive && !Thread.currentThread().isInterrupted()) {
                                try {
                                    hw.update();
                                    Thread.sleep(2);
                                } catch (InterruptedException e) {
                                    return;
                                } catch (Exception e) {
                                    new Error(hw, 102, "An error in hardware update thread", e);
                                    return;
                                }
                            }
                        });
                        hw.updateThread.start();
                    }
                } catch (NoSuchMethodException ignored) {
                    // Method not found, do nothing
                }
            }
        }
    }

    public static void onOpModeStop() {
        synchronized (actionLock) {
            opModeActive = false;

            List<ActionElement> actionsCopy = new ArrayList<>(runningActionElements);
            for (ActionElement action : actionsCopy) {
                StopAction(action);
            }
            runningActionElements.clear();

            for (HardwareElement hw : hardwareElements) {
                if (hw.updateThread != null) {
                    hw.updateThread.interrupt();
                }
                hw.stop();
            }
        }
    }
}