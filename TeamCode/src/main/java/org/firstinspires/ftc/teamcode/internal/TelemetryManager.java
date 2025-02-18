package org.firstinspires.ftc.teamcode.internal;

import android.content.Context;
import android.content.res.Resources;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.R;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Random;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * This class handles telemetry and error management for the robot.
 */
public class TelemetryManager {
    public static TelemetryManager instance = null;

    final private Telemetry telemetry;
    final private String randomMessage;
    private List<Error> errors = new CopyOnWriteArrayList<>();
    private List<FunctionToLog> functionsToLog = new ArrayList<>();
    private HashMap<String, String> messagesToLog = new HashMap<>();

    private Thread thread = null;

    public TelemetryManager(Telemetry telemetry, Context context) {
        if (instance != null) {

        }
        instance = this;
        this.telemetry = telemetry;
        Resources res = context.getResources();
        String[] messages = res.getStringArray(R.array.telemetry_messages);
        Random random = new Random();
        int index = random.nextInt(messages.length);
        randomMessage = messages[index];
    }

    private void TelemetryLoop() {
        while (true) {
            telemetry.addLine("Jorje Himothy");
            telemetry.addLine("Build Date: " + BuildConfig.BUILD_DATE);
            telemetry.addLine(randomMessage);
            telemetry.addLine();

            //region Error Handling
            if (!errors.isEmpty()) {
                telemetry.addLine("An error has occurred!");
                telemetry.addLine("This will be logged to the disk.");
                telemetry.addLine();

                for (Error err : errors) {
                    switch (err.type) {
                        case HARDWARE_ERROR:
                            telemetry.addLine("A hardware exception has occurred.");
                            telemetry.addLine("The hardware element that caused this error is now disabled.");
                            if (err.hw != null) {
                                telemetry.addLine("Hardware Element: " + err.hw.getClass().getSimpleName());
                            } else {
                                telemetry.addLine("Hardware Element: Unknown");
                            }
                            break;
                        case ACTION_ERROR:
                            telemetry.addLine("An exception with a running action has occurred.");
                            telemetry.addLine("The action has been stopped.");
                            break;
                        case ROAD_RUNNER_ERROR:
                            telemetry.addLine("An exception with Road Runner has occurred.");
                            break;
                        case GENERIC_ERROR:
                            telemetry.addLine("A generic exception has occurred.");
                            break;
                        case UNKNOWN_ERROR:
                            telemetry.addLine("An unknown exception has occurred.");
                            break;
                    }
                    telemetry.addLine("Error code: " + err.code);
                    telemetry.addLine(err.message);
                    if (err.exception != null) {
                        telemetry.addLine();
                        telemetry.addLine("Exception Details: ");
                        telemetry.addLine(err.exception.getMessage());
                    }
                    telemetry.addLine();
                }
            }
            //endregion

            //region Logging
            try {
                synchronized (functionsToLog) {
                    for (FunctionToLog variable : functionsToLog) {
                        try {
                            Object dataToLog = variable.value.getValue();
                            telemetry.addData(variable.name, dataToLog);
                        } catch (Exception e) {
                            RemoveFunctionFromLogging(variable.name);
                        }
                    }
                }
                synchronized (messagesToLog) {
                    for (String key : messagesToLog.keySet()) {
                        telemetry.addData(key, messagesToLog.get(key));
                    }
                }
            } catch (Exception e) {
                //blank
            }
            //endregion

            telemetry.update();
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                return;
            }
            if (Thread.currentThread().isInterrupted()) {
                return;
            }
            if (HardwareManager.opMode == null || HardwareManager.opMode.isStopped) {
                // The opmode has stopped, but we are still running, lets terminate everything to avoid leaking of resources.
                HardwareManager.onOpModeStop();
                instance = null;
                return;
            }
        }
    }

    public void StartTelemetryLoop() {
        thread = new Thread(this::TelemetryLoop);
        thread.start();
    }

    public void StopTelemetryLoop() {
        try {
            thread.interrupt();
        } catch (NullPointerException e) {
            throw new RuntimeException(e);
        }

    }

    public void AddError(Error err) {
        errors.add(err);
    }

    public void ClearError() {
        errors.remove(0);
    }

    public void ClearAllErrors() {
        errors.clear();
    }

    public void AddFunctionToLogging(String name,LoggingFunction function) {
        functionsToLog.add(new FunctionToLog(name, function));

    }

    public void ModifyMessagesLogged(String name, String message) {
        messagesToLog.put(name, message);
    }
    public void RemoveMessageLogged(String name) {
        messagesToLog.remove(name);
    }

    public void RemoveFunctionFromLogging(String name) {
        for (FunctionToLog variable : functionsToLog) {
            if (variable.name.equals(name)) {
                functionsToLog.remove(variable);
                return;
            }
        }

    }

    public void ClearAllFunctionsFromLogging() {
        functionsToLog.clear();
    }
}