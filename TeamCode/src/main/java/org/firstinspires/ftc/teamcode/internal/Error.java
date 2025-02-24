package org.firstinspires.ftc.teamcode.internal;

import android.util.Log;

public class Error {
    public int code;
    public String message;
    public ErrorTypes type;
    public Exception exception;

    // Type specifics
    public HardwareElement hw = null;
    public ActionElement action = null;

    private static final String LOG_TAG = "FTCCode"; // Constant log tag

    public Error (HardwareElement hw, int code, String message, Exception e) {
        hw.isBroken = true;
        this.message = message;
        this.code = code;
        this.type = ErrorTypes.HARDWARE_ERROR;
        this.exception = e;
        this.hw = hw;
        Log.e(LOG_TAG, "Hardware Error [" + code + "]: " + message, e);

        if (TelemetryManager.instance != null) {
            TelemetryManager.instance.AddError(this);
            HardwareManager.GracefullyFailHardware(hw);
        }
    }

    public Error (ActionElement action, int code, String message, Exception e) {
        this.message = message;
        this.code = code;
        this.type = ErrorTypes.ACTION_ERROR;
        this.exception = e;
        this.action = action;
        if (TelemetryManager.instance != null) {
            TelemetryManager.instance.AddError(this);
            if (code != 205) {
                if (action != null) {
                    action.isStoppingDueToError = true;
                    HardwareManager.StopAction(action);
                }
            }
            Log.e(LOG_TAG, "Action Error [" + code + "]: " + message, e);
        }
    }

    public Error (int code, String message, ErrorTypes type, Exception e) {
        this.message = message;
        this.code = code;
        this.type = type;
        this.exception = e;
        if (TelemetryManager.instance != null) {
            TelemetryManager.instance.AddError(this);
            Log.e(LOG_TAG, type + " [" + code + "]: " + message, e);
        }
    }
}