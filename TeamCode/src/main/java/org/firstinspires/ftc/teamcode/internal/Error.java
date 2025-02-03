package org.firstinspires.ftc.teamcode.internal;

public class Error {
    public int code;
    public String message;
    public ErrorTypes type;
    public Exception exception;

    // Type specifics
    public HardwareElement hw = null;
    public ActionElement action = null;

    public Error (HardwareElement hw, int code, String message, Exception e) {
        hw.isBroken = true;
        this.message = message;
        this.code = code;
        this.type = ErrorTypes.HARDWARE_ERROR;
        this.exception = e;
        this.hw = hw;
        TelemetryManager.instance.AddError(this);
        HardwareManager.GracefullyFailHardware(hw);
    }

    public Error (ActionElement action, int code, String message, Exception e) {
        this.message = message;
        this.code = code;
        this.type = ErrorTypes.HARDWARE_ERROR;
        this.exception = e;
        this.action = action;
        TelemetryManager.instance.AddError(this);
        HardwareManager.StopAction(action);
    }

    public Error (int code, String message, ErrorTypes type, Exception e) {
        this.message = message;
        this.code = code;
        this.type = type;
        this.exception = e;
        TelemetryManager.instance.AddError(this);
    }
}