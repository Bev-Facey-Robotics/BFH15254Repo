package org.firstinspires.ftc.teamcode.internal;

import org.firstinspires.ftc.teamcode.hardware.HardwareElement;

public class Error {
    public int code;
    public String message;
    public ErrorTypes type;
    public Exception exception;

    // Type specifics
    public HardwareElement hw = null;

    public Error (HardwareElement hw, int code, String message, Exception e) {
        hw.isBroken = true;
        this.message = message;
        this.code = code;
        this.type = ErrorTypes.HARDWARE_ERROR;
        this.exception = e;
        this.hw = hw;
        TelemetryManager.instance.AddError(this);
    }

    public Error (int code, String message, ErrorTypes type, Exception e) {
        this.message = message;
        this.code = code;
        this.type = type;
        this.exception = e;
        TelemetryManager.instance.AddError(this);
    }
}