package org.firstinspires.ftc.teamcode.internal;

public abstract class ActionElement {
    public abstract void run() throws InterruptedException;

    public Thread runThread = null;
}
