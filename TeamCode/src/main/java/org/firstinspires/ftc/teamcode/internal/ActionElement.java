package org.firstinspires.ftc.teamcode.internal;

public abstract class ActionElement {
    public abstract void run() throws InterruptedException;

    public Thread runThread = null;

    // configuration
    public final boolean autoRestart = false; // if true, when the action is interrupted by another, more important action, it will be restarted when the other action is finished.
    public final int priority = 1; // the higher the number, the more important the action is. 0 is the lowest priority. A higher priority means that when the action is called, it will override any actions with a lower priority.
}
