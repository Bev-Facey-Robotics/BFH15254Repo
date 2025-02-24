package org.firstinspires.ftc.teamcode.internal;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public abstract class ActionElement {
    public abstract void run() throws InterruptedException, NullPointerException ;

//    @Override
//    public boolean run(@NonNull TelemetryPacket packet) throws NullPointerException {
//        try {
//            run();
//        } catch (InterruptedException e) {
//
//        } catch (NullPointerException e) {
//            if (isAutoRestart()) {
//                runThread.interrupt();
//            }
//            return false;
//        }
//        return true;
//    }

    public Thread runThread = null;

    // configuration
    // In java, you cannot override variables, so here is the solution to that problem.

    /**
     * if true, when the action is interrupted by another, more important action, it will be restarted when the other action is finished.
     * @return the boolean value of autoRestart
     */
    public abstract boolean isAutoRestart();

    public boolean isStoppingDueToPriority = false;
    public boolean isStoppingDueToError = false;
    /**
     * the higher the number, the more important the action is. 0 is the lowest priority. A higher priority means that when the action is called, it will override any actions with a lower priority.
     * @return the priority of the action
     */
    public abstract int getPriority();
}
