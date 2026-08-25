package com.antu.core.msg;

/**
 * What the drive base reports about itself. Immutable.
 *
 * <p>Separate from {@link Odometry} because it changes rarely and matters for
 * different reasons: a planner wants the pose, a supervisor wants to know the
 * e-stop is down and the battery is falling.
 */
public final class BaseStatus {

    /** Pack voltage, volts. Negative when the base does not report it. */
    public final double batteryVolts;
    /** True when the motors will act on velocity commands. */
    public final boolean motorsEnabled;
    /** True when an emergency stop is latched. */
    public final boolean emergencyStopped;
    /** True when a wheel is commanded but not turning. */
    public final boolean stalled;
    /** Model as the base identified itself, for logs. */
    public final String model;

    public BaseStatus(double batteryVolts, boolean motorsEnabled,
                      boolean emergencyStopped, boolean stalled, String model) {
        this.batteryVolts = batteryVolts;
        this.motorsEnabled = motorsEnabled;
        this.emergencyStopped = emergencyStopped;
        this.stalled = stalled;
        this.model = model;
    }

    /** True when the base is in a state that will accept motion commands. */
    public boolean canDrive() {
        return motorsEnabled && !emergencyStopped;
    }

    @Override public String toString() {
        return String.format("BaseStatus{%s %.1fV motors=%s estop=%s stall=%s}",
                model, batteryVolts, motorsEnabled, emergencyStopped, stalled);
    }
}
