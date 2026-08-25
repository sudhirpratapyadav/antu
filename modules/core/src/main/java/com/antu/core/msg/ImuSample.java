package com.antu.core.msg;

import com.antu.core.geometry.Vec3;

/**
 * One inertial reading. Immutable.
 *
 * <p>There are two IMUs on this robot: the phone's, and the gyro inside the
 * Pioneer's controller. They sit on different bodies with different mounting and
 * different noise, so each publishes on its own topic and fusion is a deliberate
 * step rather than an accident of whoever wrote last.
 *
 * <p>Axes follow the robot convention, not Android's screen convention: x
 * forward, y left, z up. Drivers rotate into it.
 */
public final class ImuSample {

    /** Angular rate, rad/s, about x, y, z. */
    public final Vec3 angularVelocity;
    /** Proper acceleration, m/s^2, gravity included unless the driver says otherwise. */
    public final Vec3 linearAcceleration;
    /** Heading about z, radians, or {@link Double#NaN} when the source has none. */
    public final double heading;

    public ImuSample(Vec3 angularVelocity, Vec3 linearAcceleration, double heading) {
        this.angularVelocity = angularVelocity;
        this.linearAcceleration = linearAcceleration;
        this.heading = heading;
    }

    /** True when this source reported an absolute heading. */
    public boolean hasHeading() {
        return !Double.isNaN(heading);
    }

    /** Yaw rate, rad/s — the only component a planar robot usually needs. */
    public double yawRate() {
        return angularVelocity.z;
    }

    @Override public String toString() {
        return "Imu{w=" + angularVelocity + " a=" + linearAcceleration
                + (hasHeading() ? " heading=" + heading : "") + "}";
    }
}
