package com.antu.core.geometry;

/**
 * A velocity in the robot's own frame: m/s forward and sideways, rad/s about z.
 * Immutable.
 *
 * <p>{@link #linearY} is always zero on a differential drive like a Pioneer, and
 * exists so a holonomic base does not force a second type on everything upstream.
 * Drivers that cannot strafe should say so rather than ignoring it silently.
 */
public final class Twist2 {

    public static final Twist2 ZERO = new Twist2(0, 0, 0);

    /** Forward, m/s. Positive is ahead. */
    public final double linearX;
    /** Sideways, m/s. Positive is to the left. Zero on a differential drive. */
    public final double linearY;
    /** Rotation, rad/s. Positive is counter-clockwise. */
    public final double angular;

    public Twist2(double linearX, double linearY, double angular) {
        this.linearX = linearX;
        this.linearY = linearY;
        this.angular = angular;
    }

    /** The common case: drive forward and turn. */
    public static Twist2 of(double linearX, double angular) {
        return new Twist2(linearX, 0, angular);
    }

    public static Twist2 forward(double metresPerSecond) {
        return new Twist2(metresPerSecond, 0, 0);
    }

    public static Twist2 turn(double radiansPerSecond) {
        return new Twist2(0, 0, radiansPerSecond);
    }

    public boolean isZero() {
        return linearX == 0 && linearY == 0 && angular == 0;
    }

    public double linearSpeed() {
        return Math.hypot(linearX, linearY);
    }

    public Twist2 scaled(double k) {
        return new Twist2(linearX * k, linearY * k, angular * k);
    }

    /**
     * Clamped to the given limits, preserving the ratio of the two.
     *
     * <p>Scaling both rather than clipping each independently matters: clipping
     * separately changes the arc the robot follows, so a path that was planned as
     * a gentle curve becomes a tighter one near the speed limit.
     */
    public Twist2 limited(double maxLinear, double maxAngular) {
        double scale = 1.0;
        double speed = linearSpeed();
        if (speed > maxLinear && speed > 0) {
            scale = Math.min(scale, maxLinear / speed);
        }
        if (Math.abs(angular) > maxAngular && angular != 0) {
            scale = Math.min(scale, maxAngular / Math.abs(angular));
        }
        return scale == 1.0 ? this : scaled(scale);
    }

    /** Wheel speeds for a differential drive, m/s, left then right. */
    public double[] toWheelSpeeds(double wheelbase) {
        double half = angular * wheelbase / 2.0;
        return new double[] {linearX - half, linearX + half};
    }

    /** The twist implied by differential wheel speeds in m/s. */
    public static Twist2 fromWheelSpeeds(double left, double right, double wheelbase) {
        return Twist2.of((left + right) / 2.0, (right - left) / wheelbase);
    }

    @Override public boolean equals(Object o) {
        if (!(o instanceof Twist2)) {
            return false;
        }
        Twist2 t = (Twist2) o;
        return Double.compare(linearX, t.linearX) == 0
                && Double.compare(linearY, t.linearY) == 0
                && Double.compare(angular, t.angular) == 0;
    }

    @Override public int hashCode() {
        return (Double.hashCode(linearX) * 31 + Double.hashCode(linearY)) * 31
                + Double.hashCode(angular);
    }

    @Override public String toString() {
        return String.format("(%.3f m/s, %.3f m/s, %.1f deg/s)",
                linearX, linearY, Angles.toDegrees(angular));
    }
}
