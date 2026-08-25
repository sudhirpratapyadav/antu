package com.antu.core.geometry;

/** Angle helpers. Radians throughout. */
public final class Angles {

    private Angles() { }

    public static final double TWO_PI = 2 * Math.PI;

    /**
     * Wraps to (-pi, pi].
     *
     * <p>Every angle stored in a {@link Pose2} passes through here. Skipping it is
     * how a robot asked to turn 1 degree past pi decides to spin most of the way
     * round instead.
     */
    public static double normalise(double radians) {
        double a = radians % TWO_PI;
        if (a > Math.PI) {
            a -= TWO_PI;
        } else if (a <= -Math.PI) {
            a += TWO_PI;
        }
        return a;
    }

    /** Shortest signed rotation from {@code from} to {@code to}, in (-pi, pi]. */
    public static double difference(double from, double to) {
        return normalise(to - from);
    }

    /** Magnitude of the shortest rotation between two angles, in [0, pi]. */
    public static double distance(double a, double b) {
        return Math.abs(difference(a, b));
    }

    /**
     * Interpolates the short way round.
     *
     * @param t 0 gives {@code from}, 1 gives {@code to}
     */
    public static double lerp(double from, double to, double t) {
        return normalise(from + difference(from, to) * t);
    }

    public static double toDegrees(double radians) {
        return radians * 180.0 / Math.PI;
    }

    public static double toRadians(double degrees) {
        return degrees * Math.PI / 180.0;
    }
}
