package com.antu.core.geometry;

/**
 * A position and heading in the plane: metres and radians. Immutable.
 *
 * <p>A pose is meaningless without knowing which frame it is in. That is carried
 * by the topic it arrives on — {@code /odom} is the odometry frame, {@code /pose}
 * the map frame — until the transform tree lands and makes it explicit.
 */
public final class Pose2 {

    public static final Pose2 ORIGIN = new Pose2(0, 0, 0);

    public final double x;
    public final double y;
    /** Heading, radians, always normalised to (-pi, pi]. */
    public final double theta;

    public Pose2(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = Angles.normalise(theta);
    }

    public Pose2(Vec2 position, double theta) {
        this(position.x, position.y, theta);
    }

    public Vec2 position() {
        return new Vec2(x, y);
    }

    /** Unit vector pointing the way this pose faces. */
    public Vec2 heading() {
        return Vec2.fromAngle(theta);
    }

    /**
     * Treats this pose as a transform and applies it to a point expressed in this
     * pose's frame, giving the point in the parent frame.
     */
    public Vec2 apply(Vec2 local) {
        return local.rotated(theta).plus(position());
    }

    /**
     * Composes: {@code this} then {@code relative}, where {@code relative} is
     * expressed in this pose's frame.
     */
    public Pose2 compose(Pose2 relative) {
        return new Pose2(apply(relative.position()), theta + relative.theta);
    }

    /** The transform that undoes this one. */
    public Pose2 inverse() {
        Vec2 p = position().scaled(-1).rotated(-theta);
        return new Pose2(p, -theta);
    }

    /**
     * This pose expressed in {@code reference}'s frame. The usual way to ask
     * "where is the goal, from where I am now".
     */
    public Pose2 relativeTo(Pose2 reference) {
        return reference.inverse().compose(this);
    }

    public double distanceTo(Pose2 other) {
        return Math.hypot(x - other.x, y - other.y);
    }

    /** Shortest signed rotation from this heading to {@code other}'s. */
    public double angleTo(Pose2 other) {
        return Angles.difference(theta, other.theta);
    }

    /**
     * Advances by a body-frame velocity held for {@code dt} seconds.
     *
     * <p>Uses the exact arc for a constant twist rather than the straight-line
     * approximation. At a slow tick rate on a turning robot the difference is
     * visible drift, and it costs one branch.
     */
    public Pose2 integrate(Twist2 twist, double dt) {
        double dTheta = twist.angular * dt;
        double dx = twist.linearX * dt;
        double dy = twist.linearY * dt;

        if (Math.abs(dTheta) < 1e-9) {
            return new Pose2(apply(new Vec2(dx, dy)), theta);
        }
        // Exact integration of a constant twist: the body follows a circular arc.
        double sin = Math.sin(dTheta);
        double cos = Math.cos(dTheta);
        double localX = (dx * sin + dy * (cos - 1)) / dTheta;
        double localY = (dx * (1 - cos) + dy * sin) / dTheta;
        return new Pose2(apply(new Vec2(localX, localY)), theta + dTheta);
    }

    @Override public boolean equals(Object o) {
        if (!(o instanceof Pose2)) {
            return false;
        }
        Pose2 p = (Pose2) o;
        return Double.compare(x, p.x) == 0
                && Double.compare(y, p.y) == 0
                && Double.compare(theta, p.theta) == 0;
    }

    @Override public int hashCode() {
        return (Double.hashCode(x) * 31 + Double.hashCode(y)) * 31 + Double.hashCode(theta);
    }

    @Override public String toString() {
        return String.format("(%.3fm, %.3fm, %.1f deg)", x, y, Angles.toDegrees(theta));
    }
}
