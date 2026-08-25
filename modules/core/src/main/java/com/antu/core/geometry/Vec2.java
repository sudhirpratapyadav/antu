package com.antu.core.geometry;

/** A point or vector in the plane, in metres. Immutable. */
public final class Vec2 {

    public static final Vec2 ZERO = new Vec2(0, 0);

    public final double x;
    public final double y;

    public Vec2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /** A unit vector at {@code radians} from the x axis. */
    public static Vec2 fromAngle(double radians) {
        return new Vec2(Math.cos(radians), Math.sin(radians));
    }

    public Vec2 plus(Vec2 o) {
        return new Vec2(x + o.x, y + o.y);
    }

    public Vec2 minus(Vec2 o) {
        return new Vec2(x - o.x, y - o.y);
    }

    public Vec2 scaled(double k) {
        return new Vec2(x * k, y * k);
    }

    /** Rotated about the origin by {@code radians}, counter-clockwise. */
    public Vec2 rotated(double radians) {
        double c = Math.cos(radians);
        double s = Math.sin(radians);
        return new Vec2(x * c - y * s, x * s + y * c);
    }

    public double length() {
        return Math.hypot(x, y);
    }

    /** Squared length, for comparisons that do not need the square root. */
    public double lengthSquared() {
        return x * x + y * y;
    }

    public double distanceTo(Vec2 o) {
        return Math.hypot(x - o.x, y - o.y);
    }

    /** Direction from the origin, radians. */
    public double angle() {
        return Math.atan2(y, x);
    }

    /** This vector scaled to unit length, or {@link #ZERO} if it has none. */
    public Vec2 normalised() {
        double len = length();
        return len < 1e-12 ? ZERO : scaled(1.0 / len);
    }

    public double dot(Vec2 o) {
        return x * o.x + y * o.y;
    }

    /** The z component of the 3D cross product; sign gives the turn direction. */
    public double cross(Vec2 o) {
        return x * o.y - y * o.x;
    }

    @Override public boolean equals(Object o) {
        if (!(o instanceof Vec2)) {
            return false;
        }
        Vec2 v = (Vec2) o;
        return Double.compare(x, v.x) == 0 && Double.compare(y, v.y) == 0;
    }

    @Override public int hashCode() {
        return Double.hashCode(x) * 31 + Double.hashCode(y);
    }

    @Override public String toString() {
        return String.format("(%.3f, %.3f)", x, y);
    }
}
