package com.antu.core.geometry;

/**
 * A rotation in three dimensions. Immutable, and kept normalised.
 *
 * <p>Quaternions rather than Euler angles because the robot's pose now comes from
 * a visual-inertial tracker that reports full orientation, and Euler angles have
 * a singularity — a phone tilted to look at the floor is close enough to it that
 * yaw and roll start trading places. They also compose without the ordering
 * arguments that make Euler code so easy to get subtly wrong.
 *
 * <p>Component order is (x, y, z, w), matching ARCore and most graphics APIs.
 * Some maths libraries put w first; mixing the two silently produces a rotation
 * that is almost right, which is the worst kind of wrong.
 */
public final class Quat {

    /** No rotation. */
    public static final Quat IDENTITY = new Quat(0, 0, 0, 1);

    public final double x;
    public final double y;
    public final double z;
    public final double w;

    /** Normalises on construction, so every instance is a valid rotation. */
    public Quat(double x, double y, double z, double w) {
        double n = Math.sqrt(x * x + y * y + z * z + w * w);
        if (n < 1e-12) {
            // Degenerate input is a bug upstream, but returning identity beats
            // propagating NaN through every pose that follows.
            this.x = 0;
            this.y = 0;
            this.z = 0;
            this.w = 1;
        } else {
            this.x = x / n;
            this.y = y / n;
            this.z = z / n;
            this.w = w / n;
        }
    }

    /** A rotation of {@code radians} about {@code axis}, which need not be unit. */
    public static Quat fromAxisAngle(Vec3 axis, double radians) {
        double len = axis.length();
        if (len < 1e-12) {
            return IDENTITY;
        }
        double half = radians / 2;
        double s = Math.sin(half) / len;
        return new Quat(axis.x * s, axis.y * s, axis.z * s, Math.cos(half));
    }

    /** A rotation about the vertical, for a robot that stays on the floor. */
    public static Quat fromYaw(double radians) {
        return new Quat(0, 0, Math.sin(radians / 2), Math.cos(radians / 2));
    }

    /** Combined rotation: {@code this} applied after {@code other}. */
    public Quat times(Quat o) {
        return new Quat(
                w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x,
                w * o.z + x * o.y - y * o.x + z * o.w,
                w * o.w - x * o.x - y * o.y - z * o.z);
    }

    /** The rotation that undoes this one. Unit quaternions invert by conjugating. */
    public Quat inverse() {
        return new Quat(-x, -y, -z, w);
    }

    /** Applies this rotation to a vector. */
    public Vec3 rotate(Vec3 v) {
        // v + 2 * cross(q.xyz, cross(q.xyz, v) + w * v), which avoids building a
        // matrix for a single vector.
        double cx = y * v.z - z * v.y + w * v.x;
        double cy = z * v.x - x * v.z + w * v.y;
        double cz = x * v.y - y * v.x + w * v.z;
        return new Vec3(
                v.x + 2 * (y * cz - z * cy),
                v.y + 2 * (z * cx - x * cz),
                v.z + 2 * (x * cy - y * cx));
    }

    /**
     * Rotation about the z axis, radians.
     *
     * <p>The heading of a robot on a level floor. Meaningless when the body is
     * pitched near vertical, which is the Euler singularity this type exists to
     * avoid — so use it where the assumption holds and not elsewhere.
     */
    public double yaw() {
        return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    }

    /** Rotation about the y axis, radians. ARCore's world frame is y-up. */
    public double yawAboutY() {
        return Math.atan2(2 * (w * y + z * x), 1 - 2 * (y * y + x * x));
    }

    /** Angle of the shortest rotation between two orientations, radians. */
    public double angleTo(Quat o) {
        double dot = Math.abs(x * o.x + y * o.y + z * o.z + w * o.w);
        return 2 * Math.acos(Math.min(1.0, dot));
    }

    @Override public boolean equals(Object o) {
        if (!(o instanceof Quat)) {
            return false;
        }
        Quat q = (Quat) o;
        return Double.compare(x, q.x) == 0 && Double.compare(y, q.y) == 0
                && Double.compare(z, q.z) == 0 && Double.compare(w, q.w) == 0;
    }

    @Override public int hashCode() {
        return ((Double.hashCode(x) * 31 + Double.hashCode(y)) * 31
                + Double.hashCode(z)) * 31 + Double.hashCode(w);
    }

    @Override public String toString() {
        return String.format("(%.4f, %.4f, %.4f, %.4f)", x, y, z, w);
    }
}
