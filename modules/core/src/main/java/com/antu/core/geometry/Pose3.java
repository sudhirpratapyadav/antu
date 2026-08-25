package com.antu.core.geometry;

/**
 * A position and orientation in three dimensions. Immutable.
 *
 * <p>The robot drives on a floor, so most of the system works in {@link Pose2}.
 * This exists because the things that tell us where the robot <em>is</em> do not:
 * a visual-inertial tracker reports full six degrees of freedom, and the camera
 * is mounted at some height and tilt that only a 3D transform can express.
 *
 * <p>Also serves as a transform. A pose and a transform are the same arithmetic —
 * "where the camera is, in the world" and "how to take a point from the camera's
 * frame into the world" are one object.
 */
public final class Pose3 {

    public static final Pose3 IDENTITY = new Pose3(Vec3.ZERO, Quat.IDENTITY);

    public final Vec3 position;
    public final Quat rotation;

    public Pose3(Vec3 position, Quat rotation) {
        this.position = position;
        this.rotation = rotation;
    }

    /** A pose from ARCore's translation and (x, y, z, w) quaternion. */
    public static Pose3 of(double tx, double ty, double tz,
                           double qx, double qy, double qz, double qw) {
        return new Pose3(new Vec3(tx, ty, tz), new Quat(qx, qy, qz, qw));
    }

    /** Takes a point expressed in this frame into the parent frame. */
    public Vec3 apply(Vec3 local) {
        return rotation.rotate(local).plus(position);
    }

    /** Composes: this transform, then {@code relative} expressed in its frame. */
    public Pose3 compose(Pose3 relative) {
        return new Pose3(apply(relative.position), rotation.times(relative.rotation));
    }

    /** The transform that undoes this one. */
    public Pose3 inverse() {
        Quat r = rotation.inverse();
        return new Pose3(r.rotate(position).scaled(-1), r);
    }

    /** This pose expressed in {@code reference}'s frame. */
    public Pose3 relativeTo(Pose3 reference) {
        return reference.inverse().compose(this);
    }

    public double distanceTo(Pose3 other) {
        return position.minus(other.position).length();
    }

    /**
     * Flattened onto the floor: x and y from the position, heading from the
     * rotation about z.
     *
     * <p>Correct only when this pose is already expressed in a frame whose z axis
     * is up. ARCore's world frame is <em>y</em>-up, so a tracker pose has to be
     * rotated into the robot's convention before this means anything — which is
     * exactly what calibration produces.
     */
    public Pose2 flatten() {
        return new Pose2(position.x, position.y, rotation.yaw());
    }

    @Override public boolean equals(Object o) {
        if (!(o instanceof Pose3)) {
            return false;
        }
        Pose3 p = (Pose3) o;
        return position.equals(p.position) && rotation.equals(p.rotation);
    }

    @Override public int hashCode() {
        return position.hashCode() * 31 + rotation.hashCode();
    }

    @Override public String toString() {
        return "Pose3{" + position + " " + rotation + "}";
    }
}
