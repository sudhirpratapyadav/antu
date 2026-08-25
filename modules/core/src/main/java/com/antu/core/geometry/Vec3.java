package com.antu.core.geometry;

/** A 3D vector. Units depend on use: m/s^2 for acceleration, rad/s for rates. */
public final class Vec3 {

    public static final Vec3 ZERO = new Vec3(0, 0, 0);

    public final double x;
    public final double y;
    public final double z;

    public Vec3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vec3 plus(Vec3 o) {
        return new Vec3(x + o.x, y + o.y, z + o.z);
    }

    public Vec3 minus(Vec3 o) {
        return new Vec3(x - o.x, y - o.y, z - o.z);
    }

    public Vec3 scaled(double k) {
        return new Vec3(x * k, y * k, z * k);
    }

    public double length() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    public double dot(Vec3 o) {
        return x * o.x + y * o.y + z * o.z;
    }

    @Override public boolean equals(Object o) {
        if (!(o instanceof Vec3)) {
            return false;
        }
        Vec3 v = (Vec3) o;
        return Double.compare(x, v.x) == 0
                && Double.compare(y, v.y) == 0
                && Double.compare(z, v.z) == 0;
    }

    @Override public int hashCode() {
        return (Double.hashCode(x) * 31 + Double.hashCode(y)) * 31 + Double.hashCode(z);
    }

    @Override public String toString() {
        return String.format("(%.3f, %.3f, %.3f)", x, y, z);
    }
}
