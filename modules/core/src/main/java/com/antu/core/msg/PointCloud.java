package com.antu.core.msg;

/**
 * A set of 3D points in the tracker's world frame. Immutable.
 *
 * <p>Stored as flat arrays rather than a list of point objects. A cloud is
 * thousands of points arriving many times a second, and an object per point
 * would spend more time in the allocator than in the arithmetic.
 *
 * <p>Confidence is not decoration. A visual tracker's feature points include
 * plenty it is unsure about — a reflection, a shadow edge, a point seen once
 * from one angle — and treating those as solid geometry fills a map with walls
 * that are not there. Filtering on confidence is the difference between a map
 * and a cloud of noise.
 */
public final class PointCloud {

    /** x, y, z per point, in the tracker's world frame, metres. */
    private final float[] xyz;
    /** Confidence per point, 0 to 1. */
    private final float[] confidence;
    /** Points actually present; the arrays may be larger. */
    public final int size;

    public PointCloud(float[] xyz, float[] confidence, int size) {
        if (xyz.length < size * 3) {
            throw new IllegalArgumentException(
                    "xyz holds " + xyz.length + " floats, too few for " + size + " points");
        }
        this.xyz = xyz;
        this.confidence = confidence;
        this.size = size;
    }

    /** An empty cloud, for when the tracker has nothing to offer. */
    public static PointCloud empty() {
        return new PointCloud(new float[0], new float[0], 0);
    }

    public double x(int i) {
        return xyz[i * 3];
    }

    /** Height. The tracker's world frame is y-up. */
    public double y(int i) {
        return xyz[i * 3 + 1];
    }

    public double z(int i) {
        return xyz[i * 3 + 2];
    }

    public double confidence(int i) {
        return i < confidence.length ? confidence[i] : 1.0;
    }

    /** How many points clear a confidence threshold. */
    public int countAbove(double threshold) {
        int n = 0;
        for (int i = 0; i < size; i++) {
            if (confidence(i) >= threshold) {
                n++;
            }
        }
        return n;
    }

    @Override public String toString() {
        return "PointCloud{" + size + " points}";
    }
}
