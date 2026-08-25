package com.antu.core.msg;

import java.util.Arrays;

/**
 * A set of range readings taken from fixed sensors, in metres. Immutable.
 *
 * <p>Covers a sonar ring now and a lidar later: both are "ranges at known
 * bearings", and a costmap does not care which produced them.
 *
 * <p>A reading of {@link #NO_RETURN} means the sensor heard nothing, which is not
 * the same as "clear to the maximum range" — an angled surface reflects away and
 * reads as empty. Treating no-return as free space is how a robot drives into a
 * wall it could technically see.
 */
public final class RangeScan {

    /** No echo came back. Not a distance. */
    public static final double NO_RETURN = Double.POSITIVE_INFINITY;

    /** Bearings of each sensor in the robot frame, radians, counter-clockwise. */
    private final double[] bearings;
    /** Measured range per sensor, metres, or {@link #NO_RETURN}. */
    private final double[] ranges;
    /** Beyond this the sensor is not trusted, metres. */
    public final double maxRange;
    /** Below this a reading is noise, metres. */
    public final double minRange;

    public RangeScan(double[] bearings, double[] ranges, double minRange, double maxRange) {
        if (bearings.length != ranges.length) {
            throw new IllegalArgumentException(
                    "bearings and ranges differ: " + bearings.length + " vs " + ranges.length);
        }
        // Defensive copies: the arrays are handed out to every subscriber and the
        // bus contract is that payloads never change after publishing.
        this.bearings = bearings.clone();
        this.ranges = ranges.clone();
        this.minRange = minRange;
        this.maxRange = maxRange;
    }

    public int size() {
        return ranges.length;
    }

    public double bearing(int i) {
        return bearings[i];
    }

    public double range(int i) {
        return ranges[i];
    }

    /** True when reading {@code i} is a real measurement within the trusted band. */
    public boolean isValid(int i) {
        double r = ranges[i];
        return r >= minRange && r <= maxRange;
    }

    /** Nearest valid reading in metres, or {@link #NO_RETURN} if there is none. */
    public double closest() {
        double best = NO_RETURN;
        for (int i = 0; i < ranges.length; i++) {
            if (isValid(i) && ranges[i] < best) {
                best = ranges[i];
            }
        }
        return best;
    }

    /** Nearest valid reading whose bearing is within {@code halfWidth} of ahead. */
    public double closestAhead(double halfWidth) {
        double best = NO_RETURN;
        for (int i = 0; i < ranges.length; i++) {
            if (isValid(i) && Math.abs(bearings[i]) <= halfWidth && ranges[i] < best) {
                best = ranges[i];
            }
        }
        return best;
    }

    @Override public String toString() {
        return "RangeScan{" + ranges.length + " beams, closest="
                + (closest() == NO_RETURN ? "none" : String.format("%.2fm", closest())) + "}";
    }

    /** A copy of the ranges, for callers that need the raw array. */
    public double[] ranges() {
        return ranges.clone();
    }

    /** A copy of the bearings. */
    public double[] bearings() {
        return bearings.clone();
    }

    @Override public boolean equals(Object o) {
        if (!(o instanceof RangeScan)) {
            return false;
        }
        RangeScan s = (RangeScan) o;
        return Arrays.equals(ranges, s.ranges) && Arrays.equals(bearings, s.bearings);
    }

    @Override public int hashCode() {
        return Arrays.hashCode(ranges) * 31 + Arrays.hashCode(bearings);
    }
}
