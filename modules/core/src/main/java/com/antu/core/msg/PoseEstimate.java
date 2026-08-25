package com.antu.core.msg;

import com.antu.core.geometry.Pose2;

/**
 * The robot's best guess at where it is, and how much to trust it. Immutable.
 *
 * <p>Distinct from {@link Odometry}, which is what the wheels say. This is the
 * fused answer, and it carries its provenance because the two differ in a way
 * that matters: while a visual tracker holds, the pose is anchored to the room
 * and does not drift; once it drops, the same field is dead reckoning and the
 * error grows with every metre travelled.
 *
 * <p>A consumer that ignores {@link #source} will eventually plan a long route on
 * a pose that stopped being anchored several corridors ago.
 */
public final class PoseEstimate {

    /** Where the estimate came from. */
    public enum Source {
        /** Anchored by the visual tracker. Does not drift. */
        TRACKED,
        /** The tracker is down; riding wheel odometry from the last good fix. */
        DEAD_RECKONED
    }

    /** Best estimate in the fused world frame, metres and radians. */
    public final Pose2 pose;
    public final Source source;
    /** Seconds since the pose was last anchored by the tracker. 0 while tracking. */
    public final double secondsSinceFix;
    /**
     * Distance the estimate jumped when the tracker was last re-acquired, metres.
     *
     * <p>Published rather than smoothed away. A tracker coming back after a long
     * blind stretch can correct by a lot, and a planner deserves to know its world
     * just moved instead of quietly following a path that no longer fits.
     */
    public final double lastCorrection;

    public PoseEstimate(Pose2 pose, Source source, double secondsSinceFix,
                        double lastCorrection) {
        this.pose = pose;
        this.source = source;
        this.secondsSinceFix = secondsSinceFix;
        this.lastCorrection = lastCorrection;
    }

    /** True while the tracker is anchoring the estimate. */
    public boolean isAnchored() {
        return source == Source.TRACKED;
    }

    @Override public String toString() {
        return String.format("PoseEstimate{%s %s%s}", pose, source,
                source == Source.TRACKED ? ""
                        : String.format(" for %.1fs", secondsSinceFix));
    }
}
