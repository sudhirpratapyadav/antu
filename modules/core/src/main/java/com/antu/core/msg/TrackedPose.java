package com.antu.core.msg;

import com.antu.core.geometry.Pose3;

/**
 * Where a visual-inertial tracker believes the camera is, and whether to believe
 * it. Immutable.
 *
 * <p>The tracking state is not an afterthought. A VIO system does not fail by
 * reporting nonsense — it reports a pose that looks entirely reasonable and is
 * quietly wrong, having lost its features somewhere back down the corridor. Any
 * consumer that uses {@link #pose} without checking {@link #isTracking} will one
 * day drive confidently into a wall on stale information.
 *
 * <p>The pose is in the tracker's own world frame: the origin is wherever the
 * session started, and ARCore's axes are y-up, which is not the robot's
 * convention. Relating it to the wheels is a calibration step, deliberately kept
 * separate from this so the raw estimate stays inspectable.
 */
public final class TrackedPose {

    /** What the tracker thinks of its own estimate. */
    public enum State {
        /** The pose is current and usable. */
        TRACKING,
        /** Temporarily lost; the pose is the last good one and is going stale. */
        PAUSED,
        /** Not running. */
        STOPPED
    }

    /** Camera pose in the tracker's world frame, metres. */
    public final Pose3 pose;
    public final State state;
    /**
     * Why tracking is not healthy, as the tracker describes it — insufficient
     * features, excessive motion, bad lighting. Empty while tracking.
     */
    public final String reason;
    /** Frames processed since the session started. */
    public final long frames;

    public TrackedPose(Pose3 pose, State state, String reason, long frames) {
        this.pose = pose;
        this.state = state;
        this.reason = reason == null ? "" : reason;
        this.frames = frames;
    }

    /** True when the pose may be used. Check this before trusting the pose. */
    public boolean isTracking() {
        return state == State.TRACKING;
    }

    @Override public String toString() {
        return "TrackedPose{" + state + (reason.isEmpty() ? "" : " (" + reason + ")")
                + " " + pose + "}";
    }
}
