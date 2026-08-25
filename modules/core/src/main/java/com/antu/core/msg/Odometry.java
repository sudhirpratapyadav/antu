package com.antu.core.msg;

import com.antu.core.geometry.Pose2;
import com.antu.core.geometry.Twist2;

/**
 * Where the base believes it is, and how fast it is going. Immutable.
 *
 * <p>Odometry drifts. It is continuous and smooth, which makes it the right input
 * to a controller, and wrong as an answer to "where am I in the building". The
 * map-frame pose is a separate topic produced by localisation.
 */
public final class Odometry {

    /** Pose in the odometry frame, metres and radians. */
    public final Pose2 pose;
    /** Velocity in the robot's own frame. */
    public final Twist2 velocity;

    public Odometry(Pose2 pose, Twist2 velocity) {
        this.pose = pose;
        this.velocity = velocity;
    }

    @Override public String toString() {
        return "Odometry{" + pose + " " + velocity + "}";
    }
}
