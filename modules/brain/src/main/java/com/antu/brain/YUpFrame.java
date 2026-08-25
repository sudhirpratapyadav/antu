package com.antu.brain;

import com.antu.core.geometry.Pose2;
import com.antu.core.geometry.Quat;
import com.antu.core.geometry.Vec2;

/**
 * Converting between a y-up tracker frame and the robot's z-up planar one.
 *
 * <p>ARCore's world has y pointing up; the robot's has z up and works on the
 * floor. Seen from above, the two disagree about which way angles run, so the 2D
 * mapping below is a reflection even though the underlying 3D relationship is a
 * proper rotation. That is counterintuitive enough to be worth one shared
 * implementation rather than several: getting it wrong makes the robot turn the
 * right amount in the wrong direction, which looks like a sign error somewhere
 * else entirely.
 *
 * <p>Confirmed by driving the robot rather than by reasoning about it. A forward
 * run put the travel direction within 0.8 degrees of the reported heading, where
 * a sign error would show as tens or hundreds.
 */
public final class YUpFrame {

    private YUpFrame() { }

    /** A tracker-frame position, flattened onto the robot's floor plane. */
    public static Vec2 toPlanar(double x, double z) {
        return new Vec2(-z, -x);
    }

    /** The height of a tracker-frame point. The tracker's y axis is up. */
    public static double heightOf(double y) {
        return y;
    }

    /** A tracker-frame orientation, as a heading on the floor. */
    public static double headingOf(Quat rotation) {
        return rotation.yawAboutY();
    }

    /** A full tracker pose, flattened. */
    public static Pose2 toPlanar(com.antu.core.geometry.Pose3 pose) {
        Vec2 p = toPlanar(pose.position.x, pose.position.z);
        return new Pose2(p.x, p.y, headingOf(pose.rotation));
    }
}
