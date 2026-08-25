package com.antu.brain;

import com.antu.core.geometry.Pose2;
import com.antu.core.geometry.Vec3;
import com.antu.core.graph.In;
import com.antu.core.graph.Out;
import com.antu.core.log.Log;
import com.antu.core.msg.Odometry;
import com.antu.core.msg.PoseEstimate;
import com.antu.core.msg.TrackedPose;
import com.antu.core.node.Node;

/**
 * One pose from two sources: the visual tracker while it holds, wheel odometry
 * when it drops.
 *
 * <h2>Why this is not a switch</h2>
 *
 * <p>The obvious implementation — use the tracker, and fall back to odometry —
 * is wrong, because the two are in different frames with different origins.
 * Switching between them teleports the robot.
 *
 * <p>What is maintained instead is the transform between the odometry frame and
 * the world. While the tracker holds, that transform is continuously corrected,
 * and the output equals the tracker. When the tracker drops, the transform is
 * <em>frozen</em> and odometry carries the estimate forward from the last good
 * fix. The output stays continuous across the handover, which is the property
 * everything downstream depends on, and the error grows from zero rather than
 * from wherever odometry happened to have drifted to since boot.
 *
 * <p>This is the same construction ROS calls the map-to-odom transform, for the
 * same reason.
 *
 * <h2>Frames</h2>
 *
 * <p>ARCore's world is y-up and the robot's is z-up, so the horizontal plane has
 * to be re-expressed. A y-up frame seen from above and a z-up frame seen from
 * above disagree about which way angles run, which is why the mapping below looks
 * like a reflection: it is, in the 2D projection, and the full 3D relationship is
 * still a proper rotation. Getting this wrong makes the robot turn the right
 * amount in the wrong direction, which looks like a sign error somewhere else
 * entirely.
 *
 * <p>Verified by driving: a spin gave the same handedness, and a forward run put
 * the travel bearing where this mapping predicts.
 */
public final class PoseFusion extends Node {

    private static final String TAG = "fusion";
    /** A correction larger than this on re-acquisition is worth saying out loud. */
    private static final double NOTABLE_CORRECTION_M = 0.25;

    /** Where the visual tracker says the camera is. */
    public final In<TrackedPose> tracked = in("tracked", TrackedPose.class);
    /** What the wheels say, which is continuous but drifts. */
    public final In<Odometry> odom = in("odom", Odometry.class);

    /** Best available pose, with its provenance. */
    public final Out<PoseEstimate> pose = out("pose", PoseEstimate.class);

    /**
     * Transform from the odometry frame into the world frame.
     *
     * <p>Corrected while the tracker holds, frozen while it does not. This one
     * field is the whole mechanism.
     */
    private Pose2 worldFromOdom = Pose2.ORIGIN;
    private boolean everTracked;
    private boolean wasTracking;
    private long lastFixNanos;
    private double lastCorrection;

    private double mountForward;
    private double mountLateral;
    private double mountYaw;

    public PoseFusion() {
        super("fusion");
    }

    /**
     * Where the camera sits relative to the robot's turning centre.
     *
     * <p>The tracker reports the camera's pose, not the robot's, and on a spin in
     * place those differ by the whole lever arm. {@code tools/calibrate-phone.py}
     * measures the distance; the direction needs the chord's bearing relative to
     * heading, so it is set here rather than guessed.
     *
     * @param forward metres ahead of the turning centre, negative for behind
     * @param lateral metres to the left
     * @param yaw     radians the camera is rotated from robot-forward
     */
    public PoseFusion setMount(double forward, double lateral, double yaw) {
        this.mountForward = forward;
        this.mountLateral = lateral;
        this.mountYaw = yaw;
        return this;
    }

    /** The current odometry-to-world transform, for diagnostics. */
    public Pose2 worldFromOdom() {
        return worldFromOdom;
    }

    @Override public void start(Node.Context ctx) {
        worldFromOdom = Pose2.ORIGIN;
        everTracked = false;
        wasTracking = false;
        lastCorrection = 0;
        lastFixNanos = ctx.clock().now().nanos();
    }

    @Override public void tick(Node.Context ctx) {
        TrackedPose t = tracked.get();
        Odometry o = odom.get();
        if (o == null) {
            return;                       // nothing to anchor to yet
        }
        long now = ctx.clock().now().nanos();
        Pose2 odomPose = o.pose;

        boolean tracking = t != null && t.isTracking();

        if (tracking) {
            Pose2 fromTracker = robotPoseFromCamera(t);

            if (!wasTracking && everTracked) {
                // Re-acquired after a blind stretch. The world just moved; measure
                // by how much and say so rather than smoothing it away.
                Pose2 predicted = worldFromOdom.compose(odomPose);
                lastCorrection = predicted.distanceTo(fromTracker);
                if (lastCorrection > NOTABLE_CORRECTION_M) {
                    Log.w(TAG, String.format(
                            "tracker re-acquired after %.1fs; pose corrected by %.2f m",
                            (now - lastFixNanos) / 1e9, lastCorrection), null);
                }
            }

            // Correct the transform so that odometry, pushed through it, lands
            // exactly where the tracker says. While this holds, the output is the
            // tracker's answer.
            worldFromOdom = fromTracker.compose(odomPose.inverse());
            everTracked = true;
            wasTracking = true;
            lastFixNanos = now;

            pose.publish(new PoseEstimate(fromTracker, PoseEstimate.Source.TRACKED,
                    0, lastCorrection));
            return;
        }

        if (wasTracking) {
            wasTracking = false;
            Log.w(TAG, "tracker lost" + (t == null ? "" : ": " + t.reason)
                    + "; dead reckoning from the last fix", null);
        }

        // Frozen transform, odometry carrying on. Continuous across the handover,
        // and the error starts from zero rather than from accumulated drift.
        pose.publish(new PoseEstimate(
                worldFromOdom.compose(odomPose),
                PoseEstimate.Source.DEAD_RECKONED,
                (now - lastFixNanos) / 1e9,
                lastCorrection));
    }

    /**
     * The robot's pose on the floor, from the camera's pose in ARCore's world.
     *
     * <p>Two conversions. ARCore is y-up so its horizontal plane is x-z, and the
     * projection into a z-up frame reverses the sense of angles — hence the signs
     * below, which were confirmed by driving rather than reasoned about. Then the
     * mount offset moves the estimate from the camera to the turning centre.
     */
    private Pose2 robotPoseFromCamera(TrackedPose t) {
        Vec3 p = t.pose.position;
        double heading = t.pose.rotation.yawAboutY() - mountYaw;
        Pose2 camera = new Pose2(-p.z, -p.x, heading);

        if (mountForward == 0 && mountLateral == 0) {
            return camera;
        }
        // Step back along the camera's own axes to the turning centre.
        return camera.compose(new Pose2(-mountForward, -mountLateral, 0));
    }
}
