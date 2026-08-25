package com.antu.brain;

import com.antu.core.Check;
import com.antu.core.geometry.Pose2;
import com.antu.core.geometry.Pose3;
import com.antu.core.geometry.Quat;
import com.antu.core.geometry.Twist2;
import com.antu.core.geometry.Vec3;
import com.antu.core.graph.Graph;
import com.antu.core.graph.In;
import com.antu.core.graph.Out;
import com.antu.core.msg.Odometry;
import com.antu.core.msg.PoseEstimate;
import com.antu.core.msg.TrackedPose;
import com.antu.core.node.Node;
import com.antu.core.time.ManualClock;
import com.antu.core.time.Rate;

import java.util.ArrayList;
import java.util.List;

/**
 * Fusing a visual tracker with wheel odometry.
 *
 * <p>The property under test is continuity. A tracker dropping must not teleport
 * the robot, and coming back must not be smoothed into a lie.
 */
public final class FusionTest {

    public static void main(String[] args) throws Exception {
        Check c = new Check("FusionTest");

        arcoreFrameIsConvertedCorrectly(c);
        trackerWinsWhileItHolds(c);
        losingTheTrackerIsContinuous(c);
        deadReckoningFollowsOdometry(c);
        reacquisitionReportsTheJump(c);
        odometryDriftIsNotInherited(c);
        worksBeforeTheTrackerEverLocks(c);

        c.finish();
    }

    // ---------- fixtures ----------

    private static final class Tracker extends Node {
        final Out<TrackedPose> out = out("tracked", TrackedPose.class);
        TrackedPose next = stopped();

        Tracker() {
            super("tracker");
        }

        @Override public void tick(Node.Context ctx) {
            out.publish(next);
        }
    }

    private static final class Wheels extends Node {
        final Out<Odometry> out = out("odom", Odometry.class);
        Pose2 pose = Pose2.ORIGIN;

        Wheels() {
            super("wheels");
        }

        @Override public void tick(Node.Context ctx) {
            out.publish(new Odometry(pose, Twist2.ZERO));
        }
    }

    private static final class Consumer extends Node {
        final In<PoseEstimate> in = in("pose", PoseEstimate.class);
        final List<PoseEstimate> got = new ArrayList<>();

        Consumer() {
            super("consumer");
        }

        @Override public void tick(Node.Context ctx) {
            if (in.isFresh()) {
                got.add(in.get());
            }
        }
    }

    private static final class Rig {
        final ManualClock clock = new ManualClock();
        final Tracker tracker = new Tracker();
        final Wheels wheels = new Wheels();
        final PoseFusion fusion = new PoseFusion();
        final Consumer consumer = new Consumer();
        final Graph graph;

        Rig() {
            graph = Graph.builder(clock)
                    .add(tracker, Rate.hz(10))
                    .add(wheels, Rate.hz(10))
                    .add(fusion, Rate.hz(10))
                    .add(consumer, Rate.hz(10))
                    .connect(tracker.out, fusion.tracked)
                    .connect(wheels.out, fusion.odom)
                    .connect(fusion.pose, consumer.in)
                    .build();
        }

        PoseEstimate last() {
            return consumer.got.isEmpty() ? null : consumer.got.get(consumer.got.size() - 1);
        }
    }

    private static TrackedPose stopped() {
        return new TrackedPose(Pose3.IDENTITY, TrackedPose.State.STOPPED, "", 0);
    }

    /**
     * ARCore reports a camera at (x, y, z) with y up, heading as rotation about y.
     * The robot works in a z-up frame. Building the tracked pose from a known
     * robot pose and asking for it back is the tightest check of that mapping.
     */
    private static TrackedPose trackingAt(double robotX, double robotY, double heading) {
        // Inverse of the conversion under test: robot x is -z, robot y is -x.
        Vec3 position = new Vec3(-robotY, 0, -robotX);
        return new TrackedPose(new Pose3(position, Quat.fromAxisAngle(new Vec3(0, 1, 0), heading)),
                TrackedPose.State.TRACKING, "", 1);
    }

    private static TrackedPose lost(TrackedPose last) {
        return new TrackedPose(last.pose, TrackedPose.State.PAUSED,
                "INSUFFICIENT_FEATURES", last.frames + 1);
    }

    // ---------- tests ----------

    private static void arcoreFrameIsConvertedCorrectly(Check c) throws Exception {
        Rig r = new Rig();
        r.graph.start();
        r.tracker.next = trackingAt(2.0, -1.0, Math.PI / 2);
        r.graph.step(2);

        PoseEstimate e = r.last();
        c.eq("frame: x", 2.0, round(e.pose.x));
        c.eq("frame: y", -1.0, round(e.pose.y));
        c.eq("frame: heading", round(Math.PI / 2), round(e.pose.theta));
        c.eq("frame: reported as tracked", PoseEstimate.Source.TRACKED, e.source);
        r.graph.stop();
    }

    private static void trackerWinsWhileItHolds(Check c) throws Exception {
        Rig r = new Rig();
        r.graph.start();
        // Odometry says one thing, the tracker another. The tracker is anchored to
        // the room, so it wins outright.
        r.wheels.pose = new Pose2(50, 50, 1.0);
        r.tracker.next = trackingAt(1.0, 0.0, 0.0);
        r.graph.step(2);

        c.eq("tracked: follows the tracker, not the wheels", 1.0, round(r.last().pose.x));
        c.eq("tracked: anchored", true, r.last().isAnchored());
        r.graph.stop();
    }

    /**
     * The property the whole design exists for. Switching sources naively would
     * teleport the robot from the tracker's frame into odometry's.
     */
    private static void losingTheTrackerIsContinuous(Check c) throws Exception {
        Rig r = new Rig();
        r.graph.start();
        // Odometry is a long way off in its own frame — exactly the case a naive
        // fallback gets wrong.
        r.wheels.pose = new Pose2(100, -70, 0.0);
        r.tracker.next = trackingAt(3.0, 4.0, 0.0);
        r.graph.step(2);
        Pose2 before = r.last().pose;

        r.tracker.next = lost(r.tracker.next);
        r.graph.step(2);
        Pose2 after = r.last().pose;

        c.eq("handover: no jump", true, before.distanceTo(after) < 1e-6);
        c.eq("handover: now dead reckoning",
                PoseEstimate.Source.DEAD_RECKONED, r.last().source);
        r.graph.stop();
    }

    private static void deadReckoningFollowsOdometry(Check c) throws Exception {
        Rig r = new Rig();
        r.graph.start();
        r.wheels.pose = new Pose2(10, 10, 0.0);
        r.tracker.next = trackingAt(0.0, 0.0, 0.0);
        r.graph.step(2);

        r.tracker.next = lost(r.tracker.next);
        r.graph.step(2);
        // The wheels advance a metre in their own frame; the estimate should
        // advance a metre in the world frame.
        r.wheels.pose = new Pose2(11, 10, 0.0);
        r.graph.step(2);

        c.eq("dead reckoning: moved with the wheels", 1.0, round(r.last().pose.x));
        c.eq("dead reckoning: age reported", true, r.last().secondsSinceFix > 0);
        r.graph.stop();
    }

    private static void reacquisitionReportsTheJump(Check c) throws Exception {
        Rig r = new Rig();
        r.graph.start();
        r.tracker.next = trackingAt(0.0, 0.0, 0.0);
        r.graph.step(2);

        r.tracker.next = lost(r.tracker.next);
        r.graph.step(2);
        r.wheels.pose = new Pose2(2.0, 0, 0);        // odometry thinks it went 2 m
        r.graph.step(2);

        com.antu.core.log.Log.setSink(com.antu.core.log.Log.NONE);
        // The tracker comes back and says it actually went 2.5 m.
        r.tracker.next = trackingAt(2.5, 0.0, 0.0);
        r.graph.step(2);
        com.antu.core.log.Log.setSink(com.antu.core.log.Log.CONSOLE);

        c.eq("reacquire: snaps to the tracker", 2.5, round(r.last().pose.x));
        // Reported, not smoothed away: a planner deserves to know its world moved.
        c.eq("reacquire: correction measured", 0.5, round(r.last().lastCorrection));
        c.eq("reacquire: anchored again", true, r.last().isAnchored());
        r.graph.stop();
    }

    /**
     * Odometry drifting before the tracker ever locks must not poison the world
     * frame afterwards. Only motion since the last fix should count.
     */
    private static void odometryDriftIsNotInherited(Check c) throws Exception {
        Rig r = new Rig();
        r.graph.start();
        r.wheels.pose = new Pose2(37, -12, 2.1);     // hours of accumulated nonsense
        r.graph.step(2);

        r.tracker.next = trackingAt(0.0, 0.0, 0.0);
        r.graph.step(2);
        c.eq("drift: fix ignores accumulated odometry", 0.0, round(r.last().pose.x));

        r.tracker.next = lost(r.tracker.next);
        r.graph.step(2);
        // Half a metre forward, along the heading the robot actually has. Moving
        // it along odometry's own x instead would be the robot sliding sideways,
        // and the estimate would rightly show that rotated into the anchored
        // frame rather than as straight-ahead travel.
        r.wheels.pose = new Pose2(37 + 0.5 * Math.cos(2.1), -12 + 0.5 * Math.sin(2.1), 2.1);
        r.graph.step(2);
        c.eq("drift: only motion since the fix counts", 0.5, round(r.last().pose.x));
        c.eq("drift: forward travel stays forward", 0.0, round(r.last().pose.y));
        r.graph.stop();
    }

    private static void worksBeforeTheTrackerEverLocks(Check c) throws Exception {
        Rig r = new Rig();
        r.graph.start();
        r.wheels.pose = new Pose2(1.5, 0, 0);
        r.graph.step(2);

        // No fix yet, so odometry is all there is — and saying so is the point.
        c.eq("cold start: falls back to odometry", 1.5, round(r.last().pose.x));
        c.eq("cold start: not claimed as anchored", false, r.last().isAnchored());
        r.graph.stop();
    }

    private static double round(double v) {
        return Math.round(v * 1000.0) / 1000.0;
    }
}
