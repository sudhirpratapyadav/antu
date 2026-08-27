package com.antu.brain;

import com.antu.core.geometry.Angles;
import com.antu.core.geometry.Pose2;
import com.antu.core.geometry.Twist2;
import com.antu.core.graph.In;
import com.antu.core.graph.Out;
import com.antu.core.log.Log;
import com.antu.core.msg.PoseEstimate;
import com.antu.core.node.Node;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Executes one relative move — turn, then drive — closed-loop on the fused pose.
 *
 * <p>This is the robot's guarded motion primitive: "turn 40° and go 0.8 m"
 * rather than a stream of velocities. The caller finds out how it ended —
 * completed, preempted, timed out, or lost the pose — which is the property a
 * remote planner actually needs. A model deciding what to do next from a
 * camera frame cannot close a control loop over a network round-trip; this
 * node closes it on the robot at tick rate, and the network only carries the
 * goal and the outcome.
 *
 * <h2>Bounded on purpose</h2>
 *
 * <p>Goals are clamped: a step of at most {@link #MAX_STEP_M} at no more than
 * {@link #MAX_SPEED}, with a deadline derived from the goal rather than trusted
 * from the caller. Whatever asks for a move — an agent with a unit bug, a
 * harness gone wrong — the robot moves at most one short, slow step per
 * request, and the watchdogs downstream still apply. The arbiter is wired
 * between this node and the base, so teleop overrides autonomy the moment a
 * human touches a control.
 *
 * <h2>One goal at a time</h2>
 *
 * <p>A new goal preempts the previous one, which completes with
 * {@code preempted} rather than blocking or queueing. Queued motion is how a
 * robot executes a plan the world has moved on from; whoever submits decides
 * what the newest goal is, and history is not consulted.
 */
public final class MoveExecutor extends Node {

    private static final String TAG = "move";

    /** The longest single step. A planner wanting more asks again. */
    public static final double MAX_STEP_M = 1.0;
    /** Indoor pace, below the arbiter's own clamp. */
    public static final double MAX_SPEED = 0.3;
    private static final double MAX_TURN_RATE = 0.7;      // rad/s
    private static final double MIN_TURN_RATE = 0.12;     // overcomes stiction
    private static final double MIN_SPEED = 0.05;
    private static final double TURN_TOL = 0.035;         // ~2 degrees
    private static final double DIST_TOL = 0.03;          // metres
    private static final double K_TURN = 1.6;
    private static final double K_LIN = 1.2;
    /** How long to wait for the first pose before giving up on a goal. */
    private static final double NO_POSE_S = 2.0;

    /** The fused pose the loop closes on. */
    public final In<PoseEstimate> pose = in("pose", PoseEstimate.class);
    /** Commands, via the arbiter's autonomy input, never the base directly. */
    public final Out<Twist2> cmdVel = out("cmd_vel", Twist2.class);

    /** How a move ended. */
    public static final class Result {
        /** completed, preempted, timeout, or no_pose. */
        public final String outcome;
        /** Where the robot was when it ended, or null if no pose ever arrived. */
        public final Pose2 finalPose;

        Result(String outcome, Pose2 finalPose) {
            this.outcome = outcome;
            this.finalPose = finalPose;
        }
    }

    /** A submitted move: an opaque handle whose only public face is its result. */
    public static final class Goal {
        final double dtheta;
        final double distance;      // signed: negative backs up
        final double maxSpeed;
        final CountDownLatch done = new CountDownLatch(1);
        volatile Result result;

        // Owned by the tick thread once the goal is active.
        boolean initialised;
        long deadlineNanos;
        long firstTickNanos;
        double targetTheta;
        Pose2 driveStart;
        boolean turning;

        Goal(double dtheta, double distance, double maxSpeed) {
            this.dtheta = dtheta;
            this.distance = distance;
            this.maxSpeed = maxSpeed;
        }
    }

    private final AtomicReference<Goal> active = new AtomicReference<>();
    private boolean wasDriving;

    public MoveExecutor() {
        super("move");
    }

    /**
     * Submits a goal without waiting; the previous goal, if any, is preempted.
     * Angles in radians, distance in metres (negative reverses).
     */
    public Goal submit(double dthetaRad, double distanceM, double maxSpeed) {
        Goal goal = new Goal(
                Angles.normalise(dthetaRad),
                Math.max(-MAX_STEP_M, Math.min(MAX_STEP_M, distanceM)),
                Math.max(MIN_SPEED, Math.min(MAX_SPEED, maxSpeed)));
        Goal previous = active.getAndSet(goal);
        if (previous != null) {
            finish(previous, "preempted", null);
        }
        return goal;
    }

    /**
     * Submits a goal and blocks until it ends. Safe to call from any thread —
     * an HTTP handler, typically — while the graph ticks the motion.
     */
    public Result execute(double dthetaRad, double distanceM, double maxSpeed) {
        Goal goal = submit(dthetaRad, distanceM, maxSpeed);
        // The tick loop enforces the real deadline; this wait is only a backstop
        // for a graph that stopped ticking, and is sized to outlast any goal.
        try {
            if (!goal.done.await(60, TimeUnit.SECONDS)) {
                finish(goal, "timeout", null);
                active.compareAndSet(goal, null);
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            finish(goal, "preempted", null);
            active.compareAndSet(goal, null);
        }
        return goal.result;
    }

    /** The result of a submitted goal, for step-driven callers and tests. */
    public Result resultOf(Goal goal) {
        return goal.result;
    }

    @Override public void tick(Context ctx) {
        Goal g = active.get();
        if (g == null) {
            if (wasDriving) {
                // One explicit zero on the way out; after that, silence, so the
                // arbiter's own autonomy timeout takes over.
                cmdVel.publish(Twist2.ZERO);
                wasDriving = false;
            }
            return;
        }

        long now = ctx.clock().now().nanos();
        PoseEstimate estimate = pose.get();
        if (estimate == null) {
            if (g.firstTickNanos == 0) {
                g.firstTickNanos = now;
            } else if (now - g.firstTickNanos > (long) (NO_POSE_S * 1e9)) {
                end(g, "no_pose", null);
            }
            return;
        }
        Pose2 p = estimate.pose;

        if (!g.initialised) {
            g.initialised = true;
            g.targetTheta = Angles.normalise(p.theta + g.dtheta);
            g.turning = Math.abs(g.dtheta) > TURN_TOL;
            // The deadline comes from the goal, not the caller: generous enough
            // for stiction and settling, bounded enough that a wedged robot
            // reports instead of pushing.
            double seconds = 4.0
                    + Math.abs(g.dtheta) / (MAX_TURN_RATE * 0.5)
                    + Math.abs(g.distance) / (g.maxSpeed * 0.5);
            g.deadlineNanos = now + (long) (seconds * 1e9);
            if (!g.turning) {
                g.driveStart = p;
            }
        }

        if (now > g.deadlineNanos) {
            end(g, "timeout", p);
            return;
        }

        if (g.turning) {
            double error = Angles.difference(p.theta, g.targetTheta);
            if (Math.abs(error) <= TURN_TOL) {
                g.turning = false;
                g.driveStart = p;
            } else {
                double rate = Math.max(MIN_TURN_RATE,
                        Math.min(MAX_TURN_RATE, Math.abs(error) * K_TURN));
                cmdVel.publish(Twist2.of(0, Math.copySign(rate, error)));
                wasDriving = true;
                return;
            }
        }

        double direction = Math.signum(g.distance);
        if (direction == 0 || Math.abs(g.distance) <= DIST_TOL) {
            end(g, "completed", p);
            return;
        }
        // Along-track progress: how far along the intended line, not how far
        // from the start — a robot pushed sideways has not advanced.
        double ux = Math.cos(g.targetTheta);
        double uy = Math.sin(g.targetTheta);
        double travelled = (p.x - g.driveStart.x) * ux + (p.y - g.driveStart.y) * uy;
        double remaining = Math.abs(g.distance) - direction * travelled;
        if (remaining <= DIST_TOL) {
            end(g, "completed", p);
            return;
        }

        double speed = Math.max(MIN_SPEED, Math.min(g.maxSpeed, remaining * K_LIN));
        // Keep pointing down the line while driving; drift builds otherwise.
        double headingError = Angles.difference(p.theta, g.targetTheta);
        double correction = Math.max(-0.3, Math.min(0.3, headingError * K_TURN));
        cmdVel.publish(Twist2.of(direction * speed, correction));
        wasDriving = true;
    }

    private void end(Goal g, String outcome, Pose2 at) {
        cmdVel.publish(Twist2.ZERO);
        wasDriving = false;
        active.compareAndSet(g, null);
        finish(g, outcome, at);
        if (!"completed".equals(outcome)) {
            Log.w(TAG, "move ended: " + outcome, null);
        }
    }

    private static void finish(Goal g, String outcome, Pose2 at) {
        if (g.result == null) {
            g.result = new Result(outcome, at);
        }
        g.done.countDown();
    }
}
