package com.antu.brain;

import com.antu.core.Check;
import com.antu.core.geometry.Angles;
import com.antu.core.geometry.Pose2;
import com.antu.core.geometry.Twist2;
import com.antu.core.graph.Graph;
import com.antu.core.graph.In;
import com.antu.core.graph.Out;
import com.antu.core.msg.PoseEstimate;
import com.antu.core.node.Node;
import com.antu.core.time.ManualClock;
import com.antu.core.time.Rate;

/**
 * The guarded motion primitive, driven against a simulated robot.
 *
 * <p>The simulation integrates whatever the executor commands into a pose and
 * feeds it back, so these tests exercise the actual control loop: turn to the
 * heading, drive the line, stop at the distance — and report honestly when any
 * of that cannot happen.
 */
public final class MoveExecutorTest {

    public static void main(String[] args) throws Exception {
        Check c = new Check("MoveExecutorTest");

        turnsThenDrivesToTheGoal(c);
        reversingWorks(c);
        pureRotationCompletes(c);
        aNewGoalPreemptsTheOld(c);
        aFrozenRobotTimesOut(c);
        noPoseIsItsOwnOutcome(c);
        goalsAreClamped(c);

        c.finish();
    }

    /** Check has eq/pass/fail; this adds the boolean form these tests want. */
    private static void ok(Check c, boolean condition, String label) {
        if (condition) {
            c.pass(label);
        } else {
            c.fail(label);
        }
    }

    // ---------- fixtures ----------

    /** Integrates commanded velocity into a pose, like a well-behaved robot. */
    private static final class Sim extends Node {
        final In<Twist2> cmd = in("cmd", Twist2.class, Twist2.ZERO);
        final Out<PoseEstimate> pose = out("pose", PoseEstimate.class);
        Pose2 p = Pose2.ORIGIN;
        boolean frozen;
        boolean silent;
        static final double DT = 0.05;          // matches 20 Hz below

        Sim() {
            super("sim");
        }

        @Override public void tick(Context ctx) {
            if (silent) {
                return;
            }
            Twist2 v = cmd.get();
            if (!frozen && v != null) {
                double theta = p.theta + v.angular * DT;
                p = new Pose2(
                        p.x + v.linearX * Math.cos(theta) * DT,
                        p.y + v.linearX * Math.sin(theta) * DT,
                        Angles.normalise(theta));
            }
            pose.publish(new PoseEstimate(p, PoseEstimate.Source.TRACKED, 0, 0));
        }
    }

    private static final class Rig {
        final ManualClock clock = new ManualClock();
        final Sim sim = new Sim();
        final MoveExecutor move = new MoveExecutor();
        final Graph graph;

        Rig() throws Exception {
            graph = Graph.builder(clock)
                    .add(sim, Rate.hz(20))
                    .add(move, Rate.hz(20))
                    .connect(sim.pose, move.pose)
                    .connectDelayed(move.cmdVel, sim.cmd)
                    .build();
            graph.start();
        }

        /** Ticks until the goal ends or simulated time runs out. */
        MoveExecutor.Result run(MoveExecutor.Goal goal, double maxSeconds) throws Exception {
            for (int i = 0; i < maxSeconds * 20 && move.resultOf(goal) == null; i++) {
                clock.advanceMillis(50);
                graph.step(1);
            }
            return move.resultOf(goal);
        }
    }

    // ---------- tests ----------

    private static void turnsThenDrivesToTheGoal(Check c) throws Exception {
        Rig rig = new Rig();
        MoveExecutor.Goal goal = rig.move.submit(Math.PI / 2, 0.8, 0.3);
        MoveExecutor.Result r = rig.run(goal, 30);

        ok(c, r != null && "completed".equals(r.outcome),
                "quarter turn and 0.8 m completes: " + (r == null ? "never" : r.outcome));
        ok(c, Angles.distance(rig.sim.p.theta, Math.PI / 2) < 0.1,
                "heading near +90 degrees, got "
                        + Angles.toDegrees(rig.sim.p.theta));
        double travelled = Math.hypot(rig.sim.p.x, rig.sim.p.y);
        ok(c, Math.abs(travelled - 0.8) < 0.1,
                "travelled about 0.8 m, got " + travelled);
        ok(c, Math.abs(rig.sim.p.x) < 0.1,
                "moved along +y after the turn, x stayed " + rig.sim.p.x);
    }

    private static void reversingWorks(Check c) throws Exception {
        Rig rig = new Rig();
        MoveExecutor.Result r = rig.run(rig.move.submit(0, -0.5, 0.3), 30);

        ok(c, r != null && "completed".equals(r.outcome), "reverse completes");
        ok(c, rig.sim.p.x < -0.4, "backed up along -x, got " + rig.sim.p.x);
    }

    private static void pureRotationCompletes(Check c) throws Exception {
        Rig rig = new Rig();
        MoveExecutor.Result r = rig.run(rig.move.submit(-Math.PI / 2, 0, 0.3), 30);

        ok(c, r != null && "completed".equals(r.outcome), "pure rotation completes");
        ok(c, Angles.distance(rig.sim.p.theta, -Math.PI / 2) < 0.1,
                "heading near -90 degrees, got " + Angles.toDegrees(rig.sim.p.theta));
        ok(c, Math.hypot(rig.sim.p.x, rig.sim.p.y) < 0.02,
                "did not translate while turning");
    }

    private static void aNewGoalPreemptsTheOld(Check c) throws Exception {
        Rig rig = new Rig();
        MoveExecutor.Goal first = rig.move.submit(0, 1.0, 0.3);
        rig.run(first, 0.5);                       // partway there
        MoveExecutor.Goal second = rig.move.submit(0, 0.2, 0.3);

        ok(c, "preempted".equals(rig.move.resultOf(first).outcome),
                "the first goal reports preempted");
        MoveExecutor.Result r = rig.run(second, 30);
        ok(c, r != null && "completed".equals(r.outcome), "the second goal completes");
    }

    private static void aFrozenRobotTimesOut(Check c) throws Exception {
        Rig rig = new Rig();
        rig.sim.frozen = true;                     // publishes pose, never moves
        MoveExecutor.Result r = rig.run(rig.move.submit(0, 0.5, 0.3), 60);

        ok(c, r != null && "timeout".equals(r.outcome),
                "a robot that will not move reports timeout: "
                        + (r == null ? "never ended" : r.outcome));
    }

    private static void noPoseIsItsOwnOutcome(Check c) throws Exception {
        Rig rig = new Rig();
        rig.sim.silent = true;                     // no pose at all
        MoveExecutor.Result r = rig.run(rig.move.submit(0, 0.5, 0.3), 10);

        ok(c, r != null && "no_pose".equals(r.outcome),
                "no pose reports no_pose, not a hang: "
                        + (r == null ? "never ended" : r.outcome));
    }

    private static void goalsAreClamped(Check c) throws Exception {
        Rig rig = new Rig();
        // Ask for far too much; the robot should do at most one bounded step.
        MoveExecutor.Result r = rig.run(rig.move.submit(0, 40.0, 5.0), 60);

        ok(c, r != null && "completed".equals(r.outcome), "oversized goal completes");
        ok(c, rig.sim.p.x < MoveExecutor.MAX_STEP_M + 0.1,
                "distance clamped to one step, got " + rig.sim.p.x);
    }
}
