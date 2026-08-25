package com.antu.brain;

import com.antu.core.Check;
import com.antu.core.geometry.Twist2;
import com.antu.core.graph.Graph;
import com.antu.core.graph.In;
import com.antu.core.graph.Out;
import com.antu.core.node.Node;
import com.antu.core.time.ManualClock;
import com.antu.core.time.Rate;

import java.util.ArrayList;
import java.util.List;

/**
 * Who drives, and when control changes hands.
 *
 * <p>Runs against a manual clock, so a one-second cooldown is tested by advancing
 * time rather than sleeping through it, and the result is the same every run.
 */
public final class ArbiterTest {

    public static void main(String[] args) throws Exception {
        Check c = new Check("ArbiterTest");

        autonomyDrivesWhenNobodyElseDoes(c);
        teleopPreempts(c);
        releasingDoesNotHandBackImmediately(c);
        autonomyResumesAfterCooldown(c);
        silenceIsNotACommand(c);
        limitsApplyToBothSources(c);
        limitsPreserveTheArc(c);
        teleopScaling(c);
        worksWithNoAutonomyWired(c);

        c.finish();
    }

    // ---------- fixtures ----------

    /** Publishes whatever it is told to, when it is told to. */
    private static final class Source extends Node {
        final Out<Twist2> out = out("cmd", Twist2.class);
        Twist2 next;

        Source(String name) {
            super(name);
        }

        @Override public void tick(Node.Context ctx) {
            if (next != null) {
                out.publish(next);
                next = null;
            }
        }
    }

    /** Records everything the arbiter decided. */
    private static final class Base extends Node {
        final In<Twist2> cmd = in("cmd_vel", Twist2.class, null);
        final List<Twist2> got = new ArrayList<>();

        Base() {
            super("base");
        }

        @Override public void tick(Node.Context ctx) {
            if (cmd.isFresh()) {
                got.add(cmd.get());
            }
        }
    }

    /** A wired-up graph, ready to step. */
    private static final class Rig {
        final ManualClock clock = new ManualClock();
        final Source teleop = new Source("teleop");
        final Source autonomy = new Source("autonomy");
        final CommandArbiter arbiter = new CommandArbiter();
        final Base base = new Base();
        final Graph graph;

        Rig(boolean wireAutonomy) {
            Graph.Builder b = Graph.builder(clock)
                    .add(teleop, Rate.hz(10))
                    .add(arbiter, Rate.hz(10))
                    .add(base, Rate.hz(10))
                    .connect(teleop.out, arbiter.teleop)
                    .connect(arbiter.cmdVel, base.cmd);
            if (wireAutonomy) {
                b.add(autonomy, Rate.hz(10)).connect(autonomy.out, arbiter.autonomy);
            }
            graph = b.build();
        }

        Rig start() throws Exception {
            graph.start();
            return this;
        }

        void step(int n) throws Exception {
            graph.step(n);
        }

        Twist2 last() {
            return base.got.isEmpty() ? null : base.got.get(base.got.size() - 1);
        }
    }

    // ---------- tests ----------

    private static void autonomyDrivesWhenNobodyElseDoes(Check c) throws Exception {
        Rig r = new Rig(true).start();
        r.autonomy.next = Twist2.of(0.3, 0.1);
        r.step(2);
        c.eq("idle: autonomy drives", Twist2.of(0.3, 0.1), r.last());
        r.graph.stop();
    }

    private static void teleopPreempts(Check c) throws Exception {
        Rig r = new Rig(true).start();
        r.autonomy.next = Twist2.of(0.3, 0);
        r.step(2);
        c.eq("preempt: autonomy first", Twist2.of(0.3, 0), r.last());

        r.teleop.next = Twist2.of(-0.2, 0.5);
        r.autonomy.next = Twist2.of(0.3, 0);      // still asking, and ignored
        r.step(2);
        c.eq("preempt: teleop wins", Twist2.of(-0.2, 0.5), r.last());
        c.eq("preempt: reported as holding", true, r.arbiter.isTeleopHolding());
        r.graph.stop();
    }

    /**
     * The reason the cooldown exists. Releasing a stick publishes a zero; without
     * a hold that zero is the last teleop message, autonomy resumes on the next
     * tick, and the robot drives off the instant you let go.
     */
    private static void releasingDoesNotHandBackImmediately(Check c) throws Exception {
        Rig r = new Rig(true).start();
        r.teleop.next = Twist2.of(0.4, 0);
        r.step(2);

        r.teleop.next = Twist2.ZERO;              // finger lifts
        r.autonomy.next = Twist2.of(0.5, 0);      // planner still wants to go
        r.step(3);                                 // 300 ms, well inside the cooldown

        c.eq("release: still stopped", Twist2.ZERO, r.last());
        c.eq("release: teleop still holds", true, r.arbiter.isTeleopHolding());
        r.graph.stop();
    }

    private static void autonomyResumesAfterCooldown(Check c) throws Exception {
        Rig r = new Rig(true).start();
        r.arbiter.setCooldown(0.5);
        r.teleop.next = Twist2.of(0.4, 0);
        r.step(2);

        r.teleop.next = Twist2.ZERO;
        r.step(2);
        c.eq("cooldown: held during", true, r.arbiter.isTeleopHolding());

        r.step(8);                                 // past 500 ms with no teleop
        c.eq("cooldown: released after", false, r.arbiter.isTeleopHolding());

        r.autonomy.next = Twist2.of(0.25, 0);
        r.step(2);
        c.eq("cooldown: autonomy resumes", Twist2.of(0.25, 0), r.last());
        r.graph.stop();
    }

    /**
     * With nobody driving, the arbiter says nothing rather than streaming zeros.
     * A continuous zero is indistinguishable from a planner that has died, and
     * the base driver's own silence timeout is the better judge.
     */
    private static void silenceIsNotACommand(Check c) throws Exception {
        Rig r = new Rig(true).start();
        r.teleop.next = Twist2.of(0.4, 0);
        r.step(2);
        int afterDriving = r.base.got.size();

        r.teleop.next = Twist2.ZERO;
        r.step(30);                                // long past the cooldown
        int afterRelease = r.base.got.size();

        r.step(30);                                // and longer still
        c.eq("silence: one zero on the way out, then quiet",
                afterRelease, r.base.got.size());
        c.eq("silence: something was sent while driving", true, afterDriving > 0);
        c.eq("silence: last word was stop", Twist2.ZERO, r.last());
        r.graph.stop();
    }

    private static void limitsApplyToBothSources(Check c) throws Exception {
        Rig r = new Rig(true).start();
        r.arbiter.setLimits(0.5, 1.0);

        r.autonomy.next = Twist2.of(9.0, 0);       // a unit bug upstream
        r.step(2);
        c.eq("limits: autonomy capped", 0.5, round(r.last().linearX));

        r.teleop.next = Twist2.of(-9.0, 0);
        r.step(2);
        c.eq("limits: teleop capped too", -0.5, round(r.last().linearX));
        r.graph.stop();
    }

    /**
     * Scaling both components together rather than clipping each keeps the path
     * the robot was asked to follow. Clipping separately tightens the curve near
     * the limit, so a planned arc quietly becomes a different one.
     */
    private static void limitsPreserveTheArc(Check c) throws Exception {
        Rig r = new Rig(true).start();
        r.arbiter.setLimits(0.5, 10.0);
        r.autonomy.next = Twist2.of(1.0, 2.0);     // ratio 2.0
        r.step(2);
        Twist2 out = r.last();
        c.eq("arc: linear capped", 0.5, round(out.linearX));
        c.eq("arc: ratio preserved", 2.0, round(out.angular / out.linearX));
        r.graph.stop();
    }

    private static void teleopScaling(Check c) throws Exception {
        Rig r = new Rig(true).start();
        r.arbiter.setTeleopScale(0.5);
        r.teleop.next = Twist2.of(0.4, 0.8);
        r.step(2);
        c.eq("scale: linear halved", 0.2, round(r.last().linearX));
        c.eq("scale: angular halved", 0.4, round(r.last().angular));
        r.graph.stop();
    }

    /** A robot with no autonomy wired in is a normal thing to build. */
    private static void worksWithNoAutonomyWired(Check c) throws Exception {
        Rig r = new Rig(false).start();
        r.teleop.next = Twist2.of(0.3, 0);
        r.step(2);
        c.eq("no autonomy: teleop still drives", Twist2.of(0.3, 0), r.last());
        r.graph.stop();
    }

    private static double round(double v) {
        return Math.round(v * 1000.0) / 1000.0;
    }
}
