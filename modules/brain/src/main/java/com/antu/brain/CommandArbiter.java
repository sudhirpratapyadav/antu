package com.antu.brain;

import com.antu.core.geometry.Twist2;
import com.antu.core.graph.In;
import com.antu.core.graph.Out;
import com.antu.core.log.Log;
import com.antu.core.node.Node;

/**
 * Decides who is driving.
 *
 * <p>A robot has more than one thing that wants to command velocity — a person
 * on a joystick, a planner, later an agent — and a base taking commands from two
 * of them stutters between them. The graph refuses to wire two writers to one
 * input, which is the right answer structurally but leaves the question of who
 * wins. This node is that answer: several named inputs, one output.
 *
 * <h2>Teleop preempts, and hands back slowly</h2>
 *
 * <p>A person touching the controls takes over immediately. Autonomy resumes only
 * after {@link #setCooldown} has passed with no teleop input at all.
 *
 * <p>The cooldown is the part worth understanding. Releasing a joystick publishes
 * a <em>zero</em>, and without a cooldown that zero would be the last teleop
 * message, autonomy would resume on the very next tick, and the robot would
 * drive off the instant you let go. So any teleop message refreshes the hold,
 * including a zero one — letting go means stop, and it means stop for a moment
 * longer than it takes to notice.
 *
 * <h2>Silence is not a command</h2>
 *
 * <p>With nobody driving, this publishes nothing rather than a stream of zeros.
 * A continuous zero looks identical to a planner that has died, and the base
 * driver's own silence timeout is the better judge of that. One zero is sent on
 * the way out so the robot stops promptly, and then it goes quiet.
 */
public final class CommandArbiter extends Node {

    private static final String TAG = "arbiter";
    /** Default hold after the last teleop message. Matches what dimos settled on. */
    private static final double DEFAULT_COOLDOWN_SECONDS = 1.0;

    /** Velocity from a person. Wins whenever it is present. */
    public final In<Twist2> teleop = in("teleop", Twist2.class, Twist2.ZERO);

    /**
     * Velocity from a planner or an agent. Optional, because a robot with no
     * autonomy wired in is a normal thing to build and should still drive.
     */
    public final In<Twist2> autonomy = in("autonomy", Twist2.class, Twist2.ZERO).optional();

    /** What the base should do. */
    public final Out<Twist2> cmdVel = out("cmd_vel", Twist2.class);

    /**
     * Whether a person currently holds control.
     *
     * <p>So autonomy can pause deliberately instead of publishing into a void and
     * wondering why the robot ignores it. A planner that knows it has been
     * preempted can hold its plan rather than treating the time as progress.
     */
    public final Out<Boolean> teleopActive = out("teleop_active", Boolean.class);

    private long cooldownNanos = (long) (DEFAULT_COOLDOWN_SECONDS * 1e9);
    private double teleopScale = 1.0;
    private double maxLinear = 1.5;
    private double maxAngular = 2.0;

    private long lastTeleopNanos = Long.MIN_VALUE;
    private boolean holding;
    private boolean announced;

    public CommandArbiter() {
        super("arbiter");
    }

    /** How long teleop keeps control after its last message. 0 hands back at once. */
    public CommandArbiter setCooldown(double seconds) {
        this.cooldownNanos = (long) (Math.max(0, seconds) * 1e9);
        return this;
    }

    /** Scales teleop input, for a stick that is too lively on a given robot. */
    public CommandArbiter setTeleopScale(double scale) {
        this.teleopScale = scale;
        return this;
    }

    /**
     * Caps whatever passes through, whoever sent it.
     *
     * <p>The last gate before the base, so a planner with a unit bug cannot ask
     * for 40 m/s. Limits scale both components together rather than clipping each,
     * which preserves the arc the robot was asked to follow.
     */
    public CommandArbiter setLimits(double maxLinearMetresPerSecond,
                                    double maxAngularRadiansPerSecond) {
        this.maxLinear = maxLinearMetresPerSecond;
        this.maxAngular = maxAngularRadiansPerSecond;
        return this;
    }

    /** True while a person holds control. */
    public boolean isTeleopHolding() {
        return holding;
    }

    @Override public void start(Node.Context ctx) {
        lastTeleopNanos = Long.MIN_VALUE;
        holding = false;
        announced = false;
    }

    @Override public void tick(Node.Context ctx) {
        long now = ctx.clock().now().nanos();

        // Any teleop message refreshes the hold, including a zero: letting go is
        // itself an instruction, and the most important one.
        if (teleop.isFresh()) {
            lastTeleopNanos = now;
        }
        // Long.MIN_VALUE would overflow the subtraction, so test the sentinel
        // rather than the difference. The rate limiter learned this the hard way.
        boolean nowHolding = lastTeleopNanos != Long.MIN_VALUE
                && now - lastTeleopNanos < cooldownNanos;

        if (nowHolding != holding) {
            holding = nowHolding;
            teleopActive.publish(holding);
            if (holding) {
                Log.i(TAG, "teleop has control");
            } else {
                Log.i(TAG, "teleop released; autonomy may resume");
                // One zero on the way out, so the robot stops promptly rather than
                // coasting until the base driver's own timeout notices.
                cmdVel.publish(Twist2.ZERO);
                announced = false;
                return;
            }
        }
        if (!announced) {
            announced = true;
            teleopActive.publish(holding);
        }

        if (holding) {
            // Republished every tick: a single dropped message must not leave the
            // base holding a stale velocity.
            cmdVel.publish(limit(teleop.get().scaled(teleopScale)));
            return;
        }

        if (autonomy.isConnected() && autonomy.isFresh()) {
            cmdVel.publish(limit(autonomy.get()));
        }
        // Otherwise nothing is driving, so say nothing. A stream of zeros looks
        // exactly like a planner that has stopped, and hides it.
    }

    private Twist2 limit(Twist2 command) {
        return command.limited(maxLinear, maxAngular);
    }
}
