package com.antu.core;

import com.antu.core.bus.Topic;
import com.antu.core.exec.Graph;
import com.antu.core.node.AbstractNode;
import com.antu.core.node.Node;
import com.antu.core.time.ManualClock;
import com.antu.core.time.Rate;

import java.util.ArrayList;
import java.util.List;

/** Scheduling, rate division, isolation, and reproducibility. */
public final class GraphTest {

    private static final Topic<Integer> TICKS = Topic.of("/ticks", Integer.class);

    public static void main(String[] args) throws Exception {
        Check c = new Check("GraphTest");

        ratesDivide(c);
        declaredOrderIsExecutionOrder(c);
        queuedMessagesArriveBeforeTick(c);
        orderDecidesLatency(c);
        aFailingNodeDoesNotStopTheGraph(c);
        stopUnsubscribes(c);
        duplicateNamesRejected(c);
        replayIsDeterministic(c);
        jitterDoesNotDropTicks(c);
        aSlowNodeGivesUpSlotsRatherThanBursting(c);

        c.finish();
    }

    /** A node that records when it ran. */
    private static final class Recorder extends AbstractNode {
        final List<Long> ticksAt = new ArrayList<>();
        final List<Integer> received = new ArrayList<>();
        final List<String> trace;
        boolean publishes;
        boolean throwsOnTick;
        /** Milliseconds of clock time to burn in tick(), to simulate an overrun. */
        long overrunMillis;
        ManualClock burnClock;

        Recorder(String name, List<String> trace) {
            super(name);
            this.trace = trace;
        }

        @Override public void start(Node.Context ctx) {
            ctx.subscribe(TICKS, m -> received.add(m.payload()));
        }

        @Override public void tick(Node.Context ctx) {
            trace.add(name());
            ticksAt.add(ctx.clock().now().millis());
            if (overrunMillis > 0 && burnClock != null) {
                burnClock.advanceMillis(overrunMillis);
            }
            if (throwsOnTick) {
                throw new IllegalStateException("deliberate");
            }
            if (publishes) {
                ctx.publish(TICKS, (int) ctx.tickCount());
            }
        }
    }

    private static void ratesDivide(Check c) throws Exception {
        ManualClock clock = new ManualClock();
        Graph g = new Graph(clock);
        List<String> trace = new ArrayList<>();
        Recorder fast = new Recorder("fast", trace);
        Recorder slow = new Recorder("slow", trace);
        g.add(fast, Rate.hz(100));      // every 10 ms
        g.add(slow, Rate.hz(10));       // every 100 ms
        g.start();
        g.step(100);                    // loop runs at the fastest rate: 100 x 10 ms
        g.stop();

        // A 100 Hz filter and a 10 Hz planner share one thread with no coordination.
        c.eq("rates: fast ticked every loop", 100, fast.ticksAt.size());
        c.eq("rates: slow ticked a tenth as often", 10, slow.ticksAt.size());
        c.eq("rates: slow spaced 100ms apart", 100L,
                slow.ticksAt.get(1) - slow.ticksAt.get(0));
    }

    private static void declaredOrderIsExecutionOrder(Check c) throws Exception {
        ManualClock clock = new ManualClock();
        Graph g = new Graph(clock);
        List<String> trace = new ArrayList<>();
        g.add(new Recorder("first", trace), Rate.hz(10));
        g.add(new Recorder("second", trace), Rate.hz(10));
        g.add(new Recorder("third", trace), Rate.hz(10));
        g.start();
        g.step(2);
        g.stop();

        // Insertion order, every loop. Putting a filter ahead of the planner that
        // consumes it therefore saves a whole cycle of latency.
        c.eq("order: follows insertion", "[first, second, third, first, second, third]",
                trace.toString());
    }

    private static void queuedMessagesArriveBeforeTick(Check c) throws Exception {
        ManualClock clock = new ManualClock();
        Graph g = new Graph(clock);
        List<String> trace = new ArrayList<>();
        Recorder producer = new Recorder("producer", trace);
        producer.publishes = true;
        Recorder consumer = new Recorder("consumer", trace);
        g.add(producer, Rate.hz(10));
        g.add(consumer, Rate.hz(10));
        g.start();
        g.step(3);
        g.stop();

        // Same loop, not the next one: the consumer is declared after the producer,
        // so by the time its queues are drained the producer has already published.
        // Declaration order is worth a full cycle of latency, which is why it is
        // insertion order rather than something the scheduler is free to choose.
        c.eq("delivery: consumer saw the publisher", "[0, 1, 2]", consumer.received.toString());
    }

    /** The mirror image: declared before the producer, the consumer waits a cycle. */
    private static void orderDecidesLatency(Check c) throws Exception {
        ManualClock clock = new ManualClock();
        Graph g = new Graph(clock);
        List<String> trace = new ArrayList<>();
        Recorder consumer = new Recorder("consumer", trace);
        Recorder producer = new Recorder("producer", trace);
        producer.publishes = true;
        g.add(consumer, Rate.hz(10));   // declared first, so it ticks first
        g.add(producer, Rate.hz(10));
        g.start();
        g.step(3);
        g.stop();

        // One loop behind: it drained before the producer had published.
        c.eq("latency: wrong order costs a cycle", "[0, 1]", consumer.received.toString());
    }

    private static void aFailingNodeDoesNotStopTheGraph(Check c) throws Exception {
        ManualClock clock = new ManualClock();
        Graph g = new Graph(clock);
        List<String> trace = new ArrayList<>();
        Recorder bad = new Recorder("bad", trace);
        bad.throwsOnTick = true;
        Recorder good = new Recorder("good", trace);
        g.add(bad, Rate.hz(10));
        g.add(good, Rate.hz(10));

        // A throwing node would otherwise spam the console through the default handler.
        Thread.currentThread().setUncaughtExceptionHandler((t, e) -> { });
        g.start();
        g.step(5);
        g.stop();
        Thread.currentThread().setUncaughtExceptionHandler(null);

        // One bad driver must not take the robot down with it.
        c.eq("isolation: healthy node kept running", 5, good.ticksAt.size());
        c.eq("isolation: errors counted", 5L, g.nodes().get(0).errors);
        c.eq("isolation: failed ticks not counted as work", 0L, g.nodes().get(0).ticks);
    }

    private static void stopUnsubscribes(Check c) throws Exception {
        ManualClock clock = new ManualClock();
        Graph g = new Graph(clock);
        List<String> trace = new ArrayList<>();
        Recorder r = new Recorder("r", trace);
        g.add(r, Rate.hz(10));
        g.start();
        g.step(1);
        g.stop();

        // After stop the listener must be detached, or a restarted graph delivers
        // to both the old and new instances.
        g.bus().publish(TICKS, 99);
        c.eq("stop: listener detached", 0, r.received.size());
    }

    private static void duplicateNamesRejected(Check c) {
        Graph g = new Graph(new ManualClock());
        List<String> trace = new ArrayList<>();
        g.add(new Recorder("dup", trace), Rate.hz(10));
        try {
            g.add(new Recorder("dup", trace), Rate.hz(10));
            c.fail("names: duplicate should be rejected");
        } catch (IllegalArgumentException e) {
            c.pass("names: duplicate rejected");
        }
    }

    /**
     * A clock whose sleeps come back slightly short, as a real one does.
     *
     * <p>{@link ManualClock} advances by exactly what is asked, so it cannot
     * reproduce scheduler drift. This one wakes 0.5% early — far less jitter than
     * an actual phone under load — which was enough to make a 10 Hz node tick at
     * 6 Hz before the deadlines were advanced in whole periods.
     */
    private static final class JitteryClock implements com.antu.core.time.Clock {
        private final ManualClock inner = new ManualClock();

        @Override public com.antu.core.time.Stamp now() {
            return inner.now();
        }

        @Override public void sleepNanos(long nanos) {
            inner.advanceNanos(Math.max(1, (long) (nanos * 0.995)));
        }
    }

    private static void jitterDoesNotDropTicks(Check c) throws Exception {
        JitteryClock clock = new JitteryClock();
        Graph g = new Graph(clock);
        List<String> trace = new ArrayList<>();
        Recorder n = new Recorder("same-rate", trace);
        g.add(n, Rate.hz(10));          // the only node, so it runs at the loop rate
        g.start();
        g.step(50);
        g.stop();

        // Every loop, not two out of three.
        c.eq("jitter: ticks every loop", 50, n.ticksAt.size());
        c.eq("jitter: nothing recorded as missed", 0L, g.nodes().get(0).missed);
    }

    private static void aSlowNodeGivesUpSlotsRatherThanBursting(Check c) throws Exception {
        ManualClock clock = new ManualClock();
        Graph g = new Graph(clock);
        List<String> trace = new ArrayList<>();
        Recorder fast = new Recorder("fast", trace);
        Recorder slow = new Recorder("slow", trace);
        slow.burnClock = clock;
        slow.overrunMillis = 500;       // overruns its 100 ms budget fivefold
        g.add(fast, Rate.hz(10));
        g.add(slow, Rate.hz(10));
        g.start();
        g.step(10);
        g.stop();

        // A node that cannot keep up must drop the slots it missed, not fire a
        // burst of catch-up ticks at a robot with stale sensor data.
        c.eq("overrun: no catch-up burst", true, slow.ticksAt.size() <= 10);
        c.eq("overrun: missed slots counted", true, g.nodes().get(1).missed > 0);
    }

    /**
     * The architectural bet: same inputs, same execution, every run. Without this
     * a recording is a rough guide rather than a reproduction, and intermittent
     * robot bugs stay intermittent.
     */
    private static void replayIsDeterministic(Check c) throws Exception {
        String first = runOnce();
        String second = runOnce();
        String third = runOnce();
        c.eq("determinism: run 2 matches run 1", first, second);
        c.eq("determinism: run 3 matches run 1", first, third);
        c.eq("determinism: trace is non-trivial", true, first.length() > 100);
    }

    private static String runOnce() throws Exception {
        ManualClock clock = new ManualClock();
        Graph g = new Graph(clock);
        List<String> trace = new ArrayList<>();
        Recorder imu = new Recorder("imu", trace);
        imu.publishes = true;
        Recorder filter = new Recorder("filter", trace);
        Recorder planner = new Recorder("planner", trace);
        g.add(imu, Rate.hz(200));
        g.add(filter, Rate.hz(50));
        g.add(planner, Rate.hz(5));
        g.start();
        g.step(200);
        g.stop();
        return trace.toString() + filter.received.size() + "/" + planner.received.size();
    }
}
