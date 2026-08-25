package com.antu.core.exec;

import com.antu.core.bus.Bus;
import com.antu.core.bus.Subscription;
import com.antu.core.node.Node;
import com.antu.core.time.Clock;
import com.antu.core.time.Rate;
import com.antu.core.time.Stamp;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * The set of nodes that make up a robot, and the schedule they run on.
 *
 * <p>Nodes are added with a rate and executed by a single tick loop in the order
 * they were added. That ordering is the whole point: with a manual clock and a
 * recording, the same inputs produce the same sequence of calls, every run. A
 * pool of threads racing to service callbacks would be marginally faster and
 * impossible to debug.
 *
 * <p>The loop runs at the fastest declared rate, and each node ticks when its own
 * period has elapsed. A 200 Hz IMU filter and a 5 Hz planner therefore share one
 * thread with no coordination, and a phone spends its cores on perception rather
 * than on context switches.
 *
 * <p>Nodes that must block — waiting on a camera frame or a serial read — own
 * their thread and hand results over through the bus. The tick loop never blocks.
 */
public final class Graph {

    private final Bus bus;
    private final Clock clock;
    private final Map<String, Entry> entries = new LinkedHashMap<>();
    private final List<String> order = new ArrayList<>();

    private volatile boolean running;
    private volatile boolean started;
    private Thread thread;
    private long loopPeriodNanos = Long.MAX_VALUE;
    private volatile long loopCount;
    private volatile long overruns;

    public Graph(Clock clock) {
        this.clock = clock;
        this.bus = new Bus(clock);
    }

    public Bus bus() {
        return bus;
    }

    public Clock clock() {
        return clock;
    }

    /**
     * Adds a node to run at {@code rate}. Execution order is insertion order, so
     * putting a filter before the planner that consumes it saves a full cycle of
     * latency.
     */
    public Graph add(Node node, Rate rate) {
        if (running) {
            throw new IllegalStateException("cannot add " + node.name() + " to a running graph");
        }
        if (entries.containsKey(node.name())) {
            throw new IllegalArgumentException("duplicate node name: " + node.name());
        }
        entries.put(node.name(), new Entry(node, rate));
        order.add(node.name());
        loopPeriodNanos = Math.min(loopPeriodNanos, rate.periodNanos());
        return this;
    }

    /**
     * Starts every node, but does not begin ticking.
     *
     * <p>Deliberately separate from {@link #spin}: a graph that both spawned a
     * loop thread and allowed {@link #step} would tick from two places at once,
     * which is exactly as confusing as it sounds.
     */
    public void start() throws Exception {
        if (started) {
            return;
        }
        if (entries.isEmpty()) {
            throw new IllegalStateException("graph has no nodes");
        }
        long now = clock.now().nanos();
        for (String name : order) {
            Entry e = entries.get(name);
            // Due immediately, so every node gets one tick before any waiting.
            e.nextDueNanos = now;
            e.lastTickNanos = now;
            e.node.start(e.context);
            e.started = true;
        }
        started = true;
    }

    /**
     * Starts if needed, then runs the tick loop on its own thread. This is what a
     * live robot calls; tests and replay use {@link #step} instead.
     */
    public void spin() throws Exception {
        start();
        if (running) {
            return;
        }
        running = true;
        thread = new Thread(this::loop, "antu-graph");
        thread.setDaemon(true);
        thread.start();
    }

    /** Stops the loop and every started node, in reverse order. */
    public void stop() {
        running = false;
        Thread t = thread;
        thread = null;
        if (t != null) {
            try {
                t.join(2000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        started = false;
        List<String> reversed = new ArrayList<>(order);
        Collections.reverse(reversed);
        for (String name : reversed) {
            Entry e = entries.get(name);
            if (e.started) {
                e.started = false;
                try {
                    e.node.stop();
                } catch (Throwable ex) {
                    report(e, ex);
                } finally {
                    // Even a node that threw must not leave listeners attached.
                    e.context.closeAll();
                }
            }
        }
    }

    public boolean isRunning() {
        return running;
    }

    /** Loop iterations completed. */
    public long loopCount() {
        return loopCount;
    }

    /** Iterations that took longer than the loop period. */
    public long overruns() {
        return overruns;
    }

    /**
     * Runs exactly {@code iterations} of the loop on the calling thread, without
     * starting a background thread or sleeping.
     *
     * <p>This is how tests and replay drive a graph: paired with a
     * {@link com.antu.core.time.ManualClock}, a run is fully determined by its
     * inputs, so a failure reproduces exactly rather than one time in twenty.
     */
    public void step(int iterations) throws Exception {
        if (running) {
            throw new IllegalStateException(
                    "graph is spinning; step() and spin() cannot both drive it");
        }
        if (!started) {
            throw new IllegalStateException("call start() before step()");
        }
        for (int i = 0; i < iterations; i++) {
            long at = clock.now().nanos();
            tickDue(at);
            loopCount++;
            // Work may have consumed the whole period already, so never ask the
            // clock to move backwards.
            long remaining = loopPeriodNanos - (clock.now().nanos() - at);
            if (remaining > 0) {
                clock.sleepNanos(remaining);
            }
        }
    }

    private void loop() {
        long next = clock.now().nanos();
        while (running) {
            long now = clock.now().nanos();
            tickDue(now);
            loopCount++;

            next += loopPeriodNanos;
            long sleep = next - clock.now().nanos();
            if (sleep <= 0) {
                // Behind schedule. Give up the missed slots rather than sprinting
                // to catch up, which on a phone means a thermal spike and a robot
                // acting on stale sensor data.
                overruns++;
                next = clock.now().nanos();
                continue;
            }
            try {
                clock.sleepNanos(sleep);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }

    /** Ticks every node that is due, in declared order. */
    private void tickDue(long nowNanos) {
        for (String name : order) {
            Entry e = entries.get(name);
            // Tick if we are nearer the deadline than the next loop would be.
            // Without this slack an early wake-up of a few microseconds defers the
            // node by an entire loop, and a node running at the loop rate ends up
            // firing on alternate loops: a 10 Hz node observed at 6 Hz on a phone.
            if (nowNanos + loopPeriodNanos / 2 < e.nextDueNanos) {
                continue;
            }
            // Advance the deadline by whole periods rather than re-anchoring it to
            // now. Re-anchoring accumulates every wake-up's jitter, so a node
            // running at the loop rate drifts out of phase and starts skipping
            // loops entirely: 10 Hz becomes an erratic 6 Hz.
            e.nextDueNanos += e.rate.periodNanos();
            if (e.nextDueNanos <= nowNanos) {
                // Genuinely behind, not jitter. Give up the missed slots instead of
                // firing a burst of catch-up ticks at a robot.
                e.missed += (nowNanos - e.nextDueNanos) / e.rate.periodNanos() + 1;
                e.nextDueNanos = nowNanos + e.rate.periodNanos();
            }
            e.lastTickNanos = nowNanos;
            // Deliver what arrived since last time, then let the node act on it.
            for (Subscription s : e.context.queued) {
                bus.drain(s);
            }
            try {
                e.node.tick(e.context);
                e.ticks++;
            } catch (Throwable t) {
                e.errors++;
                report(e, t);
            }
        }
    }

    private void report(Entry e, Throwable t) {
        Thread current = Thread.currentThread();
        current.getUncaughtExceptionHandler().uncaughtException(current,
                new RuntimeException("node " + e.node.name() + " failed", t));
    }

    /** Per-node counters, for the diagnostics page. */
    public List<NodeInfo> nodes() {
        List<NodeInfo> out = new ArrayList<>();
        for (String name : order) {
            Entry e = entries.get(name);
            out.add(new NodeInfo(name, e.rate, e.ticks, e.errors, e.missed, e.started));
        }
        return out;
    }

    public static final class NodeInfo {
        public final String name;
        public final Rate rate;
        public final long ticks;
        public final long errors;
        /** Slots skipped because the node could not keep up with its own rate. */
        public final long missed;
        public final boolean started;

        NodeInfo(String name, Rate rate, long ticks, long errors, long missed, boolean started) {
            this.name = name;
            this.rate = rate;
            this.ticks = ticks;
            this.errors = errors;
            this.missed = missed;
            this.started = started;
        }

        @Override public String toString() {
            return String.format("%-16s %-8s ticks=%-8d missed=%-6d errors=%d",
                    name, rate, ticks, missed, errors);
        }
    }

    private final class Entry {
        final Node node;
        final Rate rate;
        final Ctx context;
        boolean started;
        long nextDueNanos;
        long lastTickNanos;
        long ticks;
        long errors;
        long missed;

        Entry(Node node, Rate rate) {
            this.node = node;
            this.rate = rate;
            this.context = new Ctx(node.name(), this);
        }
    }

    private final class Ctx implements Node.Context {
        final String name;
        final Entry entry;
        /** Every subscription this node made, closed when it stops. */
        final List<Subscription> subscriptions = new ArrayList<>();
        /** The queued subset, drained before each tick. */
        final List<Subscription> queued = new ArrayList<>();

        Ctx(String name, Entry entry) {
            this.name = name;
            this.entry = entry;
        }

        @Override public Bus bus() {
            return Graph.this.bus;
        }

        @Override public Clock clock() {
            return clock;
        }

        @Override public String nodeName() {
            return name;
        }

        @Override public long tickCount() {
            return entry.ticks;
        }

        @Override public <T> Subscription subscribe(com.antu.core.bus.Topic<T> topic,
                                                    com.antu.core.bus.Listener<T> listener) {
            return subscribe(topic, listener, Bus.Delivery.QUEUED);
        }

        @Override public <T> Subscription subscribe(com.antu.core.bus.Topic<T> topic,
                                                    com.antu.core.bus.Listener<T> listener,
                                                    Bus.Delivery delivery) {
            Subscription s = Graph.this.bus.subscribe(topic, listener, delivery);
            subscriptions.add(s);
            if (delivery == Bus.Delivery.QUEUED) {
                queued.add(s);
            }
            return s;
        }

        @Override public <T> void publish(com.antu.core.bus.Topic<T> topic, T payload) {
            Graph.this.bus.publish(topic, payload);
        }

        @Override public <T> void publish(com.antu.core.bus.Topic<T> topic, T payload,
                                          Stamp stamp) {
            Graph.this.bus.publish(topic, payload, stamp);
        }

        void closeAll() {
            for (Subscription s : subscriptions) {
                s.close();
            }
            subscriptions.clear();
            queued.clear();
        }
    }
}
