package com.antu.core.graph;

import com.antu.core.log.Log;
import com.antu.core.node.Node;
import com.antu.core.time.Clock;
import com.antu.core.time.Rate;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * A fixed set of nodes, wired at build time and ticked in dataflow order.
 *
 * <p>The graph is decided before anything runs. {@link Builder#connect} is
 * generic, so wiring an {@code Out<Odometry>} to an {@code In<Twist2>} does not
 * compile; {@link Builder#build} then checks everything a type cannot express —
 * that required inputs are connected, that no input has two writers, and that the
 * direct edges contain no cycle.
 *
 * <h2>Order comes from the dataflow</h2>
 *
 * <p>Nodes are ticked in topological order, so a producer always runs before its
 * consumer and data crosses the whole graph within one tick. Nothing depends on
 * the order nodes were added, which in the previous design was load-bearing and
 * undocumented: adding a node in the wrong place quietly cost a cycle of latency.
 *
 * <h2>Feedback</h2>
 *
 * <p>Control loops are cyclic — odometry feeds a planner, which commands the
 * base, which produces odometry. A cycle of direct edges has no valid order and
 * is a build error naming the loop. Mark the feedback edge
 * {@linkplain Builder#connectDelayed delayed} and the consumer reads the previous
 * tick's value, which breaks the cycle for ordering and is what a control loop
 * means anyway.
 */
public final class Graph {

    private static final String TAG = "graph";

    private final Clock clock;
    private final List<Entry> order;
    private final Map<String, Channel<?>> channels;

    private volatile boolean running;
    private volatile boolean started;
    private Thread thread;
    private final long loopPeriodNanos;
    private volatile long loopCount;
    private volatile long overruns;

    private Graph(Clock clock, List<Entry> order, Map<String, Channel<?>> channels,
                  long loopPeriodNanos) {
        this.clock = clock;
        this.order = order;
        this.channels = channels;
        this.loopPeriodNanos = loopPeriodNanos;
    }

    public static Builder builder(Clock clock) {
        return new Builder(clock);
    }

    public Clock clock() {
        return clock;
    }

    /** Every channel, keyed by name. Fixed for the graph's lifetime. */
    public Map<String, Channel<?>> channels() {
        return Collections.unmodifiableMap(channels);
    }

    public Channel<?> channel(String name) {
        return channels.get(name);
    }

    // ---------- lifecycle ----------

    /** Starts every node without ticking. Pair with {@link #step} in tests. */
    public void start() throws Exception {
        if (started) {
            return;
        }
        long now = clock.now().nanos();
        int ok = 0;
        for (Entry e : order) {
            e.nextDueNanos = now;
            try {
                e.node.start(e.context);
                e.started = true;
                ok++;
            } catch (Throwable t) {
                // One node failing to start must not take the robot with it: an
                // ops server whose port is busy is no reason for the drive base to
                // stay dead. Loud, and visible in nodes().
                e.errors++;
                Log.e(TAG, "node " + e.node.name() + " failed to start; continuing without it", t);
            }
        }
        if (ok == 0) {
            throw new IllegalStateException("no node started successfully");
        }
        started = true;
    }

    /** Starts if needed, then ticks on a background thread. What a robot calls. */
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

    /**
     * Ticks {@code iterations} times on the calling thread, without sleeping in
     * real time. With a {@link com.antu.core.time.ManualClock} a run is fully
     * determined by its inputs, so a failure reproduces exactly.
     */
    public void step(int iterations) throws Exception {
        if (running) {
            throw new IllegalStateException("graph is spinning; step() and spin() cannot both drive it");
        }
        if (!started) {
            throw new IllegalStateException("call start() before step()");
        }
        for (int i = 0; i < iterations; i++) {
            long at = clock.now().nanos();
            tickDue(at);
            loopCount++;
            long remaining = loopPeriodNanos - (clock.now().nanos() - at);
            if (remaining > 0) {
                clock.sleepNanos(remaining);
            }
        }
    }

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
        List<Entry> reversed = new ArrayList<>(order);
        Collections.reverse(reversed);
        for (Entry e : reversed) {
            if (e.started) {
                e.started = false;
                try {
                    e.node.stop();
                } catch (Throwable t2) {
                    Log.e(TAG, "node " + e.node.name() + " failed to stop", t2);
                }
            }
        }
    }

    public boolean isRunning() {
        return running;
    }

    public long loopCount() {
        return loopCount;
    }

    public long overruns() {
        return overruns;
    }

    private void loop() {
        long next = clock.now().nanos();
        while (running) {
            tickDue(clock.now().nanos());
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

    /** Ticks every node that is due, in dataflow order. */
    private void tickDue(long nowNanos) {
        for (Entry e : order) {
            if (!e.started) {
                continue;
            }
            // Tick if we are nearer the deadline than the next loop would be.
            // Without this slack an early wake-up of a few microseconds defers the
            // node by an entire loop, and a node running at the loop rate fires on
            // alternate loops: a 10 Hz node observed at 6 Hz on a phone.
            if (nowNanos + loopPeriodNanos / 2 < e.nextDueNanos) {
                continue;
            }
            // Deadlines advance by whole periods rather than re-anchoring to now,
            // so jitter cannot accumulate.
            e.nextDueNanos += e.rate.periodNanos();
            if (e.nextDueNanos <= nowNanos) {
                e.missed += (nowNanos - e.nextDueNanos) / e.rate.periodNanos() + 1;
                e.nextDueNanos = nowNanos + e.rate.periodNanos();
            }
            try {
                e.node.tick(e.context);
                e.ticks++;
            } catch (Throwable t) {
                e.errors++;
                Log.e(TAG, "node " + e.node.name() + " failed", t);
            } finally {
                // Freshness means "arrived since the previous tick", so it is
                // cleared after the node has had its chance to look.
                for (Channel<?>.Sink sink : e.sinks) {
                    sink.clearFresh();
                }
            }
        }
    }

    /** Per-node counters, for the diagnostics page. */
    public List<NodeInfo> nodes() {
        List<NodeInfo> out = new ArrayList<>();
        for (Entry e : order) {
            out.add(new NodeInfo(e.node.name(), e.rate, e.ticks, e.errors, e.missed, e.started));
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

    private static final class Entry {
        final Node node;
        final Rate rate;
        final Node.Context context;
        final List<Channel<?>.Sink> sinks = new ArrayList<>();
        boolean started;
        long nextDueNanos;
        long ticks;
        long errors;
        long missed;

        Entry(Node node, Rate rate, Clock clock) {
            this.node = node;
            this.rate = rate;
            this.context = new Node.Context() {
                @Override public Clock clock() {
                    return clock;
                }

                @Override public String nodeName() {
                    return node.name();
                }

                @Override public long tickCount() {
                    return ticks;
                }
            };
        }
    }

    // ---------- building ----------

    /** Collects nodes and wiring, then validates the whole thing at once. */
    public static final class Builder {

        private final Clock clock;
        private final Map<String, Node> nodes = new LinkedHashMap<>();
        private final Map<String, Rate> rates = new LinkedHashMap<>();
        private final List<Edge> edges = new ArrayList<>();

        Builder(Clock clock) {
            this.clock = clock;
        }

        /** Adds a node to run at {@code rate}. */
        public Builder add(Node node, Rate rate) {
            if (nodes.containsKey(node.name())) {
                throw new IllegalArgumentException("duplicate node name: " + node.name());
            }
            nodes.put(node.name(), node);
            rates.put(node.name(), rate);
            return this;
        }

        /**
         * Wires an output to an input, delivered within the same tick.
         *
         * <p>Generic in {@code T}: connecting mismatched types is a compile error,
         * not something a robot discovers.
         */
        public <T> Builder connect(Out<T> from, In<T> to) {
            edges.add(new Edge(from, to, false));
            return this;
        }

        /**
         * Wires a feedback edge: the consumer sees the previous tick's value.
         *
         * <p>Needed wherever a loop closes — a controller reading the odometry
         * produced by the base it commands. Delayed edges are excluded from the
         * ordering, which is what makes a cyclic control loop schedulable.
         */
        public <T> Builder connectDelayed(Out<T> from, In<T> to) {
            edges.add(new Edge(from, to, true));
            return this;
        }

        /**
         * Wires every unconnected input to an output of the same name and type.
         *
         * <p>Convenience for the common case where a producer's {@code odom} feeds
         * a consumer's {@code odom}. Ambiguity — two outputs offering the same
         * name and type — is an error rather than a guess.
         */
        public Builder autoconnect() {
            Map<String, Out<?>> byKey = new HashMap<>();
            Set<String> ambiguous = new HashSet<>();
            for (Node n : nodes.values()) {
                for (Port<?> p : n.ports()) {
                    if (!(p instanceof Out)) {
                        continue;
                    }
                    String key = p.name + ":" + p.type.getName();
                    if (byKey.put(key, (Out<?>) p) != null) {
                        ambiguous.add(key);
                    }
                }
            }
            for (Node n : nodes.values()) {
                for (Port<?> p : n.ports()) {
                    if (!(p instanceof In) || isConnected((In<?>) p)) {
                        continue;
                    }
                    String key = p.name + ":" + p.type.getName();
                    if (ambiguous.contains(key)) {
                        throw new IllegalStateException(
                                "autoconnect is ambiguous for " + p.path()
                                        + ": several outputs offer '" + p.name + "'");
                    }
                    Out<?> source = byKey.get(key);
                    if (source != null) {
                        edges.add(new Edge(source, (In<?>) p, false));
                    }
                }
            }
            return this;
        }

        private boolean isConnected(In<?> in) {
            for (Edge e : edges) {
                if (e.to == in) {
                    return true;
                }
            }
            return false;
        }

        /** Applies a reusable piece of graph. See {@link Blueprint}. */
        public Builder apply(Blueprint blueprint) {
            blueprint.build(this);
            return this;
        }

        /**
         * Validates and constructs the graph.
         *
         * @throws IllegalStateException listing every problem found, rather than
         *         only the first: a half-wired graph usually has several, and
         *         fixing them one build at a time is miserable.
         */
        public Graph build() {
            List<String> problems = new ArrayList<>();

            // One writer per input. Two nodes driving the same input is the
            // classic way to get a robot that stutters between two commands, and
            // it is worth catching before it moves rather than warning about.
            Map<In<?>, List<Out<?>>> writers = new LinkedHashMap<>();
            for (Edge e : edges) {
                writers.computeIfAbsent(e.to, k -> new ArrayList<>()).add(e.from);
            }
            for (Map.Entry<In<?>, List<Out<?>>> entry : writers.entrySet()) {
                if (entry.getValue().size() > 1) {
                    StringBuilder sb = new StringBuilder();
                    for (Out<?> o : entry.getValue()) {
                        sb.append(sb.length() == 0 ? "" : ", ").append(o.path());
                    }
                    problems.add(entry.getKey().path() + " has " + entry.getValue().size()
                            + " writers: " + sb);
                }
            }

            for (Node n : nodes.values()) {
                for (Port<?> p : n.ports()) {
                    if (p instanceof In && ((In<?>) p).isRequired()
                            && !writers.containsKey(p)) {
                        problems.add(p.path() + " is required but nothing is connected to it");
                    }
                }
            }

            for (Edge e : edges) {
                if (!nodes.containsKey(e.from.owner())) {
                    problems.add("edge from " + e.from.path() + ", whose node is not in the graph");
                }
                if (!nodes.containsKey(e.to.owner())) {
                    problems.add("edge to " + e.to.path() + ", whose node is not in the graph");
                }
            }

            List<String> sorted = problems.isEmpty() ? topologicalOrder(problems) : null;

            if (!problems.isEmpty()) {
                StringBuilder sb = new StringBuilder("graph is not valid:");
                for (String p : problems) {
                    sb.append("\n  - ").append(p);
                }
                throw new IllegalStateException(sb.toString());
            }

            // Wiring is valid; create the channels and attach the ports.
            //
            // Every output gets a channel, whether or not a node reads it. A
            // channel is the output's identity, not a consequence of someone
            // consuming it: sonar and battery usually have no reader in the graph
            // and are still exactly what an operator needs to see. Making the
            // channel conditional on a reader made them invisible to telemetry.
            Map<String, Channel<?>> channels = new LinkedHashMap<>();
            Map<Out<?>, Channel<?>> byOut = new HashMap<>();
            Map<String, List<Channel<?>.Sink>> sinksByNode = new HashMap<>();

            for (Node n : nodes.values()) {
                for (Port<?> p : n.ports()) {
                    if (p instanceof Out) {
                        Channel<?> channel = newChannel((Out<?>) p, clock);
                        byOut.put((Out<?>) p, channel);
                        channels.put(channel.name(), channel);
                    }
                }
            }

            for (Edge e : edges) {
                Channel<?> channel = byOut.get(e.from);
                Channel<?>.Sink sink = attach(channel, e.to);
                sinksByNode.computeIfAbsent(e.to.owner(), k -> new ArrayList<>()).add(sink);
            }

            long loopPeriod = Long.MAX_VALUE;
            List<Entry> order = new ArrayList<>();
            for (String name : sorted) {
                Node node = nodes.get(name);
                Rate rate = rates.get(name);
                Entry entry = new Entry(node, rate, clock);
                List<Channel<?>.Sink> sinks = sinksByNode.get(name);
                if (sinks != null) {
                    entry.sinks.addAll(sinks);
                }
                order.add(entry);
                loopPeriod = Math.min(loopPeriod, rate.periodNanos());
            }
            if (order.isEmpty()) {
                throw new IllegalStateException("graph has no nodes");
            }
            return new Graph(clock, order, channels, loopPeriod);
        }

        @SuppressWarnings("unchecked")
        private static <T> Channel<T> newChannel(Out<T> out, Clock clock) {
            Channel<T> channel = new Channel<>(out.path(), out.type, clock);
            out.attach(channel);
            return channel;
        }

        @SuppressWarnings("unchecked")
        private static <T> Channel<T>.Sink attach(Channel<T> channel, In<?> in) {
            In<T> typed = (In<T>) in;
            Channel<T>.Sink sink = channel.newSink(typed.depth());
            typed.attach(sink);
            return sink;
        }

        /**
         * Kahn's algorithm over the direct edges. Reports a cycle by naming the
         * nodes still stuck, which is what you need to know to pick the feedback
         * edge to mark delayed.
         */
        private List<String> topologicalOrder(List<String> problems) {
            Map<String, Set<String>> dependsOn = new LinkedHashMap<>();
            for (String name : nodes.keySet()) {
                dependsOn.put(name, new LinkedHashSet<>());
            }
            for (Edge e : edges) {
                if (e.delayed) {
                    continue;                     // feedback: breaks the cycle
                }
                String producer = e.from.owner();
                String consumer = e.to.owner();
                if (!producer.equals(consumer)) {
                    dependsOn.get(consumer).add(producer);
                }
            }

            List<String> out = new ArrayList<>();
            Set<String> placed = new LinkedHashSet<>();
            boolean progress = true;
            while (progress && placed.size() < nodes.size()) {
                progress = false;
                for (String name : nodes.keySet()) {
                    if (placed.contains(name) || !placed.containsAll(dependsOn.get(name))) {
                        continue;
                    }
                    placed.add(name);
                    out.add(name);
                    progress = true;
                }
            }
            if (placed.size() < nodes.size()) {
                Set<String> stuck = new LinkedHashSet<>(nodes.keySet());
                stuck.removeAll(placed);
                problems.add("cycle among " + stuck
                        + " — mark the feedback edge connectDelayed() to break it");
                return null;
            }
            return out;
        }
    }

    /** A reusable piece of graph: a robot, a capability, a teleop stack. */
    public interface Blueprint {
        void build(Builder builder);
    }

    private static final class Edge {
        final Out<?> from;
        final In<?> to;
        final boolean delayed;

        Edge(Out<?> from, In<?> to, boolean delayed) {
            this.from = from;
            this.to = to;
            this.delayed = delayed;
        }
    }
}
