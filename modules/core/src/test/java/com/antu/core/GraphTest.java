package com.antu.core;

import com.antu.core.graph.Graph;
import com.antu.core.graph.In;
import com.antu.core.graph.Message;
import com.antu.core.graph.Out;
import com.antu.core.node.Node;
import com.antu.core.time.Clock;
import com.antu.core.time.ManualClock;
import com.antu.core.time.Rate;

import java.util.ArrayList;
import java.util.List;

/**
 * The static graph, and the guarantees it is supposed to buy.
 *
 * <p>Type mismatches are absent from this file on purpose: {@code connect} is
 * generic, so wiring an {@code Out<Integer>} to an {@code In<String>} does not
 * compile and cannot be asserted at runtime. That is the point of the design —
 * the errors these tests do cover are the ones a type cannot express.
 */
public final class GraphTest {

    public static void main(String[] args) throws Exception {
        Check c = new Check("GraphTest");

        dataCrossesTheGraphInOneTick(c);
        orderIgnoresInsertion(c);
        missingRequiredInputFails(c);
        optionalInputMayDangle(c);
        twoWritersFail(c);
        directCycleFails(c);
        delayedEdgeBreaksTheCycle(c);
        autoconnectMatchesNameAndType(c);
        autoconnectRefusesAmbiguity(c);
        everyProblemIsReportedAtOnce(c);
        ratesDivide(c);
        freshnessAndDrain(c);
        aFailingNodeIsIsolated(c);
        determinism(c);
        everyOutputGetsAChannel(c);

        c.finish();
    }

    // ---------- fixtures ----------

    /** Publishes an incrementing counter. */
    private static final class Source extends Node {
        final Out<Integer> value = out("value", Integer.class);
        final List<String> trace;
        int n;

        Source(String name, List<String> trace) {
            super(name);
            this.trace = trace;
        }

        @Override public void tick(Context ctx) {
            trace.add(name());
            value.publish(++n);
        }
    }

    /** Reads a counter and republishes it, so chains can be built. */
    private static final class Relay extends Node {
        final In<Integer> input = in("value", Integer.class, 0);
        final Out<Integer> output = out("relayed", Integer.class);
        final List<String> trace;
        final List<Integer> seen = new ArrayList<>();
        boolean throwsOnTick;
        boolean throwsOnStart;

        Relay(String name, List<String> trace) {
            super(name);
            this.trace = trace;
        }

        @Override public void start(Context ctx) {
            if (throwsOnStart) {
                throw new IllegalStateException("deliberate start failure");
            }
        }

        @Override public void tick(Context ctx) {
            trace.add(name());
            if (throwsOnTick) {
                throw new IllegalStateException("deliberate");
            }
            seen.add(input.get());
            output.publish(input.get());
        }
    }

    /** Terminal consumer. */
    private static final class Sink extends Node {
        final In<Integer> input = in("relayed", Integer.class, 0);
        final List<String> trace;
        final List<Integer> seen = new ArrayList<>();
        final List<Integer> drained = new ArrayList<>();
        boolean useDrain;

        Sink(String name, List<String> trace) {
            super(name);
            this.trace = trace;
        }

        @Override public void tick(Context ctx) {
            trace.add(name());
            if (useDrain) {
                for (Message<Integer> m : input.drain()) {
                    drained.add(m.payload());
                }
            }
            seen.add(input.get());
        }
    }

    // ---------- ordering ----------

    /**
     * A value published by the first node reaches the last within the same tick.
     * With declaration-order scheduling this took three ticks, and nothing said so.
     */
    private static void dataCrossesTheGraphInOneTick(Check c) throws Exception {
        List<String> trace = new ArrayList<>();
        Source source = new Source("source", trace);
        Relay relay = new Relay("relay", trace);
        Sink sink = new Sink("sink", trace);

        Graph g = Graph.builder(new ManualClock())
                .add(sink, Rate.hz(10))          // added first, on purpose
                .add(relay, Rate.hz(10))
                .add(source, Rate.hz(10))
                .connect(source.value, relay.input)
                .connect(relay.output, sink.input)
                .build();
        g.start();
        g.step(1);
        g.stop();

        c.eq("one tick: source ran first", "[source, relay, sink]", trace.toString());
        c.eq("one tick: value reached the end", "[1]", sink.seen.toString());
    }

    private static void orderIgnoresInsertion(Check c) throws Exception {
        List<String> a = new ArrayList<>();
        List<String> b = new ArrayList<>();
        c.eq("order: independent of insertion", run(a, true), run(b, false));
    }

    private static String run(List<String> trace, boolean sourceFirst) throws Exception {
        Source source = new Source("source", trace);
        Relay relay = new Relay("relay", trace);
        Graph.Builder builder = Graph.builder(new ManualClock());
        if (sourceFirst) {
            builder.add(source, Rate.hz(10)).add(relay, Rate.hz(10));
        } else {
            builder.add(relay, Rate.hz(10)).add(source, Rate.hz(10));
        }
        Graph g = builder.connect(source.value, relay.input).build();
        g.start();
        g.step(2);
        g.stop();
        return trace.toString();
    }

    // ---------- validation ----------

    private static void missingRequiredInputFails(Check c) {
        List<String> trace = new ArrayList<>();
        try {
            Graph.builder(new ManualClock())
                    .add(new Relay("relay", trace), Rate.hz(10))
                    .build();
            c.fail("missing input: should have failed");
        } catch (IllegalStateException e) {
            // A graph missing a wire must not run: a node that silently never
            // receives anything looks exactly like a broken sensor.
            c.eq("missing input: names the port", true, e.getMessage().contains("relay.value"));
        }
    }

    private static void optionalInputMayDangle(Check c) throws Exception {
        List<String> trace = new ArrayList<>();
        Relay relay = new Relay("relay", trace);
        relay.input.optional();
        Graph g = Graph.builder(new ManualClock()).add(relay, Rate.hz(10)).build();
        g.start();
        g.step(1);
        g.stop();
        c.eq("optional input: falls back", "[0]", relay.seen.toString());
    }

    private static void twoWritersFail(Check c) {
        List<String> trace = new ArrayList<>();
        Source a = new Source("a", trace);
        Source b = new Source("b", trace);
        Relay relay = new Relay("relay", trace);
        try {
            Graph.builder(new ManualClock())
                    .add(a, Rate.hz(10)).add(b, Rate.hz(10)).add(relay, Rate.hz(10))
                    .connect(a.value, relay.input)
                    .connect(b.value, relay.input)
                    .build();
            c.fail("two writers: should have failed");
        } catch (IllegalStateException e) {
            // Teleop and a planner both driving cmd_vel makes a robot stutter
            // between two commands. Caught before it moves, not warned about.
            c.eq("two writers: reports both", true,
                    e.getMessage().contains("a.value") && e.getMessage().contains("b.value"));
        }
    }

    // ---------- cycles ----------

    private static void directCycleFails(Check c) {
        List<String> trace = new ArrayList<>();
        Relay one = new Relay("one", trace);
        Relay two = new Relay("two", trace);
        try {
            Graph.builder(new ManualClock())
                    .add(one, Rate.hz(10)).add(two, Rate.hz(10))
                    .connect(one.output, two.input)
                    .connect(two.output, one.input)
                    .build();
            c.fail("cycle: should have failed");
        } catch (IllegalStateException e) {
            c.eq("cycle: names the nodes", true,
                    e.getMessage().contains("one") && e.getMessage().contains("two"));
            c.eq("cycle: suggests the fix", true,
                    e.getMessage().contains("connectDelayed"));
        }
    }

    /** A control loop is inherently cyclic; the feedback edge makes it schedulable. */
    private static void delayedEdgeBreaksTheCycle(Check c) throws Exception {
        List<String> trace = new ArrayList<>();
        Relay controller = new Relay("controller", trace);
        Relay plant = new Relay("plant", trace);

        Graph g = Graph.builder(new ManualClock())
                .add(controller, Rate.hz(10)).add(plant, Rate.hz(10))
                .connect(controller.output, plant.input)
                .connectDelayed(plant.output, controller.input)   // feedback
                .build();
        g.start();
        g.step(3);
        g.stop();

        c.eq("feedback: controller runs before the plant", "controller", trace.get(0));
        c.eq("feedback: loop actually ran", 3, plant.seen.size());
    }

    // ---------- autoconnect ----------

    private static void autoconnectMatchesNameAndType(Check c) throws Exception {
        List<String> trace = new ArrayList<>();
        Source source = new Source("source", trace);
        Relay relay = new Relay("relay", trace);      // its input is also called "value"

        Graph g = Graph.builder(new ManualClock())
                .add(source, Rate.hz(10)).add(relay, Rate.hz(10))
                .autoconnect()
                .build();
        g.start();
        g.step(1);
        g.stop();
        c.eq("autoconnect: wired by name and type", "[1]", relay.seen.toString());
    }

    private static void autoconnectRefusesAmbiguity(Check c) {
        List<String> trace = new ArrayList<>();
        try {
            Graph.builder(new ManualClock())
                    .add(new Source("a", trace), Rate.hz(10))
                    .add(new Source("b", trace), Rate.hz(10))
                    .add(new Relay("relay", trace), Rate.hz(10))
                    .autoconnect()
                    .build();
            c.fail("ambiguous autoconnect: should have failed");
        } catch (IllegalStateException e) {
            // Guessing which of two sources feeds an input is exactly the kind of
            // silent choice that makes a system unpredictable.
            c.eq("ambiguous autoconnect: says so", true,
                    e.getMessage().contains("ambiguous"));
        }
    }

    private static void everyProblemIsReportedAtOnce(Check c) {
        List<String> trace = new ArrayList<>();
        Source a = new Source("a", trace);
        Source b = new Source("b", trace);
        Relay relay = new Relay("relay", trace);
        Sink sink = new Sink("sink", trace);
        try {
            Graph.builder(new ManualClock())
                    .add(a, Rate.hz(10)).add(b, Rate.hz(10))
                    .add(relay, Rate.hz(10)).add(sink, Rate.hz(10))
                    .connect(a.value, relay.input)
                    .connect(b.value, relay.input)      // two writers
                    .build();                            // and sink.relayed unwired
            c.fail("multiple problems: should have failed");
        } catch (IllegalStateException e) {
            // Fixing a half-wired graph one build at a time is miserable.
            c.eq("multiple problems: reports both", true,
                    e.getMessage().contains("writers") && e.getMessage().contains("sink.relayed"));
        }
    }

    // ---------- scheduling ----------

    private static void ratesDivide(Check c) throws Exception {
        List<String> trace = new ArrayList<>();
        Source fast = new Source("fast", trace);
        Relay slow = new Relay("slow", trace);
        Graph g = Graph.builder(new ManualClock())
                .add(fast, Rate.hz(100)).add(slow, Rate.hz(10))
                .connect(fast.value, slow.input)
                .build();
        g.start();
        g.step(100);
        g.stop();
        c.eq("rates: fast ticked every loop", 100, fast.n);
        c.eq("rates: slow ticked a tenth as often", 10, slow.seen.size());
    }

    private static void freshnessAndDrain(Check c) throws Exception {
        List<String> trace = new ArrayList<>();
        Source source = new Source("source", trace);
        Relay relay = new Relay("relay", trace);
        Sink sink = new Sink("sink", trace);
        sink.useDrain = true;

        Graph g = Graph.builder(new ManualClock())
                .add(source, Rate.hz(100)).add(relay, Rate.hz(100)).add(sink, Rate.hz(20))
                .connect(source.value, relay.input)
                .connect(relay.output, sink.input)
                .build();
        g.start();
        g.step(20);
        g.stop();

        // get() gives the latest and skips; drain() loses nothing. A node
        // integrating distance must use drain or it under-reports.
        //
        // The sink ticks at loops 0, 5, 10 and 15, so it has drained 16 of the 20
        // published; the last four are still queued, not dropped. That distinction
        // is the whole point, so assert it rather than the total.
        c.eq("drain: nothing dropped", 0L, sink.input.dropped());
        c.eq("drain: everything up to the last tick", 16, sink.drained.size());
        c.eq("get: only the latest per tick", 4, sink.seen.size());

        // And the tail is still there to be collected.
        c.eq("drain: the rest is queued, not lost", 4, sink.input.drain().size());
    }

    private static void aFailingNodeIsIsolated(Check c) throws Exception {
        List<String> trace = new ArrayList<>();
        Source source = new Source("source", trace);
        Relay bad = new Relay("bad", trace);
        bad.throwsOnTick = true;
        Relay good = new Relay("good", trace);

        com.antu.core.log.Log.setSink(com.antu.core.log.Log.NONE);
        Graph g = Graph.builder(new ManualClock())
                .add(source, Rate.hz(10)).add(bad, Rate.hz(10)).add(good, Rate.hz(10))
                .connect(source.value, bad.input)
                .connect(source.value, good.input)
                .build();
        g.start();
        g.step(5);
        g.stop();
        com.antu.core.log.Log.setSink(com.antu.core.log.Log.CONSOLE);

        c.eq("isolation: healthy node kept running", 5, good.seen.size());
        c.eq("isolation: errors counted", 5L, g.nodes().get(1).errors);
    }

    private static void determinism(Check c) throws Exception {
        c.eq("determinism: run 2 matches run 1", once(), once());
        c.eq("determinism: run 3 matches run 1", once(), once());
    }

    /**
     * Telemetry depends on this: sonar and battery usually have no consumer in the
     * graph and are still exactly what an operator needs to watch.
     */
    private static void everyOutputGetsAChannel(Check c) throws Exception {
        List<String> trace = new ArrayList<>();
        Source source = new Source("source", trace);
        Relay relay = new Relay("relay", trace);

        Graph g = Graph.builder(new ManualClock())
                .add(source, Rate.hz(10)).add(relay, Rate.hz(10))
                .connect(source.value, relay.input)
                .build();
        g.start();
        g.step(1);
        g.stop();

        c.eq("channels: one per output, read or not", 2, g.channels().size());
        c.eq("channels: the unread one exists", true, g.channel("relay.relayed") != null);
        c.eq("channels: and carries data", 1L, g.channel("relay.relayed").published());
        c.eq("channels: readers counted", 1, g.channel("source.value").readerCount());
        c.eq("channels: unread has none", 0, g.channel("relay.relayed").readerCount());
    }

    private static String once() throws Exception {
        List<String> trace = new ArrayList<>();
        Source source = new Source("source", trace);
        Relay relay = new Relay("relay", trace);
        Sink sink = new Sink("sink", trace);
        Graph g = Graph.builder(new ManualClock())
                .add(source, Rate.hz(200)).add(relay, Rate.hz(50)).add(sink, Rate.hz(5))
                .connect(source.value, relay.input)
                .connect(relay.output, sink.input)
                .build();
        g.start();
        g.step(200);
        g.stop();
        return trace.toString() + relay.seen.size() + "/" + sink.seen.size();
    }
}
