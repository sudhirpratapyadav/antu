package com.antu.core;

import com.antu.core.bus.Bus;
import com.antu.core.bus.Message;
import com.antu.core.bus.Subscription;
import com.antu.core.bus.Topic;
import com.antu.core.time.ManualClock;
import com.antu.core.time.Stamp;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicInteger;

/** Delivery, ordering, backpressure and type safety on the bus. */
public final class BusTest {

    private static final Topic<String> CHAT = Topic.of("/chat", String.class);
    private static final Topic<Integer> COUNT = Topic.of("/count", Integer.class);

    public static void main(String[] args) throws Exception {
        Check c = new Check("BusTest");

        queuedDeliveryWaitsForDrain(c);
        directDeliveryIsImmediate(c);
        latestValue(c);
        multipleSubscribers(c);
        unsubscribeStopsDelivery(c);
        fullQueueDropsOldest(c);
        stampsAreCarried(c);
        typeMismatchRejected(c);
        topicNameValidated(c);
        introspection(c);
        concurrentPublishersAreNotLost(c);

        c.finish();
    }

    private static void queuedDeliveryWaitsForDrain(Check c) {
        Bus bus = new Bus(new ManualClock());
        List<String> got = new ArrayList<>();
        Subscription sub = bus.subscribe(CHAT, m -> got.add(m.payload()));

        bus.publish(CHAT, "a");
        bus.publish(CHAT, "b");
        // Nothing is delivered until drained. This is what makes ordering the
        // scheduler's decision rather than the thread scheduler's.
        c.eq("queued: nothing before drain", 0, got.size());
        c.eq("queued: two pending", 2, sub.pending());

        c.eq("queued: drain reports count", 2, bus.drain(sub));
        c.eq("queued: both arrived", "[a, b]", got.toString());
        c.eq("queued: drained empty", 0, bus.drain(sub));
    }

    private static void directDeliveryIsImmediate(Check c) {
        Bus bus = new Bus(new ManualClock());
        List<String> got = new ArrayList<>();
        bus.subscribe(CHAT, m -> got.add(m.payload()), Bus.Delivery.DIRECT);

        bus.publish(CHAT, "now");
        c.eq("direct: delivered on publish", "[now]", got.toString());
    }

    private static void latestValue(Check c) {
        Bus bus = new Bus(new ManualClock());
        c.eq("latest: absent before publish", null, bus.latest(CHAT));
        c.eq("latest: fallback used", "none", bus.latestOr(CHAT, "none"));

        bus.publish(CHAT, "one");
        bus.publish(CHAT, "two");
        // Latest-value needs no subscriber, which is how state (pose, map, goal)
        // is read without replaying a stream.
        c.eq("latest: newest wins", "two", bus.latest(CHAT).payload());
        c.eq("latest: sequence counts", 2L, bus.latest(CHAT).sequence());
    }

    private static void multipleSubscribers(Check c) {
        Bus bus = new Bus(new ManualClock());
        List<String> a = new ArrayList<>();
        List<String> b = new ArrayList<>();
        Subscription sa = bus.subscribe(CHAT, m -> a.add(m.payload()));
        Subscription sb = bus.subscribe(CHAT, m -> b.add(m.payload()));

        bus.publish(CHAT, "x");
        bus.drain(sa);
        bus.drain(sb);
        c.eq("fanout: first got it", "[x]", a.toString());
        c.eq("fanout: second got it", "[x]", b.toString());
    }

    private static void unsubscribeStopsDelivery(Check c) {
        Bus bus = new Bus(new ManualClock());
        List<String> got = new ArrayList<>();
        Subscription sub = bus.subscribe(CHAT, m -> got.add(m.payload()));

        bus.publish(CHAT, "before");
        bus.drain(sub);
        sub.close();
        bus.publish(CHAT, "after");
        bus.drain(sub);

        c.eq("close: no delivery after", "[before]", got.toString());
        c.eq("close: twice is harmless", true, closeTwiceOk(sub));
    }

    private static boolean closeTwiceOk(Subscription sub) {
        try {
            sub.close();
            return true;
        } catch (Throwable t) {
            return false;
        }
    }

    private static void fullQueueDropsOldest(Check c) {
        Bus bus = new Bus(new ManualClock());
        List<Integer> got = new ArrayList<>();
        Subscription sub = bus.subscribe(COUNT, m -> got.add(m.payload()),
                Bus.Delivery.QUEUED, 3);

        for (int i = 1; i <= 5; i++) {
            bus.publish(COUNT, i);
        }
        bus.drain(sub);
        // Newest wins: a stale command is worse than a missing one.
        c.eq("backpressure: keeps newest", "[3, 4, 5]", got.toString());
        c.eq("backpressure: drops counted", 2L, sub.dropped());
    }

    private static void stampsAreCarried(Check c) {
        ManualClock clock = new ManualClock();
        Bus bus = new Bus(clock);
        List<Stamp> stamps = new ArrayList<>();
        Subscription sub = bus.subscribe(CHAT, m -> stamps.add(m.stamp()));

        clock.advanceMillis(100);
        bus.publish(CHAT, "auto");
        bus.publish(CHAT, "explicit", Stamp.ofMillis(5));
        bus.drain(sub);

        c.eq("stamp: clock used by default", 100L, stamps.get(0).millis());
        // A driver's own stamp must survive: fusing sensors depends on it.
        c.eq("stamp: explicit preserved", 5L, stamps.get(1).millis());
    }

    private static void typeMismatchRejected(Check c) {
        Bus bus = new Bus(new ManualClock());
        bus.publish(CHAT, "hello");
        Topic<Integer> clash = Topic.of("/chat", Integer.class);
        try {
            bus.publish(clash, 1);
            c.fail("type clash: should have been rejected");
        } catch (IllegalArgumentException e) {
            c.pass("type clash: rejected");
        }
    }

    private static void topicNameValidated(Check c) {
        try {
            Topic.of("chat", String.class);
            c.fail("naming: should require a leading slash");
        } catch (IllegalArgumentException e) {
            c.pass("naming: leading slash required");
        }
    }

    private static void introspection(Check c) {
        Bus bus = new Bus(new ManualClock());
        bus.subscribe(CHAT, m -> { });
        bus.publish(CHAT, "a");
        bus.publish(CHAT, "b");

        List<Bus.TopicInfo> topics = bus.topics();
        c.eq("introspect: one topic", 1, topics.size());
        c.eq("introspect: name", "/chat", topics.get(0).name);
        c.eq("introspect: publish count", 2L, topics.get(0).published);
        c.eq("introspect: subscriber count", 1, topics.get(0).subscribers);
    }

    private static void concurrentPublishersAreNotLost(Check c) throws Exception {
        Bus bus = new Bus(new ManualClock());
        AtomicInteger received = new AtomicInteger();
        // Deep queue: this is testing that nothing is lost to a race, not backpressure.
        Subscription sub = bus.subscribe(COUNT, m -> received.incrementAndGet(),
                Bus.Delivery.QUEUED, 10000);

        int threads = 4;
        int each = 500;
        CountDownLatch go = new CountDownLatch(1);
        CountDownLatch done = new CountDownLatch(threads);
        for (int t = 0; t < threads; t++) {
            new Thread(() -> {
                try {
                    go.await();
                    for (int i = 0; i < each; i++) {
                        bus.publish(COUNT, i);
                    }
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                } finally {
                    done.countDown();
                }
            }).start();
        }
        go.countDown();
        done.await();
        bus.drain(sub);

        c.eq("concurrency: nothing lost", threads * each, received.get());
        c.eq("concurrency: sequence complete", (long) threads * each,
                bus.latest(COUNT).sequence());
    }
}
