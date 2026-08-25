package com.antu.core.bus;

import com.antu.core.time.Clock;
import com.antu.core.time.Stamp;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Deque;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicLong;

/**
 * In-process publish/subscribe.
 *
 * <p>Messages are handed to subscribers by reference. There is no serialisation
 * anywhere in this class — that happens only at the edges, in the recorder and
 * the bridge, where data actually leaves the process. Publishing costs an
 * allocation for the envelope and an enqueue per subscriber, which is why the
 * usual objection to a message bus on a phone does not apply here. The cost of
 * ROS is DDS and IPC, not the idea.
 *
 * <h2>Two delivery modes, and why the default is the slower-looking one</h2>
 *
 * <p>{@link Delivery#DIRECT} invokes the listener on the publisher's thread.
 * Lowest latency, and the right choice for a control loop that must react to an
 * IMU sample immediately. It also means a slow subscriber stalls the driver, and
 * that execution order depends on thread scheduling — so a recording will not
 * replay the same way twice.
 *
 * <p>{@link Delivery#QUEUED}, the default, buffers into a per-subscriber queue
 * that its owner drains on its own tick. Publishers never block, and because the
 * scheduler drains in a declared order, the same recording produces the same
 * execution every time. That property is worth more than the microseconds it
 * costs, and it is close to impossible to retrofit later.
 *
 * <p>Thread-safe. Publish from anywhere.
 */
public final class Bus {

    /** How a subscriber wants its messages. */
    public enum Delivery {
        /** Buffered; the owner drains it. Deterministic under a tick scheduler. */
        QUEUED,
        /** Invoked inline on the publisher's thread. Lowest latency, non-deterministic. */
        DIRECT
    }

    /** Queue depth before the oldest message is dropped. */
    private static final int DEFAULT_QUEUE_DEPTH = 64;

    private final Clock clock;
    private final Map<String, TopicState<?>> topics = new ConcurrentHashMap<>();

    public Bus(Clock clock) {
        this.clock = clock;
    }

    public Clock clock() {
        return clock;
    }

    /**
     * Publishes with the current time as the stamp. Use
     * {@link #publish(Topic, Object, Stamp)} wherever the hardware offers a
     * better one.
     */
    public <T> void publish(Topic<T> topic, T payload) {
        publish(topic, payload, clock.now());
    }

    /** Publishes {@code payload} as having been true at {@code stamp}. */
    public <T> void publish(Topic<T> topic, T payload, Stamp stamp) {
        if (payload == null) {
            throw new IllegalArgumentException("cannot publish null on " + topic);
        }
        TopicState<T> state = state(topic);
        Message<T> message = new Message<>(topic, stamp, payload, state.count.incrementAndGet());
        state.latest = message;
        for (Sub<T> sub : state.subs) {
            sub.deliver(message);
        }
    }

    /** Subscribes with queued delivery. */
    public <T> Subscription subscribe(Topic<T> topic, Listener<T> listener) {
        return subscribe(topic, listener, Delivery.QUEUED, DEFAULT_QUEUE_DEPTH);
    }

    public <T> Subscription subscribe(Topic<T> topic, Listener<T> listener, Delivery delivery) {
        return subscribe(topic, listener, delivery, DEFAULT_QUEUE_DEPTH);
    }

    /**
     * Subscribes to {@code topic}.
     *
     * <p>A queued subscription delivers nothing until {@link Subscription} is
     * drained — {@link #drain(Subscription)}, or the scheduler doing it for a
     * node. That is deliberate: it is what makes the ordering reproducible.
     */
    public <T> Subscription subscribe(Topic<T> topic, Listener<T> listener,
                                      Delivery delivery, int queueDepth) {
        TopicState<T> state = state(topic);
        Sub<T> sub = new Sub<>(state, listener, delivery, queueDepth);
        state.subs.add(sub);
        return sub;
    }

    /**
     * Delivers everything waiting on a queued subscription.
     *
     * @return how many messages were handed over
     */
    public int drain(Subscription subscription) {
        if (!(subscription instanceof Sub)) {
            return 0;
        }
        return ((Sub<?>) subscription).drain();
    }

    /** The most recent message on {@code topic}, or null if nothing was published. */
    @SuppressWarnings("unchecked")
    public <T> Message<T> latest(Topic<T> topic) {
        TopicState<?> state = topics.get(topic.name());
        return state == null ? null : (Message<T>) state.latest;
    }

    /** The most recent payload on {@code topic}, or {@code fallback}. */
    public <T> T latestOr(Topic<T> topic, T fallback) {
        Message<T> m = latest(topic);
        return m == null ? fallback : m.payload();
    }

    /** Everything published on so far, for the topic list in the UI. */
    public List<TopicInfo> topics() {
        List<TopicInfo> out = new ArrayList<>();
        for (TopicState<?> s : topics.values()) {
            out.add(new TopicInfo(s.topic.name(), s.topic.type().getSimpleName(),
                    s.count.get(), s.subs.size(),
                    s.latest == null ? Stamp.ZERO : s.latest.stamp()));
        }
        out.sort(Comparator.comparing(a -> a.name));
        return out;
    }

    /** A snapshot of one topic, for introspection. */
    public static final class TopicInfo {
        public final String name;
        public final String type;
        public final long published;
        public final int subscribers;
        public final Stamp lastStamp;

        TopicInfo(String name, String type, long published, int subscribers, Stamp lastStamp) {
            this.name = name;
            this.type = type;
            this.published = published;
            this.subscribers = subscribers;
            this.lastStamp = lastStamp;
        }

        @Override public String toString() {
            return String.format("%-24s %-14s n=%-8d subs=%d", name, type, published, subscribers);
        }
    }

    @SuppressWarnings("unchecked")
    private <T> TopicState<T> state(Topic<T> topic) {
        TopicState<?> existing = topics.computeIfAbsent(topic.name(), k -> new TopicState<>(topic));
        if (existing.topic.type() != topic.type()) {
            throw new IllegalArgumentException("topic " + topic.name() + " is already "
                    + existing.topic.type().getSimpleName()
                    + ", cannot also be " + topic.type().getSimpleName());
        }
        return (TopicState<T>) existing;
    }

    private static final class TopicState<T> {
        final Topic<T> topic;
        final CopyOnWriteArrayList<Sub<T>> subs = new CopyOnWriteArrayList<>();
        final AtomicLong count = new AtomicLong();
        volatile Message<T> latest;

        TopicState(Topic<T> topic) {
            this.topic = topic;
        }
    }

    private static final class Sub<T> implements Subscription {
        private final TopicState<T> state;
        private final Listener<T> listener;
        private final Delivery delivery;
        private final int depth;
        private final Deque<Message<T>> queue = new ArrayDeque<>();
        private final AtomicLong dropped = new AtomicLong();
        private volatile boolean open = true;

        Sub(TopicState<T> state, Listener<T> listener, Delivery delivery, int depth) {
            this.state = state;
            this.listener = listener;
            this.delivery = delivery;
            this.depth = Math.max(1, depth);
        }

        void deliver(Message<T> message) {
            if (!open) {
                return;
            }
            if (delivery == Delivery.DIRECT) {
                invoke(message);
                return;
            }
            synchronized (queue) {
                // Newest data matters most on a robot: a stale velocity command is
                // worse than a missing one, so the oldest goes when full.
                if (queue.size() >= depth) {
                    queue.removeFirst();
                    dropped.incrementAndGet();
                }
                queue.addLast(message);
            }
        }

        int drain() {
            int n = 0;
            while (open) {
                Message<T> m;
                synchronized (queue) {
                    m = queue.pollFirst();
                }
                if (m == null) {
                    break;
                }
                invoke(m);
                n++;
            }
            return n;
        }

        private void invoke(Message<T> message) {
            try {
                listener.onMessage(message);
            } catch (Throwable t) {
                // One broken subscriber must not take down the publisher or its
                // siblings. The scheduler surfaces these; the bus only survives them.
                Thread current = Thread.currentThread();
                current.getUncaughtExceptionHandler().uncaughtException(current, t);
            }
        }

        @Override public void close() {
            open = false;
            state.subs.remove(this);
            synchronized (queue) {
                queue.clear();
            }
        }

        @Override public int pending() {
            synchronized (queue) {
                return queue.size();
            }
        }

        @Override public long dropped() {
            return dropped.get();
        }
    }
}
