package com.antu.core.graph;

import com.antu.core.time.Clock;
import com.antu.core.time.Stamp;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicLong;

/**
 * The conduit between one output and the inputs wired to it.
 *
 * <p>Created by {@link Graph.Builder#build}, never at runtime. Because the set of
 * channels is fixed before anything starts, a category of failure stops existing:
 * no topic that does not exist yet, no reader that attaches before its writer, no
 * disagreement about payload type. Those were not rare edge cases — they were the
 * first three bugs the dynamic version produced.
 *
 * <p>Publishing costs an allocation for the envelope and one enqueue per reader.
 * Nothing is serialised; that happens only at the edges, in the recorder and the
 * bridge, where data actually leaves the process.
 *
 * <h2>Remote channels, later</h2>
 *
 * <p>A channel that spans machines needs only the two hooks already here:
 * {@link #addListener} to observe what is published, and {@link #inject} to feed
 * in what arrives from elsewhere. That is also exactly what replaying a recording
 * needs, which is why there is no transport abstraction — it would be a name for
 * something these two already do.
 *
 * <p>Thread-safe. Drivers publish from their own threads while the graph ticks.
 */
public final class Channel<T> {

    private final String name;
    private final Class<T> type;
    private final Clock clock;
    private final List<Sink> sinks = new CopyOnWriteArrayList<>();
    private final List<Listener<T>> listeners = new CopyOnWriteArrayList<>();
    private final AtomicLong sequence = new AtomicLong();
    private volatile Message<T> latest;

    Channel(String name, Class<T> type, Clock clock) {
        this.name = name;
        this.type = type;
        this.clock = clock;
    }

    /** Channel name, taken from the producing port, e.g. {@code "base.odom"}. */
    public String name() {
        return name;
    }

    public Class<T> type() {
        return type;
    }

    Clock clock() {
        return clock;
    }

    /** Most recent message, or null before anything is published. */
    public Message<T> latest() {
        return latest;
    }

    public long published() {
        return sequence.get();
    }

    public int readerCount() {
        return sinks.size();
    }

    void publish(T value, Stamp stamp) {
        if (value == null) {
            throw new IllegalArgumentException("cannot publish null on " + name);
        }
        Message<T> message = new Message<>(stamp, value, sequence.incrementAndGet());
        latest = message;
        for (Sink sink : sinks) {
            sink.offer(message);
        }
        for (Listener<T> l : listeners) {
            try {
                l.onMessage(message);
            } catch (Throwable t) {
                // An observer must never break the graph it is watching.
            }
        }
    }

    /**
     * Publishes onto this channel from outside the graph.
     *
     * <p>For replaying a recording and for a channel fed from another machine. Use
     * sparingly and never alongside a live producer on the same channel: two
     * writers make the readings incoherent, which is the whole reason wiring
     * enforces one writer per input.
     */
    public void inject(T value, Stamp stamp) {
        publish(value, stamp);
    }

    /** Notified on every message, on the publisher's thread. */
    public interface Listener<T> {
        void onMessage(Message<T> message);
    }

    /**
     * Attaches an observer that is not part of the graph — the bridge, the
     * recorder, the diagnostics API.
     *
     * <p>Called on the publishing thread, so an implementation must not block. The
     * bridge queues and writes from its own thread for exactly that reason: doing
     * the socket write here once stalled the serial driver and killed the app.
     */
    public void addListener(Listener<T> listener) {
        listeners.add(listener);
    }

    public void removeListener(Listener<T> listener) {
        listeners.remove(listener);
    }

    Sink newSink(int depth) {
        Sink sink = new Sink(depth);
        sinks.add(sink);
        return sink;
    }

    /** One reader's view: its queue, its freshness flag, its drop count. */
    public final class Sink {

        private final Deque<Message<T>> queue = new ArrayDeque<>();
        private final int depth;
        private final AtomicLong dropped = new AtomicLong();
        private volatile boolean fresh;

        Sink(int depth) {
            this.depth = Math.max(1, depth);
        }

        void offer(Message<T> message) {
            synchronized (queue) {
                // Newest data matters most on a robot: a stale velocity command is
                // worse than a missing one, so the oldest goes when full.
                if (queue.size() >= depth) {
                    queue.removeFirst();
                    dropped.incrementAndGet();
                }
                queue.addLast(message);
            }
            fresh = true;
        }

        Message<T> latest() {
            return Channel.this.latest;
        }

        boolean isFresh() {
            return fresh;
        }

        /** Cleared by the scheduler once the owning node has ticked. */
        void clearFresh() {
            fresh = false;
        }

        List<Message<T>> drain() {
            synchronized (queue) {
                if (queue.isEmpty()) {
                    return Collections.emptyList();
                }
                List<Message<T>> out = new ArrayList<>(queue);
                queue.clear();
                return out;
            }
        }

        long dropped() {
            return dropped.get();
        }
    }

    @Override public String toString() {
        return name + " [" + type.getSimpleName() + "] n=" + published()
                + " readers=" + readerCount();
    }
}
