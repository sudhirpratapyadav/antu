package com.antu.core.graph;


import java.util.Collections;
import java.util.List;

/**
 * An input port: what a node consumes.
 *
 * <p>Read from inside {@link com.antu.core.node.Node#tick}, never through a
 * callback. In a tick-driven graph a callback only moves the same work onto
 * another thread and takes determinism with it; here the scheduler guarantees
 * that everything published for this input since the last tick is already
 * waiting.
 *
 * <p>Two ways to read, and the choice matters:
 *
 * <ul>
 *   <li>{@link #get} — the latest value. Right for state: a pose, a goal, a
 *       velocity command, where an older sample has been superseded.
 *   <li>{@link #drain} — every message since the last tick, in order. Right for
 *       events, and for integrating a sensor where skipping a sample loses
 *       distance travelled.
 * </ul>
 */
public final class In<T> extends Port<T> {

    /** Queue depth before the oldest is dropped. */
    private static final int DEFAULT_DEPTH = 64;

    private final T fallback;
    private final int depth;
    private Channel<T>.Sink sink;
    private boolean required = true;

    In(String name, Class<T> type, T fallback, int depth) {
        super(name, type);
        this.fallback = fallback;
        this.depth = depth;
    }

    static <T> In<T> of(String name, Class<T> type, T fallback) {
        return new In<>(name, type, fallback, DEFAULT_DEPTH);
    }

    /**
     * Marks this input as safe to leave unconnected.
     *
     * <p>Inputs are required by default, so a graph missing a wire fails to build
     * rather than running with a node that silently never receives anything.
     */
    public In<T> optional() {
        this.required = false;
        return this;
    }

    public boolean isRequired() {
        return required;
    }

    public boolean isConnected() {
        return sink != null;
    }

    void attach(Channel<T>.Sink sink) {
        this.sink = sink;
    }

    int depth() {
        return depth;
    }

    /** Latest value, or the port's fallback when nothing has arrived. */
    public T get() {
        Channel<T>.Sink s = sink;
        if (s == null) {
            return fallback;
        }
        Message<T> m = s.latest();
        return m == null ? fallback : m.payload();
    }

    /** Latest message, with its stamp and sequence, or null. */
    public Message<T> latest() {
        Channel<T>.Sink s = sink;
        return s == null ? null : s.latest();
    }

    /** True when something arrived since the previous tick. */
    public boolean isFresh() {
        Channel<T>.Sink s = sink;
        return s != null && s.isFresh();
    }

    /**
     * Everything queued since the last tick, oldest first, clearing the queue.
     *
     * <p>Use where every sample counts. Messages beyond the queue depth are
     * dropped oldest-first and counted by {@link #dropped}.
     */
    public List<Message<T>> drain() {
        Channel<T>.Sink s = sink;
        return s == null ? Collections.emptyList() : s.drain();
    }

    /** Messages lost because this input did not keep up. */
    public long dropped() {
        Channel<T>.Sink s = sink;
        return s == null ? 0 : s.dropped();
    }
}
