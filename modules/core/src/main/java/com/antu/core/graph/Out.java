package com.antu.core.graph;

import com.antu.core.time.Stamp;

/**
 * An output port: what a node produces.
 *
 * <p>Publishing to an unconnected output is legal and does nothing. That is
 * deliberate — a driver that reports sonar should not have to know whether
 * anything in this particular robot is listening, and a blueprint that leaves an
 * output dangling is a normal thing to build.
 */
public final class Out<T> extends Port<T> {

    private Channel<T> channel;

    Out(String name, Class<T> type) {
        super(name, type);
    }

    void attach(Channel<T> channel) {
        this.channel = channel;
    }

    /**
     * True when a node reads this output.
     *
     * <p>Distinct from having a channel: every output gets one at build time so it
     * can be observed, but that says nothing about whether anything in the graph
     * consumes it.
     */
    public boolean isConnected() {
        return channel != null && channel.readerCount() > 0;
    }

    /** Publishes with the graph clock's current time. */
    public void publish(T value) {
        Channel<T> c = channel;
        if (c != null) {
            c.publish(value, c.clock().now());
        }
    }

    /**
     * Publishes with an explicit stamp — when the data was <em>true</em>, not when
     * it was sent. Drivers should always use this where the hardware offers it,
     * since fusing a 200 Hz IMU with a 30 Hz camera depends on it.
     */
    public void publish(T value, Stamp stamp) {
        Channel<T> c = channel;
        if (c != null) {
            c.publish(value, stamp);
        }
    }

    /** The channel this feeds, or null when unconnected. */
    Channel<T> channel() {
        return channel;
    }
}
