package com.antu.core.bus;

import com.antu.core.time.Stamp;

/**
 * A payload with the moment it describes.
 *
 * <p>The stamp is when the data was <em>true</em> — when the shutter opened, when
 * the encoder was sampled — not when it was published. Drivers set it from the
 * hardware where they can. Fusing a 200 Hz IMU with a 30 Hz camera is only
 * meaningful if both are honest about that.
 *
 * <p>Payloads are handed between subscribers by reference and never copied, so
 * <b>they must be immutable</b>. Mutating one after publishing corrupts every
 * other subscriber's view, and the resulting bug is close to unfindable.
 */
public final class Message<T> {

    private final Topic<T> topic;
    private final Stamp stamp;
    private final T payload;
    private final long sequence;

    Message(Topic<T> topic, Stamp stamp, T payload, long sequence) {
        this.topic = topic;
        this.stamp = stamp;
        this.payload = payload;
        this.sequence = sequence;
    }

    public Topic<T> topic() {
        return topic;
    }

    /** When the data was true, per the publisher. */
    public Stamp stamp() {
        return stamp;
    }

    public T payload() {
        return payload;
    }

    /** Per-topic counter from 1, for spotting drops without comparing stamps. */
    public long sequence() {
        return sequence;
    }

    @Override public String toString() {
        return topic.name() + "#" + sequence + " @" + stamp + " " + payload;
    }
}
