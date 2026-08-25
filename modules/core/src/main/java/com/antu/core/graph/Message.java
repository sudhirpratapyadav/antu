package com.antu.core.graph;

import com.antu.core.time.Stamp;

/**
 * A payload with the moment it describes.
 *
 * <p>The stamp is when the data was <em>true</em> — when the shutter opened, when
 * the encoder was sampled — not when it was sent. Drivers set it from the
 * hardware where they can. Fusing a 200 Hz IMU with a 30 Hz camera is only
 * meaningful if both are honest about that.
 *
 * <p>Payloads are handed between nodes by reference and never copied, so
 * <b>they must be immutable</b>. Mutating one after publishing corrupts every
 * other reader's view, and the resulting bug is close to unfindable.
 */
public final class Message<T> {

    private final Stamp stamp;
    private final T payload;
    private final long sequence;

    Message(Stamp stamp, T payload, long sequence) {
        this.stamp = stamp;
        this.payload = payload;
        this.sequence = sequence;
    }

    /** When the data was true, per the publisher. */
    public Stamp stamp() {
        return stamp;
    }

    public T payload() {
        return payload;
    }

    /** Per-channel counter from 1, for spotting drops without comparing stamps. */
    public long sequence() {
        return sequence;
    }

    @Override public String toString() {
        return "#" + sequence + " @" + stamp + " " + payload;
    }
}
