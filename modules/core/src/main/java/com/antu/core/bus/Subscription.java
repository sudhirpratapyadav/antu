package com.antu.core.bus;

/** A registration that can be cancelled. Cancelling twice is harmless. */
public interface Subscription extends AutoCloseable {

    /** Stops delivery and releases anything queued. */
    @Override void close();

    /** Messages waiting to be drained, always 0 for direct delivery. */
    int pending();

    /** Messages dropped because the queue was full. */
    long dropped();
}
