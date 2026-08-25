package com.antu.core.time;

/**
 * A clock that only moves when told to.
 *
 * <p>The point of every test that involves timing: advance a scheduler by exactly
 * three ticks and assert what happened, rather than sleeping and hoping. It is
 * also the clock a recording plays back through, which is why it lives in the
 * main source set and not in the tests.
 *
 * <p>{@link #sleepNanos} does not block — it advances time instead. Code written
 * against {@link Clock} therefore runs at full speed under a manual clock without
 * knowing anything has changed.
 */
public final class ManualClock implements Clock {

    private long nanos;

    public ManualClock() {
        this(0);
    }

    public ManualClock(long startNanos) {
        this.nanos = startNanos;
    }

    @Override public synchronized Stamp now() {
        return Stamp.ofNanos(nanos);
    }

    /** Moves time forward. The only way this clock ever changes. */
    public synchronized void advanceNanos(long delta) {
        if (delta < 0) {
            throw new IllegalArgumentException("time does not run backwards: " + delta);
        }
        nanos += delta;
    }

    public void advanceMillis(long delta) {
        advanceNanos(delta * 1_000_000L);
    }

    public void advanceSeconds(double delta) {
        advanceNanos((long) (delta * 1_000_000_000L));
    }

    /** Jumps to an absolute moment, as replay does when seeking. */
    public synchronized void set(Stamp stamp) {
        this.nanos = stamp.nanos();
    }

    @Override public void sleepNanos(long delta) {
        advanceNanos(delta);
    }

    @Override public String toString() {
        return "ManualClock(" + now() + ")";
    }
}
