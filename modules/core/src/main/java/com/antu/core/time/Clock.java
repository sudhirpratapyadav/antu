package com.antu.core.time;

/**
 * Where time comes from.
 *
 * <p>Nothing in the system calls {@code System.nanoTime()} directly. Everything
 * asks a clock, so the same code can run against the wall clock on a robot, a
 * hand-advanced clock in a test, or a recording's timeline during replay — with
 * no branch anywhere to say which.
 */
public interface Clock {

    /** The current moment. */
    Stamp now();

    /**
     * Sleeps for {@code nanos}, or returns immediately for clocks where time is
     * driven externally.
     *
     * @throws InterruptedException if the wait is interrupted
     */
    void sleepNanos(long nanos) throws InterruptedException;

    /** The wall clock, monotonic, for use on a live robot. */
    Clock SYSTEM = new Clock() {

        @Override public Stamp now() {
            return Stamp.ofNanos(System.nanoTime());
        }

        @Override public void sleepNanos(long nanos) throws InterruptedException {
            if (nanos <= 0) {
                return;
            }
            Thread.sleep(nanos / 1_000_000L, (int) (nanos % 1_000_000L));
        }

        @Override public String toString() {
            return "Clock.SYSTEM";
        }
    };
}
