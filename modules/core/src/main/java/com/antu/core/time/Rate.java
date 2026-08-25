package com.antu.core.time;

/**
 * A frequency, and the period that falls out of it.
 *
 * <p>Exists so declarations read as {@code Rate.hz(30)} rather than a bare
 * {@code 33_333_333L}, and so the scheduler can reason about which rates divide
 * into which.
 */
public final class Rate {

    private final long periodNanos;

    private Rate(long periodNanos) {
        if (periodNanos <= 0) {
            throw new IllegalArgumentException("period must be positive: " + periodNanos);
        }
        this.periodNanos = periodNanos;
    }

    public static Rate hz(double hz) {
        if (hz <= 0) {
            throw new IllegalArgumentException("rate must be positive: " + hz);
        }
        return new Rate((long) (1_000_000_000L / hz));
    }

    public static Rate periodMillis(long millis) {
        return new Rate(millis * 1_000_000L);
    }

    public static Rate periodNanos(long nanos) {
        return new Rate(nanos);
    }

    public long periodNanos() {
        return periodNanos;
    }

    public double hz() {
        return 1_000_000_000.0 / periodNanos;
    }

    @Override public String toString() {
        return String.format("%.1fHz", hz());
    }
}
