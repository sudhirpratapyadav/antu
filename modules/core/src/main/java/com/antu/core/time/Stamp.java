package com.antu.core.time;

/**
 * A moment, in nanoseconds since an arbitrary origin.
 *
 * <p>Every message carries one, applied by the driver that produced it rather
 * than by whoever happens to receive it. That distinction is what makes a
 * recording replayable: the timeline belongs to the data, not to the run.
 *
 * <p>Nanoseconds because sensor fusion across a 200 Hz IMU and a 30 Hz camera
 * cares about sub-millisecond offsets, and a long holds nanoseconds for close to
 * three centuries.
 */
public final class Stamp implements Comparable<Stamp> {

    private static final long NANOS_PER_SECOND = 1_000_000_000L;
    private static final long NANOS_PER_MILLI = 1_000_000L;

    /** The zero moment, for uninitialised or "unknown" times. */
    public static final Stamp ZERO = new Stamp(0);

    private final long nanos;

    private Stamp(long nanos) {
        this.nanos = nanos;
    }

    public static Stamp ofNanos(long nanos) {
        return new Stamp(nanos);
    }

    public static Stamp ofMillis(long millis) {
        return new Stamp(millis * NANOS_PER_MILLI);
    }

    public static Stamp ofSeconds(double seconds) {
        return new Stamp((long) (seconds * NANOS_PER_SECOND));
    }

    public long nanos() {
        return nanos;
    }

    public long millis() {
        return nanos / NANOS_PER_MILLI;
    }

    public double seconds() {
        return (double) nanos / NANOS_PER_SECOND;
    }

    public Stamp plusNanos(long delta) {
        return new Stamp(nanos + delta);
    }

    public Stamp plusMillis(long delta) {
        return new Stamp(nanos + delta * NANOS_PER_MILLI);
    }

    /** Nanoseconds from {@code other} to this. Negative when this is earlier. */
    public long since(Stamp other) {
        return nanos - other.nanos;
    }

    /** Seconds from {@code other} to this, the usual form for integration steps. */
    public double secondsSince(Stamp other) {
        return (double) (nanos - other.nanos) / NANOS_PER_SECOND;
    }

    public boolean isBefore(Stamp other) {
        return nanos < other.nanos;
    }

    public boolean isAfter(Stamp other) {
        return nanos > other.nanos;
    }

    @Override public int compareTo(Stamp other) {
        return Long.compare(nanos, other.nanos);
    }

    @Override public boolean equals(Object o) {
        return o instanceof Stamp && ((Stamp) o).nanos == nanos;
    }

    @Override public int hashCode() {
        return Long.hashCode(nanos);
    }

    @Override public String toString() {
        return String.format("%d.%09d", nanos / NANOS_PER_SECOND, Math.abs(nanos % NANOS_PER_SECOND));
    }
}
