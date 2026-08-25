package com.antu.core.log;

/**
 * Logging for code that must not know it is on Android.
 *
 * <p>{@code core} and {@code brain} cannot call {@code android.util.Log} — that
 * is the point of them compiling without {@code android.jar}. They also cannot
 * usefully print to stdout on a phone, where nothing reads it. So they log
 * through here and the app installs a sink that forwards to logcat, while tests
 * and desktop replay get the console for free.
 *
 * <p>Static because a logger threaded through every constructor is the kind of
 * ceremony that makes people skip logging altogether.
 */
public final class Log {

    /** Severity, ordered. */
    public enum Level {
        DEBUG, INFO, WARN, ERROR
    }

    /** Where log lines go. */
    public interface Sink {
        void write(Level level, String tag, String message, Throwable error);
    }

    /** Prints to stderr. The default, and what tests and desktop tools use. */
    public static final Sink CONSOLE = (level, tag, message, error) -> {
        System.err.println(level + " " + tag + ": " + message);
        if (error != null) {
            error.printStackTrace();
        }
    };

    /** Discards everything, for benchmarks. */
    public static final Sink NONE = (level, tag, message, error) -> { };

    private static volatile Sink sink = CONSOLE;
    private static volatile Level threshold = Level.DEBUG;

    private Log() { }

    /** Installs a sink. The app does this once at startup. */
    public static void setSink(Sink newSink) {
        sink = newSink == null ? NONE : newSink;
    }

    /** Drops anything below {@code level}. */
    public static void setThreshold(Level level) {
        threshold = level;
    }

    public static void d(String tag, String message) {
        write(Level.DEBUG, tag, message, null);
    }

    public static void i(String tag, String message) {
        write(Level.INFO, tag, message, null);
    }

    public static void w(String tag, String message) {
        write(Level.WARN, tag, message, null);
    }

    public static void w(String tag, String message, Throwable error) {
        write(Level.WARN, tag, message, error);
    }

    public static void e(String tag, String message) {
        write(Level.ERROR, tag, message, null);
    }

    public static void e(String tag, String message, Throwable error) {
        write(Level.ERROR, tag, message, error);
    }

    private static void write(Level level, String tag, String message, Throwable error) {
        if (level.ordinal() < threshold.ordinal()) {
            return;
        }
        try {
            sink.write(level, tag, message, error);
        } catch (Throwable t) {
            // A logger that throws must not take down what it was reporting on.
        }
    }
}
