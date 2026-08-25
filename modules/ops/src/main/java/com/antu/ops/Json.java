package com.antu.ops;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

/**
 * Turns message payloads into JSON by reflecting over their public fields.
 *
 * <p>Reflection rather than a hand-written encoder per type, deliberately. The
 * point of this endpoint is introspection: a new message type should appear in
 * the API the moment a node publishes one, without anyone remembering to teach
 * the server about it. A forgotten encoder means a topic that silently reports
 * {@code {}} — exactly the kind of gap that goes unnoticed until it matters.
 *
 * <p>Reads public fields. Types whose state is not public — an array cannot be a
 * public field without breaking the bus's immutability contract — register an
 * encoder with {@link #register}.
 *
 * <p>Reflecting over accessors as well was tried and removed. It recursed
 * forever through derived getters ({@code Vec2.normalised()} returns a
 * {@code Vec2}), producing kilobytes of nested noise for a two-field pose. An
 * explicit registry is duller and predictable, and the fields-based default still
 * covers almost every type without ceremony.
 *
 * <p>For the debug and ops API only; the recorder wants a compact binary format.
 */
public final class Json {

    /** Nested objects deeper than this are almost certainly a cycle. */
    private static final int MAX_DEPTH = 6;

    /** Writes one type's JSON representation. */
    public interface Encoder<T> {
        void write(StringBuilder sb, T value);
    }

    private static final Map<Class<?>, Encoder<?>> ENCODERS = new HashMap<>();

    private Json() { }

    /** Registers an encoder for a type whose state is not in public fields. */
    public static <T> void register(Class<T> type, Encoder<T> encoder) {
        ENCODERS.put(type, encoder);
    }

    /** Starts an object, for use inside an {@link Encoder}. */
    public static void field(StringBuilder sb, String name, Object value, boolean first) {
        if (!first) {
            sb.append(',');
        }
        writeString(sb, name);
        sb.append(':');
        write(sb, value, 0);
    }

    /** Encodes any value: primitives, strings, collections, arrays, or a bean. */
    public static String encode(Object value) {
        StringBuilder sb = new StringBuilder();
        write(sb, value, 0);
        return sb.toString();
    }

    /** Escapes and quotes a string. */
    public static String quote(String s) {
        StringBuilder sb = new StringBuilder(s.length() + 2);
        writeString(sb, s);
        return sb.toString();
    }

    private static void write(StringBuilder sb, Object value, int depth) {
        if (value == null) {
            sb.append("null");
            return;
        }
        if (depth > MAX_DEPTH) {
            sb.append("\"...\"");
            return;
        }

        if (value instanceof String || value instanceof Character || value instanceof Enum) {
            writeString(sb, value.toString());
            return;
        }
        if (value instanceof Boolean) {
            sb.append(value);
            return;
        }
        if (value instanceof Double || value instanceof Float) {
            double d = ((Number) value).doubleValue();
            // JSON has no way to say infinity or NaN. RangeScan uses infinity for
            // "no echo", so it must survive as something a client can test.
            if (Double.isNaN(d)) {
                sb.append("\"NaN\"");
            } else if (Double.isInfinite(d)) {
                sb.append(d > 0 ? "\"Infinity\"" : "\"-Infinity\"");
            } else {
                sb.append(trim(d));
            }
            return;
        }
        if (value instanceof Number) {
            sb.append(value);
            return;
        }
        if (value instanceof Map) {
            sb.append('{');
            boolean first = true;
            for (Map.Entry<?, ?> e : ((Map<?, ?>) value).entrySet()) {
                if (!first) {
                    sb.append(',');
                }
                first = false;
                writeString(sb, String.valueOf(e.getKey()));
                sb.append(':');
                write(sb, e.getValue(), depth + 1);
            }
            sb.append('}');
            return;
        }
        if (value instanceof Collection) {
            sb.append('[');
            boolean first = true;
            for (Object o : (Collection<?>) value) {
                if (!first) {
                    sb.append(',');
                }
                first = false;
                write(sb, o, depth + 1);
            }
            sb.append(']');
            return;
        }
        if (value.getClass().isArray()) {
            sb.append('[');
            int n = java.lang.reflect.Array.getLength(value);
            for (int i = 0; i < n; i++) {
                if (i > 0) {
                    sb.append(',');
                }
                write(sb, java.lang.reflect.Array.get(value, i), depth + 1);
            }
            sb.append(']');
            return;
        }

        @SuppressWarnings("unchecked")
        Encoder<Object> encoder = (Encoder<Object>) ENCODERS.get(value.getClass());
        if (encoder != null) {
            encoder.write(sb, value);
            return;
        }
        writeBean(sb, value, depth);
    }

    /** Public, non-static fields, in declaration order. */
    private static void writeBean(StringBuilder sb, Object value, int depth) {
        sb.append('{');
        boolean first = true;
        for (Field f : value.getClass().getFields()) {
            if (Modifier.isStatic(f.getModifiers())) {
                continue;
            }
            Object v;
            try {
                v = f.get(value);
            } catch (IllegalAccessException e) {
                continue;
            }
            if (!first) {
                sb.append(',');
            }
            first = false;
            writeString(sb, f.getName());
            sb.append(':');
            write(sb, v, depth + 1);
        }
        if (first) {
            // No public fields and no registered encoder. toString beats {} and
            // makes the omission obvious rather than silent.
            writeString(sb, "value");
            sb.append(':');
            writeString(sb, value.toString());
        }
        sb.append('}');
    }

    private static void writeString(StringBuilder sb, String s) {
        sb.append('"');
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            switch (c) {
                case '"':  sb.append("\\\""); break;
                case '\\': sb.append("\\\\"); break;
                case '\n': sb.append("\\n"); break;
                case '\r': sb.append("\\r"); break;
                case '\t': sb.append("\\t"); break;
                default:
                    if (c < 0x20) {
                        sb.append(String.format("\\u%04x", (int) c));
                    } else {
                        sb.append(c);
                    }
            }
        }
        sb.append('"');
    }

    /** Keeps numbers readable: 0.485 rather than 0.48500000000000004. */
    private static String trim(double d) {
        if (d == Math.rint(d) && Math.abs(d) < 1e15) {
            return String.valueOf((long) d);
        }
        String s = String.format("%.6f", d);
        // Strip trailing zeros but leave at least one decimal place.
        int end = s.length();
        while (end > 0 && s.charAt(end - 1) == '0') {
            end--;
        }
        if (end > 0 && s.charAt(end - 1) == '.') {
            end++;
        }
        return s.substring(0, end);
    }
}
