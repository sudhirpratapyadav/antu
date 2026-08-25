package com.antu.core.bus;

/**
 * A named, typed channel.
 *
 * <p>Topics are declared as constants rather than spelled out as strings at each
 * call site, so a typo is a compile error and the payload type is carried in the
 * signature:
 *
 * <pre>
 *   public static final Topic&lt;Twist2&gt; CMD_VEL = Topic.of("/cmd_vel", Twist2.class);
 * </pre>
 *
 * <p>Identity is the name alone. Two topics with the same name and different
 * types are a bug, and {@link Bus} rejects it rather than letting a subscriber
 * receive something it cannot cast.
 */
public final class Topic<T> {

    private final String name;
    private final Class<T> type;

    private Topic(String name, Class<T> type) {
        if (name == null || name.isEmpty()) {
            throw new IllegalArgumentException("topic name is empty");
        }
        if (name.charAt(0) != '/') {
            throw new IllegalArgumentException("topic must start with '/': " + name);
        }
        this.name = name;
        this.type = type;
    }

    public static <T> Topic<T> of(String name, Class<T> type) {
        return new Topic<>(name, type);
    }

    public String name() {
        return name;
    }

    public Class<T> type() {
        return type;
    }

    @Override public boolean equals(Object o) {
        return o instanceof Topic && ((Topic<?>) o).name.equals(name);
    }

    @Override public int hashCode() {
        return name.hashCode();
    }

    @Override public String toString() {
        return name + " [" + type.getSimpleName() + "]";
    }
}
