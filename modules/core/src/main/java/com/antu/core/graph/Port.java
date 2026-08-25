package com.antu.core.graph;

/**
 * One end of a connection, declared by a node as a public final field.
 *
 * <p>Ports are what make the graph checkable before it runs. A node states what
 * it produces and consumes as typed fields; wiring them is a generic call, so a
 * mismatch is a compile error rather than an exception on a robot. Nothing here
 * is discovered at runtime.
 */
public abstract class Port<T> {

    /** Name within the owning node, e.g. {@code "odom"}. */
    public final String name;
    /** Payload type, kept for autoconnect and for the introspection API. */
    public final Class<T> type;

    private String owner = "?";

    Port(String name, Class<T> type) {
        if (name == null || name.isEmpty()) {
            throw new IllegalArgumentException("port name is empty");
        }
        this.name = name;
        this.type = type;
    }

    /** Node that declared this port. Set when the node is added to a builder. */
    public String owner() {
        return owner;
    }

    public void setOwner(String owner) {
        this.owner = owner;
    }

    /** Creates an output port. Used by {@code Node}, not by node authors. */
    public static <T> Out<T> newOut(String name, Class<T> type) {
        return new Out<>(name, type);
    }

    /** Creates an input port. Used by {@code Node}, not by node authors. */
    public static <T> In<T> newIn(String name, Class<T> type, T fallback) {
        return In.of(name, type, fallback);
    }

    /** Fully qualified, e.g. {@code "base.odom"}. Used in errors and the API. */
    public String path() {
        return owner + "." + name;
    }

    @Override public String toString() {
        return path() + " [" + type.getSimpleName() + "]";
    }
}
