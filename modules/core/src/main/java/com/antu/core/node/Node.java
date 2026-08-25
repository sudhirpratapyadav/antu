package com.antu.core.node;

import com.antu.core.graph.In;
import com.antu.core.graph.Out;
import com.antu.core.graph.Port;
import com.antu.core.time.Clock;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * One unit of behaviour: a driver, a filter, a planner.
 *
 * <p>A node declares what it consumes and produces as public final port fields,
 * and reads and writes only through them. It never learns the name of another
 * node, and there is nothing to look up at runtime — the wiring is decided when
 * the graph is built, and a mismatch is a compile error.
 *
 * <pre>
 *   public final class Odometer extends Node {
 *       public final In&lt;ImuSample&gt; imu = in("imu", ImuSample.class);
 *       public final Out&lt;Pose2&gt; pose = out("pose", Pose2.class);
 *
 *       public Odometer() { super("odometer"); }
 *
 *       &#64;Override public void tick(Context ctx) {
 *           pose.publish(integrate(imu.get()));
 *       }
 *   }
 * </pre>
 *
 * <p>{@link #tick} must not block. A node that has to wait on hardware owns a
 * thread and publishes from it; the scheduler's loop is shared by every node in
 * the graph, and one blocking read stops all of them.
 */
public abstract class Node {

    private final String name;
    private final List<Port<?>> ports = new ArrayList<>();

    protected Node(String name) {
        if (name == null || name.isEmpty()) {
            throw new IllegalArgumentException("node name is empty");
        }
        this.name = name;
    }

    /** Identifier, unique within a graph. Appears in channel names and logs. */
    public final String name() {
        return name;
    }

    /** Every port this node declared, for wiring and introspection. */
    public final List<Port<?>> ports() {
        return Collections.unmodifiableList(ports);
    }

    /** Declares an output. Call only in a field initialiser or constructor. */
    protected final <T> Out<T> out(String portName, Class<T> type) {
        Out<T> port = Port.newOut(portName, type);
        register(port);
        return port;
    }

    /** Declares an input with no value until something arrives. */
    protected final <T> In<T> in(String portName, Class<T> type) {
        return in(portName, type, null);
    }

    /**
     * Declares an input with a fallback returned by {@code get()} before the first
     * message. A velocity input defaulting to zero is safer than one defaulting to
     * null, because the node cannot forget to check.
     */
    protected final <T> In<T> in(String portName, Class<T> type, T fallback) {
        In<T> port = Port.newIn(portName, type, fallback);
        register(port);
        return port;
    }

    private void register(Port<?> port) {
        for (Port<?> existing : ports) {
            if (existing.name.equals(port.name)) {
                throw new IllegalArgumentException(
                        name + " declares two ports named '" + port.name + "'");
            }
        }
        port.setOwner(name);
        ports.add(port);
    }

    /** Claims resources. Ports are already wired by the time this runs. */
    public void start(Context context) throws Exception {
    }

    /**
     * One step of work, at the node's declared rate.
     *
     * <p>Everything published to this node's inputs since the previous tick is
     * already waiting when this is called.
     */
    public abstract void tick(Context context) throws Exception;

    /** Releases what {@link #start} claimed. Must tolerate being called twice. */
    public void stop() {
    }

    @Override public String toString() {
        return getClass().getSimpleName() + "(" + name + ")";
    }

    /** What a node is given while running. */
    public interface Context {
        Clock clock();
        String nodeName();
        /** Ticks completed since start, useful for rate-dividing inside a node. */
        long tickCount();
    }
}
