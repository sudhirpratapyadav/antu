package com.antu.core.node;

import com.antu.core.bus.Bus;

/**
 * One unit of behaviour: a driver, a filter, a planner.
 *
 * <p>Nodes talk only through the {@link Bus}. A node holding a reference to
 * another node is the thing this design exists to prevent — it is how a robot
 * stack turns into a knot that cannot be tested a piece at a time.
 *
 * <p>Lifecycle is {@link #start} then repeated {@link #tick} then {@link #stop}.
 * Subscriptions belong in {@code start}, because a node may be started and
 * stopped more than once across a session.
 */
public interface Node {

    /** Identifier, unique within a graph. Appears in logs and the topic list. */
    String name();

    /**
     * Claims resources and subscribes. Publishing here is allowed but rarely
     * wanted, since subscribers may not exist yet.
     */
    void start(Context context) throws Exception;

    /**
     * One step of work, called at the node's declared rate.
     *
     * <p>Queued subscriptions are drained immediately before this, so anything
     * that arrived since the last tick has already reached its listener.
     *
     * <p>Must not block. A node that needs to wait on hardware owns a thread and
     * hands results to its tick through a queue — see the driver modules.
     */
    void tick(Context context) throws Exception;

    /** Releases everything {@link #start} claimed. Must tolerate being called twice. */
    void stop();

    /** What a node is given: the bus, the clock, and its own identity. */
    interface Context {

        /** The bus, for publishing and for latest-value reads. */
        Bus bus();

        com.antu.core.time.Clock clock();

        String nodeName();

        /** Ticks completed since start, useful for rate-dividing inside a node. */
        long tickCount();

        /**
         * Subscribes with queued delivery, drained immediately before this node's
         * tick and closed when it stops.
         *
         * <p>Always prefer this to {@code bus().subscribe(...)}: a subscription
         * the scheduler does not know about is never drained, which presents as a
         * topic that publishes fine and is silently never received.
         */
        <T> com.antu.core.bus.Subscription subscribe(
                com.antu.core.bus.Topic<T> topic, com.antu.core.bus.Listener<T> listener);

        /**
         * Subscribes with an explicit delivery mode. {@code DIRECT} listeners fire
         * on the publisher's thread rather than on this node's tick.
         */
        <T> com.antu.core.bus.Subscription subscribe(
                com.antu.core.bus.Topic<T> topic, com.antu.core.bus.Listener<T> listener,
                Bus.Delivery delivery);

        /** Publishes with the graph clock's current time. */
        <T> void publish(com.antu.core.bus.Topic<T> topic, T payload);

        /** Publishes with an explicit stamp, as drivers should. */
        <T> void publish(com.antu.core.bus.Topic<T> topic, T payload,
                         com.antu.core.time.Stamp stamp);
    }
}
