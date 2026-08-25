package com.antu.app;

import com.antu.core.bus.Topic;
import com.antu.core.node.AbstractNode;
import com.antu.core.node.Node;

/**
 * A placeholder node that publishes a counter, so phase one has something
 * observable running on the phone before any real driver exists.
 *
 * <p>Its only lasting job is to prove that a node written against the pure core
 * runs unchanged on Android. It gets deleted once the base driver lands.
 */
public final class Heartbeat extends AbstractNode {

    public static final Topic<Long> BEAT = Topic.of("/heartbeat", Long.class);

    private long beats;

    public Heartbeat() {
        super("heartbeat");
    }

    @Override public void start(Node.Context ctx) {
        beats = 0;
    }

    @Override public void tick(Node.Context ctx) {
        ctx.publish(BEAT, ++beats);
    }
}
