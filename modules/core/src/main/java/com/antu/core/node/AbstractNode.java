package com.antu.core.node;

import com.antu.core.bus.Subscription;

import java.util.ArrayList;
import java.util.List;

/**
 * A {@link Node} that closes its own subscriptions.
 *
 * <p>Forgetting to unsubscribe leaks a listener that keeps receiving messages
 * after its node has stopped, which shows up much later as a node that appears to
 * run twice. Registering through {@link #track} makes that impossible to forget.
 */
public abstract class AbstractNode implements Node {

    private final String name;
    private final List<Subscription> subscriptions = new ArrayList<>();

    protected AbstractNode(String name) {
        this.name = name;
    }

    @Override public final String name() {
        return name;
    }

    /** Registers a subscription for automatic cleanup in {@link #stop}. */
    protected final Subscription track(Subscription subscription) {
        subscriptions.add(subscription);
        return subscription;
    }

    /** Subclasses override this instead of {@link #stop}. */
    protected void onStop() {
    }

    @Override public final void stop() {
        for (Subscription s : subscriptions) {
            s.close();
        }
        subscriptions.clear();
        onStop();
    }

    @Override public String toString() {
        return getClass().getSimpleName() + "(" + name + ")";
    }
}
