package com.antu.core.bus;

/** Receives messages on a topic. */
public interface Listener<T> {
    void onMessage(Message<T> message);
}
