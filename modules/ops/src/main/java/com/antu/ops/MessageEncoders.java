package com.antu.ops;

import com.antu.core.msg.RangeScan;

/**
 * JSON encoders for message types that {@link Json}'s field reflection cannot
 * reach.
 *
 * <p>One entry so far. {@link RangeScan} keeps its readings in arrays, which
 * cannot be public fields without handing every subscriber something mutable, so
 * it needs telling how to present itself.
 *
 * <p>If a topic ever shows up in the API as a bare {@code toString}, this is the
 * file that is missing an entry.
 */
public final class MessageEncoders {

    private static boolean installed;

    private MessageEncoders() { }

    /** Idempotent; call before serving. */
    public static synchronized void installDefaults() {
        if (installed) {
            return;
        }
        installed = true;

        Json.register(RangeScan.class, (sb, scan) -> {
            sb.append('{');
            Json.field(sb, "size", scan.size(), true);
            Json.field(sb, "minRange", scan.minRange, false);
            Json.field(sb, "maxRange", scan.maxRange, false);
            Json.field(sb, "bearings", scan.bearings(), false);
            Json.field(sb, "ranges", scan.ranges(), false);
            // Which beams are real measurements, rather than making every client
            // re-derive the min/max test and get it subtly wrong.
            boolean[] valid = new boolean[scan.size()];
            for (int i = 0; i < scan.size(); i++) {
                valid[i] = scan.isValid(i);
            }
            Json.field(sb, "valid", valid, false);
            Json.field(sb, "closest", scan.closest(), false);
            sb.append('}');
        });
    }
}
