package com.antu.ops;

import com.antu.core.msg.RangeScan;
import com.antu.core.msg.GridMap;
import com.antu.core.msg.PointCloud;
import com.antu.core.msg.PosedFrame;
import com.antu.core.msg.VideoFrame;

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

        // Metadata only. Base64 of a megabyte per frame would flood the telemetry
        // socket and is useless to read; the pixels go over /video.mjpeg instead.
        Json.register(VideoFrame.class, (sb, f) -> {
            sb.append('{');
            Json.field(sb, "width", f.width, true);
            Json.field(sb, "height", f.height, false);
            Json.field(sb, "index", f.index, false);
            Json.field(sb, "sizeBytes", f.sizeBytes(), false);
            Json.field(sb, "url", "/video.mjpeg", false);
            sb.append('}');
        });

        // Metadata and geometry, never the pixels. The pose is the interesting
        // part over telemetry — it is what says where this picture was taken.
        Json.register(PosedFrame.class, (sb, f) -> {
            sb.append('{');
            Json.field(sb, "width", f.image.width, true);
            Json.field(sb, "height", f.image.height, false);
            Json.field(sb, "index", f.image.index, false);
            Json.field(sb, "sizeBytes", f.image.sizeBytes(), false);
            Json.field(sb, "cameraPose", f.cameraPose, false);
            Json.field(sb, "fx", f.fx, false);
            Json.field(sb, "fy", f.fy, false);
            Json.field(sb, "cx", f.cx, false);
            Json.field(sb, "cy", f.cy, false);
            Json.field(sb, "url", "/video.mjpeg", false);
            sb.append('}');
        });

        // Points, but capped. A cloud is thousands of them many times a second,
        // and shipping all of it over the telemetry socket would drown everything
        // else on there. The map is built on the robot; this is for looking.
        Json.register(PointCloud.class, (sb, cloud) -> {
            final int limit = 400;
            final double minConfidence = 0.3;
            sb.append('{');
            Json.field(sb, "size", cloud.size, true);
            Json.field(sb, "confident", cloud.countAbove(minConfidence), false);
            sb.append(",\"points\":[");
            int written = 0;
            for (int i = 0; i < cloud.size && written < limit; i++) {
                if (cloud.confidence(i) < minConfidence) {
                    continue;
                }
                if (written > 0) {
                    sb.append(',');
                }
                sb.append('[').append(Json.encode(cloud.x(i))).append(',')
                  .append(Json.encode(cloud.y(i))).append(',')
                  .append(Json.encode(cloud.z(i))).append(']');
                written++;
            }
            sb.append("],");
            Json.field(sb, "truncated", written >= limit, true);
            sb.append('}');
        });

        // Only the occupied cells, as coordinates. A 400x400 grid is 160,000
        // cells and almost all of them are unknown; sending the lot every tick
        // would swamp the socket to say nothing.
        Json.register(GridMap.class, (sb, m) -> {
            sb.append('{');
            Json.field(sb, "resolution", m.resolution, true);
            Json.field(sb, "width", m.width, false);
            Json.field(sb, "height", m.height, false);
            Json.field(sb, "originX", m.originX, false);
            Json.field(sb, "originY", m.originY, false);
            Json.field(sb, "observed", m.observedCells(), false);
            Json.field(sb, "occupied", m.occupiedCells(), false);
            sb.append(",\"cells\":[");
            int written = 0;
            final int limit = 6000;
            for (int cy = 0; cy < m.height && written < limit; cy++) {
                for (int cx = 0; cx < m.width && written < limit; cx++) {
                    if (!m.isOccupied(cx, cy)) {
                        continue;
                    }
                    if (written > 0) {
                        sb.append(',');
                    }
                    // World metres rather than cell indices, so a client needs no
                    // knowledge of the grid's own geometry to draw it.
                    sb.append('[').append(Json.encode(m.worldX(cx))).append(',')
                      .append(Json.encode(m.worldY(cy))).append(']');
                    written++;
                }
            }
            sb.append("]}");
        });

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
