package com.antu.brain;

import com.antu.core.geometry.Vec2;
import com.antu.core.graph.In;
import com.antu.core.graph.Out;
import com.antu.core.log.Log;
import com.antu.core.msg.GridMap;
import com.antu.core.msg.PointCloud;
import com.antu.core.msg.PoseEstimate;
import com.antu.core.node.Node;

/**
 * Builds a top-down map from the tracker's feature points.
 *
 * <p>A trail says where the robot has been. A map says what is out there, and
 * that is the difference between reviewing a drive and planning one.
 *
 * <p>The points come free from the visual tracker, which computes them anyway to
 * know where it is. They are sparse and noisy next to what a depth model
 * produces, but they are real geometry in real metres and they are available now.
 *
 * <h2>What gets thrown away, and why</h2>
 *
 * <p>Most incoming points do not belong in a map, and being strict here is what
 * separates a map from a fog:
 *
 * <ul>
 *   <li><b>Low confidence.</b> A reflection or a shadow edge is a feature the
 *       tracker can use for motion and cannot vouch for as geometry.
 *   <li><b>Too far.</b> Triangulation error grows with distance, and points tens
 *       of metres out are guesses. Observed on this robot: features reported at
 *       15 m down a corridor with heights spanning nine metres, which is not a
 *       corridor.
 *   <li><b>Floor and ceiling.</b> A robot drives over the floor and under the
 *       ceiling; neither is an obstacle. Only the band a robot would actually
 *       collide with is kept.
 * </ul>
 *
 * <p>Cells accumulate observations rather than flipping to occupied on first
 * sight, so one spurious point does not become a wall.
 */
public final class OccupancyMapper extends Node {

    private static final String TAG = "mapper";

    /** Points below this are the tracker's guesses, not its evidence. */
    private static final double MIN_CONFIDENCE = 0.3;
    /** Beyond this, triangulation error makes a point worse than no point. */
    private static final double MAX_RANGE_M = 6.0;
    /** Above the floor, where a robot would actually hit something. */
    private static final double OBSTACLE_MIN_M = 0.10;
    private static final double OBSTACLE_MAX_M = 1.20;
    /** Observations before a cell counts as solid. */
    private static final int OCCUPIED_AT = 3;
    /** Counts saturate here rather than wrapping a byte. */
    private static final int MAX_COUNT = 250;

    /** Feature points from the tracker, in its world frame. */
    public final In<PointCloud> points = in("points", PointCloud.class);
    /** Where the robot is, so range can be judged from the observer. */
    public final In<PoseEstimate> pose = in("pose", PoseEstimate.class);

    /** The accumulated map. */
    public final Out<GridMap> map = out("map", GridMap.class);

    private final double resolution;
    private final int width;
    private final int height;
    private final byte[] cells;
    private double cameraHeight = 0.30;
    private long integrated;
    private long rejected;

    /** A 20 m square at 5 cm cells: enough for a floor of a building. */
    public OccupancyMapper() {
        this(0.05, 400, 400);
    }

    public OccupancyMapper(double resolution, int width, int height) {
        super("mapper");
        this.resolution = resolution;
        this.width = width;
        this.height = height;
        this.cells = new byte[width * height];
    }

    /**
     * How high the camera sits above the floor, metres.
     *
     * <p>The tracker's world origin is wherever the camera was when the session
     * started, so the floor is that far below zero. Getting this wrong tilts the
     * obstacle band: too small and the floor is mapped as a wall, too large and
     * real obstacles are discarded as floor.
     */
    public OccupancyMapper setCameraHeight(double metres) {
        this.cameraHeight = metres;
        return this;
    }

    /** Points folded into the map so far. */
    public long integrated() {
        return integrated;
    }

    /** Points discarded by the filters. */
    public long rejected() {
        return rejected;
    }

    @Override public void start(Node.Context ctx) {
        java.util.Arrays.fill(cells, GridMap.UNKNOWN);
        integrated = 0;
        rejected = 0;
    }

    @Override public void tick(Node.Context ctx) {
        if (points.isFresh()) {
            PoseEstimate p = pose.get();
            integrate(points.get(), p);
        }
        map.publish(snapshot());
    }

    private void integrate(PointCloud cloud, PoseEstimate observer) {
        if (cloud == null || cloud.size == 0) {
            return;
        }
        // Without a pose there is no observer to measure range from, and an
        // unanchored estimate would smear the map as it drifts.
        if (observer == null || !observer.isAnchored()) {
            rejected += cloud.size;
            return;
        }
        double floor = -cameraHeight;
        double lowest = floor + OBSTACLE_MIN_M;
        double highest = floor + OBSTACLE_MAX_M;

        for (int i = 0; i < cloud.size; i++) {
            if (cloud.confidence(i) < MIN_CONFIDENCE) {
                rejected++;
                continue;
            }
            double h = YUpFrame.heightOf(cloud.y(i));
            if (h < lowest || h > highest) {
                rejected++;                    // floor, ceiling, or nonsense
                continue;
            }
            Vec2 world = YUpFrame.toPlanar(cloud.x(i), cloud.z(i));
            double range = Math.hypot(world.x - observer.pose.x, world.y - observer.pose.y);
            if (range > MAX_RANGE_M) {
                rejected++;                    // too far to have been triangulated well
                continue;
            }
            mark(world);
        }
    }

    private void mark(Vec2 world) {
        // The grid is centred on the origin, so the robot can drive either way
        // from where it started without immediately falling off the edge.
        int cx = (int) Math.floor(world.x / resolution) + width / 2;
        int cy = (int) Math.floor(world.y / resolution) + height / 2;
        if (cx < 0 || cy < 0 || cx >= width || cy >= height) {
            rejected++;
            return;
        }
        int index = cy * width + cx;
        int count = cells[index] & 0xFF;
        if (count < MAX_COUNT) {
            cells[index] = (byte) (count + 1);
        }
        integrated++;
    }

    private GridMap snapshot() {
        // A copy per publish: the map keeps accumulating, and a reader holding a
        // live array would see it change underneath, which is the one thing the
        // immutability rule exists to prevent.
        double originX = -(width / 2.0) * resolution;
        double originY = -(height / 2.0) * resolution;
        return new GridMap(cells.clone(), width, height, resolution,
                originX, originY, OCCUPIED_AT);
    }
}
