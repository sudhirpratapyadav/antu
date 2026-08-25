package com.antu.brain;

import com.antu.core.graph.In;
import com.antu.core.graph.Out;
import com.antu.core.msg.PointCloud;
import com.antu.core.msg.PoseEstimate;
import com.antu.core.node.Node;

import java.util.HashMap;
import java.util.Map;

/**
 * The accumulated 3D model of the space.
 *
 * <p>Sibling to {@link OccupancyMapper}, from the same points and for a different
 * purpose. That one flattens everything onto the floor because a robot planning a
 * route does not care how tall a wall is. This one keeps the height, because a
 * person looking at what the robot found very much does, and because the height
 * is the only thing that distinguishes a table from a doorway.
 *
 * <h2>Voxels, or it grows without limit</h2>
 *
 * <p>Depth yields a few thousand points a frame, forever. Keeping them all would
 * fill memory in minutes and mostly with duplicates, since a robot dwelling in
 * one place re-observes the same surface hundreds of times.
 *
 * <p>Points are therefore quantised into a voxel grid, one point per occupied
 * voxel, holding a running mean of everything seen there. That bounds the cloud
 * by the size of the room rather than the length of the drive, and averaging
 * makes each surviving point better than any single observation of it.
 */
public final class CloudMap extends Node {

    /** Voxel side, metres. Finer than this and monocular noise dominates. */
    private static final double DEFAULT_VOXEL = 0.04;
    /**
     * Voxels retained. At 4 cm this is a generous room; beyond it the oldest are
     * not evicted, new ones are simply refused, because silently forgetting one
     * end of a map is worse than declining to extend it.
     */
    private static final int MAX_VOXELS = 400_000;

    /** Depth points in the tracker's world frame. */
    public final In<PointCloud> points = in("points", PointCloud.class);
    /** Only accumulate while the pose is anchored; see {@link #tick}. */
    public final In<PoseEstimate> pose = in("pose", PoseEstimate.class);

    /** The whole accumulated cloud, republished as it grows. */
    public final Out<PointCloud> cloud = out("cloud", PointCloud.class);

    private final double voxel;
    private final Map<Long, float[]> voxels = new HashMap<>();
    private long added;
    private long merged;
    private long refused;
    private boolean dirty;

    public CloudMap() {
        this(DEFAULT_VOXEL);
    }

    public CloudMap(double voxelMetres) {
        super("cloud");
        this.voxel = voxelMetres;
    }

    /** Distinct voxels held. */
    public int voxelCount() {
        return voxels.size();
    }

    /** Points folded into existing voxels rather than creating new ones. */
    public long mergedPoints() {
        return merged;
    }

    /** Points dropped because the cloud is full. */
    public long refusedPoints() {
        return refused;
    }

    @Override public void start(Node.Context ctx) {
        voxels.clear();
        added = 0;
        merged = 0;
        refused = 0;
        dirty = true;
    }

    @Override public void tick(Node.Context ctx) {
        if (points.isFresh()) {
            PoseEstimate p = pose.get();
            // Only while anchored. Points placed by a drifting estimate land in
            // the wrong voxels, and unlike the occupancy grid — where a wrong cell
            // is one wrong cell — a smeared cloud cannot be told from a real
            // surface afterwards.
            if (p != null && p.isAnchored()) {
                integrate(points.get());
            }
        }
        if (dirty) {
            dirty = false;
            cloud.publish(snapshot());
        }
    }

    private void integrate(PointCloud incoming) {
        if (incoming == null || incoming.size == 0) {
            return;
        }
        for (int i = 0; i < incoming.size; i++) {
            double x = incoming.x(i);
            double y = incoming.y(i);
            double z = incoming.z(i);
            long key = keyOf(x, y, z);

            float[] cell = voxels.get(key);
            if (cell == null) {
                if (voxels.size() >= MAX_VOXELS) {
                    refused++;
                    continue;
                }
                // x, y, z sum and a count, so the stored point is a running mean.
                voxels.put(key, new float[] {(float) x, (float) y, (float) z, 1});
                added++;
            } else {
                cell[0] += (float) x;
                cell[1] += (float) y;
                cell[2] += (float) z;
                cell[3] += 1;
                merged++;
            }
        }
        dirty = true;
    }

    /**
     * Quantises a position into a voxel key.
     *
     * <p>21 bits per axis, which spans about 80 km at 4 cm and is therefore not
     * the limit anything will hit first. The offset keeps negative coordinates
     * positive so the packing stays monotonic.
     */
    private long keyOf(double x, double y, double z) {
        long ix = (long) Math.floor(x / voxel) + (1 << 20);
        long iy = (long) Math.floor(y / voxel) + (1 << 20);
        long iz = (long) Math.floor(z / voxel) + (1 << 20);
        return (ix & 0x1FFFFF) | ((iy & 0x1FFFFF) << 21) | ((iz & 0x1FFFFF) << 42);
    }

    private PointCloud snapshot() {
        int n = voxels.size();
        float[] xyz = new float[n * 3];
        float[] conf = new float[n];
        int i = 0;
        for (float[] cell : voxels.values()) {
            float count = cell[3];
            xyz[i * 3] = cell[0] / count;
            xyz[i * 3 + 1] = cell[1] / count;
            xyz[i * 3 + 2] = cell[2] / count;
            // Repeated observation is the confidence: a surface seen from several
            // poses is real, one seen once may be a depth artefact.
            conf[i] = Math.min(1f, count / 5f);
            i++;
        }
        return new PointCloud(xyz, conf, n);
    }
}
