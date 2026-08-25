package com.antu.drivers.depth;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import com.antu.core.geometry.Vec3;
import com.antu.core.graph.In;
import com.antu.core.graph.Out;
import com.antu.core.log.Log;
import com.antu.core.msg.PointCloud;
import com.antu.core.msg.PosedFrame;
import com.antu.core.node.Node;

import ai.onnxruntime.OnnxTensor;
import ai.onnxruntime.OrtEnvironment;
import ai.onnxruntime.OrtSession;

import java.io.File;
import java.nio.FloatBuffer;
import java.util.Collections;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Monocular metric depth, turned into world-frame points.
 *
 * <p>This is the sensor the robot was missing. Its sonar ring is deaf and the
 * phone has no depth camera, so a picture and a model are the only way it can
 * learn that something is in front of it. The model is Depth Anything V2's
 * indoor metric checkpoint, which returns real metres rather than the relative
 * inverse depth the general checkpoint gives — the difference between a map with
 * a scale and a pretty picture.
 *
 * <h2>It runs on its own thread, and slowly</h2>
 *
 * <p>Measured at about four seconds per frame on this phone, alongside ARCore —
 * forty ticks of the control loop. The checkpoint's export has a fixed 518 square
 * input, so there is no smaller size to ask for. It gets a thread of its own and
 * takes the newest frame whenever it is ready, dropping anything that arrived
 * meanwhile: queueing would only guarantee the robot reasons about where it used
 * to be.
 *
 * <p>It competes with ARCore for the same cores, and the node's tick rate does
 * not throttle it — see {@link #setRest}, which is what actually does. Watch
 * {@code overruns} in the node table: 85 with no rest, none with two and a half
 * seconds of it.
 *
 * <h2>The model is on the device, not in the APK</h2>
 *
 * <p>At about 99 MB, bundling it would mean pushing a hundred megabytes to change
 * a line of Java. {@code tools/push-depth-model.sh} puts it in place. Absent, the
 * node reports itself unavailable and everything else carries on.
 */
public final class DepthMapper extends Node {

    private static final String TAG = "depth";

    /** DINOv2 patches are 14 px, so the input side must be a multiple of 14. */
    private static final int SIDE = 518;
    /** The tensor this checkpoint expects. */
    private static final String INPUT = "pixel_values";
    /** ImageNet statistics the checkpoint was trained with. */
    private static final float[] MEAN = {0.485f, 0.456f, 0.406f};
    private static final float[] STD = {0.229f, 0.224f, 0.225f};

    /**
     * Take every Nth pixel when unprojecting.
     *
     * <p>518 squared is 268,000 points per frame, which is far more than a 5 cm
     * grid can distinguish and more than the bus should carry. Every eighth pixel
     * in each axis gives about 4,000, which fills a map just as fast.
     */
    private static final int STRIDE = 8;
    /** Beyond this the indoor checkpoint is extrapolating. */
    private static final double MAX_DEPTH_M = 8.0;
    /** Below this is the robot's own bodywork, or a model artefact. */
    private static final double MIN_DEPTH_M = 0.25;

    /** Camera frames with the pose they were taken from. */
    public final In<PosedFrame> frame = in("frame", PosedFrame.class);
    /** Depth, unprojected into the tracker's world frame. */
    public final Out<PointCloud> points = out("points", PointCloud.class);

    private final File modelFile;
    private final int threads;
    private volatile long restMillis = 2000;

    private OrtEnvironment env;
    private OrtSession session;
    private Thread worker;
    private volatile boolean running;
    private volatile String failure;
    private volatile long inferences;
    private volatile long lastMillis;

    /** Newest frame awaiting inference. Replaced, never queued. */
    private final AtomicReference<PosedFrame> pending = new AtomicReference<>();
    private final AtomicReference<PointCloud> result = new AtomicReference<>();

    public DepthMapper(File modelFile) {
        this(modelFile, 2);
    }

    /**
     * @param threads inference threads. Two of eight cores leaves room for ARCore
     *        and the control loop, which matter more than depth latency.
     */
    public DepthMapper(File modelFile, int threads) {
        super("depth");
        this.modelFile = modelFile;
        this.threads = threads;
    }

    /**
     * How long to idle between inferences, milliseconds.
     *
     * <p>The node's tick rate does not throttle this: the tick only hands frames
     * over, while inference runs on its own thread and a frame is always waiting.
     * Left alone it runs back to back at full duty and starves everything else —
     * measured at 85 loop overruns and every other node missing ticks, against
     * none before depth was enabled.
     *
     * <p>Depth is the slowest and least urgent thing on the robot. It can wait;
     * the control loop cannot.
     */
    public DepthMapper setRest(double seconds) {
        this.restMillis = (long) (seconds * 1000);
        return this;
    }

    /** Why depth is not running, or null when it is. */
    public String failure() {
        return failure;
    }

    public long inferences() {
        return inferences;
    }

    /** How long the last inference took, milliseconds. */
    public long lastMillis() {
        return lastMillis;
    }

    @Override public void start(Node.Context ctx) {
        if (modelFile == null || !modelFile.exists()) {
            failure = "model not found at "
                    + (modelFile == null ? "(unset)" : modelFile.getAbsolutePath());
            Log.w(TAG, failure + " — run tools/push-depth-model.sh", null);
            return;
        }
        try {
            env = OrtEnvironment.getEnvironment();
            OrtSession.SessionOptions options = new OrtSession.SessionOptions();
            options.setIntraOpNumThreads(threads);
            session = env.createSession(modelFile.getAbsolutePath(), options);
            failure = null;
        } catch (Throwable t) {
            failure = "could not load the model: " + t.getMessage();
            Log.e(TAG, failure, t);
            return;
        }
        running = true;
        worker = new Thread(this::loop, "antu-depth");
        worker.setDaemon(true);
        worker.start();
        Log.i(TAG, "depth model loaded, running at " + SIDE + " square");
    }

    @Override public void tick(Node.Context ctx) {
        if (frame.isFresh()) {
            // Newest only. A frame that arrived while the last one was running is
            // already stale by half a second.
            pending.set(frame.get());
        }
        PointCloud cloud = result.getAndSet(null);
        if (cloud != null) {
            points.publish(cloud);
        }
    }

    @Override public void stop() {
        running = false;
        Thread t = worker;
        worker = null;
        if (t != null) {
            t.interrupt();
            try {
                t.join(2000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        if (session != null) {
            try {
                session.close();
            } catch (Throwable ignored) {
                // Shutting down.
            }
            session = null;
        }
    }

    private void loop() {
        float[] input = new float[3 * SIDE * SIDE];
        while (running) {
            PosedFrame f = pending.getAndSet(null);
            if (f == null) {
                try {
                    Thread.sleep(30);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
                continue;
            }
            try {
                long t0 = System.nanoTime();
                float[][] depth = infer(f, input);
                lastMillis = (System.nanoTime() - t0) / 1_000_000L;
                if (depth != null) {
                    inferences++;
                    result.set(unproject(f, depth));
                }
            } catch (Throwable t) {
                Log.w(TAG, "inference failed: " + t, null);
            }
            // Give the cores back. Without this the loop takes the next frame
            // immediately and never stops competing with tracking and control.
            try {
                Thread.sleep(restMillis);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }

    /** Decode, centre-crop, resize, normalise, run. */
    private float[][] infer(PosedFrame f, float[] input) throws Exception {
        byte[] jpeg = f.image.jpeg();
        Bitmap full = BitmapFactory.decodeByteArray(jpeg, 0, jpeg.length);
        if (full == null) {
            return null;
        }
        // Centre crop to a square, because the model takes one and a stretched
        // image would bend the geometry it reports.
        int side = Math.min(full.getWidth(), full.getHeight());
        int ox = (full.getWidth() - side) / 2;
        int oy = (full.getHeight() - side) / 2;
        Bitmap square = Bitmap.createBitmap(full, ox, oy, side, side);
        Bitmap scaled = Bitmap.createScaledBitmap(square, SIDE, SIDE, true);

        int[] pixels = new int[SIDE * SIDE];
        scaled.getPixels(pixels, 0, SIDE, 0, 0, SIDE, SIDE);

        final int plane = SIDE * SIDE;
        for (int i = 0; i < plane; i++) {
            int p = pixels[i];
            // NCHW: all reds, then all greens, then all blues.
            input[i] = (((p >> 16) & 0xFF) / 255f - MEAN[0]) / STD[0];
            input[plane + i] = (((p >> 8) & 0xFF) / 255f - MEAN[1]) / STD[1];
            input[2 * plane + i] = ((p & 0xFF) / 255f - MEAN[2]) / STD[2];
        }
        if (scaled != square) {
            scaled.recycle();
        }
        if (square != full) {
            square.recycle();
        }
        full.recycle();

        try (OnnxTensor tensor = OnnxTensor.createTensor(
                    env, FloatBuffer.wrap(input), new long[] {1, 3, SIDE, SIDE});
             OrtSession.Result out = session.run(Collections.singletonMap(INPUT, tensor))) {
            return ((float[][][]) out.get(0).getValue())[0];
        }
    }

    /**
     * Depth image to world points.
     *
     * <p>Two corrections matter. The model saw a centre-cropped square scaled to
     * 518, so a model pixel has to be mapped back to a pixel of the original
     * image before the intrinsics mean anything — skipping that flattens or
     * stretches the scene depending on which way the crop went. And the pose is
     * the camera's, so the result lands in the tracker's world frame directly.
     */
    private PointCloud unproject(PosedFrame f, float[][] depth) {
        int imageSide = Math.min(f.image.width, f.image.height);
        int ox = (f.image.width - imageSide) / 2;
        int oy = (f.image.height - imageSide) / 2;
        double toOriginal = imageSide / (double) SIDE;

        int capacity = ((SIDE + STRIDE - 1) / STRIDE) * ((SIDE + STRIDE - 1) / STRIDE);
        float[] xyz = new float[capacity * 3];
        float[] conf = new float[capacity];
        int n = 0;

        for (int my = 0; my < SIDE; my += STRIDE) {
            float[] row = depth[my];
            for (int mx = 0; mx < SIDE; mx += STRIDE) {
                double d = row[mx];
                if (!(d > MIN_DEPTH_M) || d > MAX_DEPTH_M || Double.isNaN(d)) {
                    continue;
                }
                double px = ox + mx * toOriginal;
                double py = oy + my * toOriginal;
                double[] ray = f.rayThrough(px, py);

                // The ray points down -z with unit z, so scaling by depth gives a
                // point at that distance along the optical axis.
                Vec3 camera = new Vec3(ray[0] * d, ray[1] * d, -d);
                Vec3 world = f.cameraPose.apply(camera);

                xyz[n * 3] = (float) world.x;
                xyz[n * 3 + 1] = (float) world.y;
                xyz[n * 3 + 2] = (float) world.z;
                // Nearer readings are the ones to trust: monocular depth degrades
                // with distance, and the indoor checkpoint extrapolates past a few
                // metres.
                conf[n] = (float) Math.max(0.2, 1.0 - d / MAX_DEPTH_M);
                n++;
            }
        }
        return new PointCloud(xyz, conf, n);
    }
}
