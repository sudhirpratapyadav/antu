package com.antu.mapper;

import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.os.Build;
import android.os.IBinder;
import android.os.PowerManager;

import java.io.File;

import com.antu.brain.CloudMap;
import com.antu.brain.OccupancyMapper;
import com.antu.brain.PoseFusion;
import com.antu.core.geometry.Pose2;
import com.antu.core.geometry.Twist2;
import com.antu.core.graph.Graph;
import com.antu.core.graph.Out;
import com.antu.core.log.Log;
import com.antu.core.msg.Odometry;
import com.antu.core.node.Node;
import com.antu.core.time.Clock;
import com.antu.core.time.Rate;
import com.antu.drivers.ar.ArTrackerDriver;
import com.antu.drivers.depth.DepthMapper;
import com.antu.ops.OpsNode;

/**
 * The mapping graph, for a phone carried by hand rather than bolted to a robot.
 *
 * <p>Same nodes as the robot app minus the base: ARCore tracks, the depth
 * network turns keyframes into metric points, and the cloud and occupancy
 * nodes accumulate them. The point of a separate app is to work on the map
 * without a robot in the loop — walk the lab, watch the reconstruction, and
 * iterate on the mapping pipeline with nothing else able to be the problem.
 *
 * <p>Fusion still runs, fed a stationary odometry so its frame conventions
 * and its tracking-lost behaviour are exactly the robot's: the map built here
 * is in the same frame the robot will localise in later.
 */
public final class MapperService extends Service {

    private static final String TAG = "antu.mapper";
    private static final String CHANNEL = "antu.mapper";
    private static final int NOTIFICATION_ID = 1;
    private static final int API_PORT = 8080;
    private static final String DEPTH_MODEL = "da_metric_hypersim_small.onnx";
    /** Roughly where a hand holds a phone. Places the obstacle band on the floor. */
    private static final double HANDHELD_HEIGHT_M = 1.2;

    private static volatile Graph graph;
    private static volatile String startFailure;
    private static volatile ArTrackerDriver tracker;
    private static volatile DepthMapper depth;

    private PowerManager.WakeLock wakeLock;

    public static Graph graph() {
        return graph;
    }

    public static String startFailure() {
        return startFailure;
    }

    /**
     * Odometry for a robot that never moves: the phone in a hand has no wheels.
     *
     * <p>With this as its odometry, fusion outputs the tracker's pose while it
     * holds and freezes at the last fix when it drops — which is what a map
     * builder wants: integrate only while anchored, never from a guess.
     */
    private static final class HeldOdometry extends Node {
        final Out<Odometry> odom = out("odom", Odometry.class);
        private final Odometry still = new Odometry(Pose2.ORIGIN, Twist2.ZERO);

        HeldOdometry() {
            super("hand");
        }

        @Override public void tick(Context ctx) {
            odom.publish(still);
        }
    }

    private static String diagnostics() {
        StringBuilder sb = new StringBuilder("{\"arcore\":");
        ArTrackerDriver t = tracker;
        if (t == null) {
            sb.append("null");
        } else {
            sb.append("{\"running\":").append(t.isRunning())
              .append(",\"depthSupported\":").append(t.isDepthSupported())
              .append(",\"framesEncoded\":").append(t.encodedFrames())
              .append(",\"framesDropped\":").append(t.droppedFrames())
              .append(",\"failure\":")
              .append(t.failure() == null ? "null" : "\"" + t.failure() + "\"")
              .append('}');
        }
        DepthMapper d = depth;
        sb.append(",\"depth\":");
        if (d == null) {
            sb.append("null");
        } else {
            sb.append("{\"inferences\":").append(d.inferences())
              .append(",\"lastMs\":").append(d.lastMillis())
              .append(",\"failure\":")
              .append(d.failure() == null ? "null" : "\"" + d.failure() + "\"")
              .append('}');
        }
        return sb.append('}').toString();
    }

    @Override public IBinder onBind(Intent intent) {
        return null;
    }

    @Override public void onCreate() {
        super.onCreate();
        startForeground(NOTIFICATION_ID, notification());
        installLogSink();

        PowerManager pm = (PowerManager) getSystemService(Context.POWER_SERVICE);
        wakeLock = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "antu:mapper");
        wakeLock.acquire();

        try {
            ArTrackerDriver ar = new ArTrackerDriver(this);
            tracker = ar;
            HeldOdometry hand = new HeldOdometry();
            PoseFusion fusion = new PoseFusion();
            OccupancyMapper occupancy = new OccupancyMapper().setCameraHeight(HANDHELD_HEIGHT_M);
            CloudMap cloud = new CloudMap();
            DepthMapper dm = new DepthMapper(new File(getExternalFilesDir(null), DEPTH_MODEL), 2)
                    .setRest(2.5);
            depth = dm;
            OpsNode ops = new OpsNode(API_PORT, MapperService::graph, this::readAsset)
                    .withDiagnostics(MapperService::diagnostics)
                    .withShutdown(this::shutdown);

            Graph g = Graph.builder(Clock.SYSTEM)
                    .add(ar, Rate.hz(10))
                    .add(hand, Rate.hz(10))
                    .add(fusion, Rate.hz(10))
                    .add(dm, Rate.hz(1))
                    .add(cloud, Rate.hz(1))
                    .add(occupancy, Rate.hz(2))
                    .add(ops, Rate.hz(20))
                    .connect(ar.pose, fusion.tracked)
                    .connect(hand.odom, fusion.odom)
                    .connect(ar.frame, dm.frame)
                    .connect(dm.points, cloud.points)
                    .connect(fusion.pose, cloud.pose)
                    .connect(dm.points, occupancy.points)
                    .connect(fusion.pose, occupancy.pose)
                    .build();
            g.spin();
            graph = g;
            android.util.Log.i(TAG, "mapping graph running: " + g.nodes().size()
                    + " nodes; console on " + ops.address());
        } catch (Exception e) {
            android.util.Log.e(TAG, "graph failed to start", e);
            startFailure = e.getMessage() == null ? e.toString() : e.getMessage();
            stopSelf();
        }
    }

    private void shutdown() {
        new Thread(() -> {
            Graph g = graph;
            graph = null;
            if (g != null) {
                g.stop();
            }
            tracker = null;
            depth = null;
            stopSelf();
            MapperActivity.quit();
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            // ARCore and ONNX Runtime are native and stay loaded until the
            // process ends; a camera the system still believes claimed is what
            // makes the next start fail.
            System.exit(0);
        }, "antu-mapper-shutdown").start();
    }

    private byte[] readAsset(String path) {
        try (java.io.InputStream in = getAssets().open(path)) {
            java.io.ByteArrayOutputStream out = new java.io.ByteArrayOutputStream();
            byte[] chunk = new byte[8192];
            int n;
            while ((n = in.read(chunk)) > 0) {
                out.write(chunk, 0, n);
            }
            return out.toByteArray();
        } catch (java.io.IOException e) {
            return null;
        }
    }

    private void installLogSink() {
        Log.setSink((level, tag, message, error) -> {
            String t = TAG + "/" + tag;
            switch (level) {
                case DEBUG: android.util.Log.d(t, message); break;
                case INFO:  android.util.Log.i(t, message); break;
                case WARN:  android.util.Log.w(t, message); break;
                default:
                    if (error != null) {
                        android.util.Log.e(t, message, error);
                    } else {
                        android.util.Log.e(t, message);
                    }
            }
        });
    }

    @Override public int onStartCommand(Intent intent, int flags, int startId) {
        return START_STICKY;
    }

    @Override public void onDestroy() {
        Graph g = graph;
        graph = null;
        tracker = null;
        depth = null;
        if (g != null) {
            g.stop();
        }
        if (wakeLock != null && wakeLock.isHeld()) {
            wakeLock.release();
        }
        super.onDestroy();
    }

    private Notification notification() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            NotificationManager nm =
                    (NotificationManager) getSystemService(Context.NOTIFICATION_SERVICE);
            NotificationChannel channel = new NotificationChannel(
                    CHANNEL, "Mapper", NotificationManager.IMPORTANCE_LOW);
            channel.setShowBadge(false);
            nm.createNotificationChannel(channel);
            return new Notification.Builder(this, CHANNEL)
                    .setContentTitle("antu mapper")
                    .setContentText("mapping")
                    .setSmallIcon(android.R.drawable.ic_menu_mapmode)
                    .build();
        }
        return new Notification.Builder(this)
                .setContentTitle("antu mapper")
                .setContentText("mapping")
                .setSmallIcon(android.R.drawable.ic_menu_mapmode)
                .build();
    }
}
