package com.antu.app;

import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.os.Build;
import android.os.IBinder;
import android.os.PowerManager;

import com.antu.core.graph.Channel;
import com.antu.core.graph.Graph;
import com.antu.core.log.Log;
import com.antu.core.time.Clock;
import com.antu.core.time.Rate;
import com.antu.drivers.base.ArcosBaseDriver;
import com.antu.drivers.camera.CameraDriver;
import com.antu.drivers.imu.PhoneImuDriver;
import com.antu.ops.OpsNode;

import com.arcos.Transport;
import com.arcos.transport.SimTransport;
import com.arcos.transport.UsbSerialTransport;

/**
 * Hosts the graph for as long as the robot is running.
 *
 * <p>A foreground service rather than an activity because a robot must keep
 * driving with the screen off, and Android will freeze or kill anything else.
 * The wake lock is not belt-and-braces: without it the CPU sleeps and the control
 * loop stops mid-manoeuvre, which is the runaway case the base driver's watchdog
 * exists to catch. Better not to provoke it.
 */
public final class RobotService extends Service {

    private static final String TAG = "antu";
    private static final String CHANNEL = "antu.robot";
    private static final int NOTIFICATION_ID = 1;

    /** The single running graph, so the UI can inspect it without binding. */
    private static volatile Graph graph;
    /** The base driver, for the console's motor and e-stop controls. */
    private static volatile ArcosBaseDriver baseDriver;
    /** The camera driver, so its diagnosis is reachable without adb. */
    private static volatile CameraDriver cameraDriver;

    /** Port for the operations API. */
    private static final int API_PORT = 8080;

    /** How often the graph reports itself to logcat. */
    private static final long STATS_PERIOD_MS = 5000;

    private PowerManager.WakeLock wakeLock;
    private Thread statsThread;

    public static Graph graph() {
        return graph;
    }

    /** The base driver, or null before the graph starts. */
    public static ArcosBaseDriver base() {
        return baseDriver;
    }

    /** The camera driver, or null before the graph starts. */
    public static CameraDriver camera() {
        return cameraDriver;
    }

    @Override public IBinder onBind(Intent intent) {
        return null;
    }

    @Override public void onCreate() {
        super.onCreate();
        startForeground(NOTIFICATION_ID, notification());
        installLogSink();

        PowerManager pm = (PowerManager) getSystemService(Context.POWER_SERVICE);
        wakeLock = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "antu:graph");
        wakeLock.acquire();

        try {
            // The whole robot, declared in one place. Wiring is checked by javac
            // where types allow and by build() where they do not: a missing
            // connection, two writers on one input, or a cycle without a delayed
            // edge all fail here rather than on a moving robot.
            ArcosBaseDriver base = new ArcosBaseDriver(this, chooseTransport());
            OpsNode ops = new OpsNode(API_PORT, RobotService::graph, this::readAsset)
                    .withBaseControls(
                            () -> base.enableMotors(true),
                            () -> base.enableMotors(false),
                            base::emergencyStop,
                            base::resetOdometry);

            // The phone's IMU is the better sensor and reports far faster than the
            // base's, but it measures the phone. Published separately so fusing
            // the two is a deliberate step, not an accident of naming.
            PhoneImuDriver phoneImu = new PhoneImuDriver(this);

            // 640x480 is a deliberate choice: enough to drive by, and small enough
            // that JPEG encoding keeps up on a phone that is also running the
            // control loop.
            CameraDriver camera = new CameraDriver(this, 640, 480, false);
            cameraDriver = camera;

            Graph g = Graph.builder(Clock.SYSTEM)
                    // The base runs at the rate ARCOS streams status; ops runs
                    // faster so a held teleop command is refreshed well inside the
                    // driver's silence timeout.
                    .add(base, Rate.hz(10))
                    .add(phoneImu, Rate.hz(50))
                    .add(camera, Rate.hz(15))
                    .add(ops, Rate.hz(20))
                    .connect(ops.cmdVel, base.cmdVel)
                    .build();

            baseDriver = base;
            g.spin();
            graph = g;
            android.util.Log.i(TAG, "graph running with " + g.nodes().size()
                    + " nodes, " + g.channels().size() + " channels; ops on " + ops.address());
            startStatsLogging();
        } catch (Exception e) {
            android.util.Log.e(TAG, "graph failed to start", e);
            stopSelf();
        }
    }

    /** Serves the web UI out of the APK's assets. */
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
            return null;                 // absent, which the caller reports as 404
        }
    }

    /** Routes core and brain logging into logcat, where it can actually be read. */
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

    /**
     * The real base if an adapter is plugged in, the simulator otherwise.
     *
     * <p>Falling back rather than failing means the graph, the UI and every node
     * above the driver can be developed and demonstrated with no robot present —
     * the same reason the simulator exists in arcos-android at all.
     */
    private Transport chooseTransport() {
        try {
            if (!UsbSerialTransport.available(this).isEmpty()) {
                android.util.Log.i(TAG, "using the USB serial adapter");
                return new UsbSerialTransport(this);
            }
        } catch (Exception e) {
            android.util.Log.w(TAG, "USB adapter unusable, falling back to the simulator: " + e.getMessage());
        }
        android.util.Log.i(TAG, "no USB adapter; using the simulator");
        return new SimTransport();
    }

    /**
     * Periodically writes the node and topic tables to logcat.
     *
     * <p>The screen is usually off or locked on a robot, and on MIUI a locked
     * screen screenshots as black, so the on-device console is not a reliable way
     * to see what the graph is doing. This is, and it costs one line every five
     * seconds. The web UI will supersede it.
     */
    private void startStatsLogging() {
        statsThread = new Thread(() -> {
            while (graph != null) {
                Graph g = graph;
                if (g == null) {
                    return;
                }
                StringBuilder sb = new StringBuilder();
                sb.append("loops=").append(g.loopCount())
                  .append(" overruns=").append(g.overruns());
                for (Graph.NodeInfo n : g.nodes()) {
                    sb.append(" | ").append(n.name)
                      .append(" ").append(n.rate)
                      .append(" ticks=").append(n.ticks)
                      .append(" missed=").append(n.missed)
                      .append(" errors=").append(n.errors);
                }
                for (Channel<?> ch : g.channels().values()) {
                    sb.append("\n    ").append(ch);
                }
                android.util.Log.i(TAG, sb.toString());
                try {
                    Thread.sleep(STATS_PERIOD_MS);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }, "antu-stats");
        statsThread.setDaemon(true);
        statsThread.start();
    }

    @Override public int onStartCommand(Intent intent, int flags, int startId) {
        // Restart if Android kills us for memory: a robot that silently stops
        // being controlled is worse than one that briefly restarts.
        return START_STICKY;
    }

    @Override public void onDestroy() {
        Graph g = graph;
        graph = null;
        baseDriver = null;
        cameraDriver = null;
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
                    CHANNEL, "Robot", NotificationManager.IMPORTANCE_LOW);
            channel.setShowBadge(false);
            nm.createNotificationChannel(channel);
            return new Notification.Builder(this, CHANNEL)
                    .setContentTitle("antu")
                    .setContentText("robot graph running")
                    .setSmallIcon(android.R.drawable.ic_media_play)
                    .build();
        }
        return new Notification.Builder(this)
                .setContentTitle("antu")
                .setContentText("robot graph running")
                .setSmallIcon(android.R.drawable.ic_media_play)
                .build();
    }
}
