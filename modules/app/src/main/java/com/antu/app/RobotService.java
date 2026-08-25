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
import android.util.Log;

import com.antu.core.exec.Graph;
import com.antu.core.time.Clock;
import com.antu.core.time.Rate;

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

    /** How often the graph reports itself to logcat. */
    private static final long STATS_PERIOD_MS = 5000;

    private PowerManager.WakeLock wakeLock;
    private Thread statsThread;

    public static Graph graph() {
        return graph;
    }

    @Override public IBinder onBind(Intent intent) {
        return null;
    }

    @Override public void onCreate() {
        super.onCreate();
        startForeground(NOTIFICATION_ID, notification());

        PowerManager pm = (PowerManager) getSystemService(Context.POWER_SERVICE);
        wakeLock = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "antu:graph");
        wakeLock.acquire();

        try {
            Graph g = new Graph(Clock.SYSTEM);
            // Phase one: one placeholder node. Drivers arrive next.
            g.add(new Heartbeat(), Rate.hz(10));
            g.spin();
            graph = g;
            Log.i(TAG, "graph running with " + g.nodes().size() + " node(s)");
            startStatsLogging();
        } catch (Exception e) {
            Log.e(TAG, "graph failed to start", e);
            stopSelf();
        }
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
                Log.i(TAG, sb.toString());
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
