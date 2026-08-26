package com.antu.app;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Color;
import android.graphics.Typeface;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.ViewGroup;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import com.antu.core.graph.Channel;
import com.antu.core.graph.Graph;

import java.util.Locale;

/**
 * A developer console, not a product UI.
 *
 * <p>The real interface will be the web app served from the ops module, viewable
 * on the phone and from any machine on the network. This exists so phase one is
 * visibly running on the device: it shows the node table and the topic list, the
 * same two things the web UI will show first.
 */
public final class MainActivity extends Activity {

    private static final int REFRESH_MS = 500;

    private final Handler ui = new Handler(Looper.getMainLooper());
    private TextView nodes;
    private TextView topics;

    /**
     * The visible console, so a shutdown started elsewhere can close it.
     *
     * <p>Weak on purpose: this is a static field holding an Activity, which is
     * the textbook way to leak one. A weak reference lets the activity be
     * collected normally and simply yields null if it already has.
     */
    private static java.lang.ref.WeakReference<MainActivity> instance;

    /**
     * Closes the console, from {@link RobotService}'s shutdown.
     *
     * <p>The service stopping is not enough to end the app: the activity keeps
     * the process alive, and quitting would leave a frozen console on screen
     * showing a graph that is no longer running. {@code finishAndRemoveTask}
     * rather than {@code finish} so the app also leaves the recents list, which
     * is what someone who pressed quit expects to see.
     */
    static void quit() {
        MainActivity a = instance == null ? null : instance.get();
        if (a != null) {
            a.ui.post(a::finishAndRemoveTask);
        }
    }

    private final Runnable refresh = new Runnable() {
        @Override public void run() {
            render();
            ui.postDelayed(this, REFRESH_MS);
        }
    };

    @Override protected void onCreate(Bundle state) {
        super.onCreate(state);
        instance = new java.lang.ref.WeakReference<>(this);

        // ARCore stops delivering frames the moment the display sleeps, and the
        // service's wake lock only holds the CPU. On a phone bolted to a robot
        // this activity is always the one in front, so keeping the screen on
        // here is what keeps the camera, the tracker and the video stream alive.
        getWindow().addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setBackgroundColor(Color.parseColor("#0d1117"));
        int pad = (int) (16 * getResources().getDisplayMetrics().density);
        root.setPadding(pad, pad, pad, pad);

        root.addView(heading("antu"));
        root.addView(heading("nodes"));
        nodes = mono();
        root.addView(nodes);
        root.addView(heading("channels"));
        topics = mono();
        root.addView(topics);

        ScrollView scroll = new ScrollView(this);
        scroll.addView(root);
        setContentView(scroll);

        // A service cannot raise a permission dialog, so the activity asks and the
        // camera driver reports itself unavailable until it is granted.
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M
                && checkSelfPermission(android.Manifest.permission.CAMERA)
                    != android.content.pm.PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[] {android.Manifest.permission.CAMERA}, 1);
        }

        Intent service = new Intent(this, RobotService.class);
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            startForegroundService(service);
        } else {
            startService(service);
        }
    }

    @Override protected void onDestroy() {
        if (instance != null && instance.get() == this) {
            instance = null;
        }
        super.onDestroy();
    }

    @Override protected void onResume() {
        super.onResume();
        ui.post(refresh);
    }

    @Override protected void onPause() {
        super.onPause();
        // The graph keeps running in the service; only the polling stops.
        ui.removeCallbacks(refresh);
    }

    private void render() {
        Graph g = RobotService.graph();
        if (g == null) {
            String why = RobotService.startFailure();
            nodes.setText(why == null ? "service not running"
                    : "service failed to start:\n" + why + "\n\nfix and relaunch");
            topics.setText("");
            return;
        }

        StringBuilder n = new StringBuilder();
        n.append(String.format(Locale.US, "loops=%d overruns=%d%n%n",
                g.loopCount(), g.overruns()));
        for (Graph.NodeInfo info : g.nodes()) {
            n.append(info).append('\n');
        }
        nodes.setText(n.toString());

        StringBuilder t = new StringBuilder();
        for (Channel<?> ch : g.channels().values()) {
            t.append(ch).append('\n');
        }
        topics.setText(t.length() == 0 ? "(no channels)" : t.toString());
    }

    private TextView heading(String text) {
        TextView v = new TextView(this);
        v.setText(text);
        v.setTextSize(15);
        v.setTextColor(Color.parseColor("#e6edf3"));
        v.setPadding(0, (int) (12 * getResources().getDisplayMetrics().density), 0, 4);
        return v;
    }

    private TextView mono() {
        TextView v = new TextView(this);
        v.setTypeface(Typeface.MONOSPACE);
        v.setTextSize(11);
        v.setTextColor(Color.parseColor("#8b949e"));
        v.setLayoutParams(new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT));
        return v;
    }
}
