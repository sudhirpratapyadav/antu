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

import com.antu.core.bus.Bus;
import com.antu.core.exec.Graph;

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

    private final Runnable refresh = new Runnable() {
        @Override public void run() {
            render();
            ui.postDelayed(this, REFRESH_MS);
        }
    };

    @Override protected void onCreate(Bundle state) {
        super.onCreate(state);

        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setBackgroundColor(Color.parseColor("#0d1117"));
        int pad = (int) (16 * getResources().getDisplayMetrics().density);
        root.setPadding(pad, pad, pad, pad);

        root.addView(heading("antu"));
        root.addView(heading("nodes"));
        nodes = mono();
        root.addView(nodes);
        root.addView(heading("topics"));
        topics = mono();
        root.addView(topics);

        ScrollView scroll = new ScrollView(this);
        scroll.addView(root);
        setContentView(scroll);

        Intent service = new Intent(this, RobotService.class);
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            startForegroundService(service);
        } else {
            startService(service);
        }
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
            nodes.setText("service not running");
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
        for (Bus.TopicInfo info : g.bus().topics()) {
            t.append(info).append('\n');
        }
        topics.setText(t.length() == 0 ? "(none published yet)" : t.toString());
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
