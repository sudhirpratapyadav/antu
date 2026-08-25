package com.antu.ops;

import com.antu.core.Topics;
import com.antu.core.bus.Bus;
import com.antu.core.bus.Message;
import com.antu.core.bus.Topic;
import com.antu.core.exec.Graph;
import com.antu.core.geometry.Angles;
import com.antu.core.geometry.Twist2;
import com.antu.core.log.Log;
import com.antu.core.node.AbstractNode;
import com.antu.core.node.Node;

import java.util.List;
import java.util.function.Supplier;

/**
 * The operations API: introspection and control over HTTP.
 *
 * <p>A node rather than a service the app starts separately, so it gets the
 * graph's lifecycle for free and shows up in the node table like everything else.
 *
 * <p>Two jobs. It answers what the graph is doing — nodes, topics, and the latest
 * value on any topic, encoded generically so a new message type appears without
 * anyone teaching the server about it. And it publishes {@code /cmd_vel}, so the
 * robot can be driven from a laptop while the phone's USB port holds the base.
 *
 * <p>Commands are GETs. On a trusted local network that makes the whole API
 * reachable with curl and a browser address bar, which is worth more here than
 * REST purity. It is not an API to expose beyond the robot's own network.
 */
public final class ApiNode extends AbstractNode {

    private static final String TAG = "api";
    /** Repeat rate for a held teleop command; see {@link #tick}. */
    private static final double TELEOP_HOLD_SECONDS = 0.3;

    private final int port;
    private final Supplier<Graph> graphSupplier;
    private final AssetSource assets;

    private HttpServer server;
    private Node.Context ctx;
    /** Latest command from HTTP, republished until it expires. */
    private volatile Twist2 teleop = Twist2.ZERO;
    private volatile long teleopUntilNanos;
    private volatile Runnable onMotorsOn;
    private volatile Runnable onMotorsOff;
    private volatile Runnable onEmergencyStop;
    private volatile Runnable onResetOdometry;

    /** Supplies the web UI's files, so the same code serves an APK or a directory. */
    public interface AssetSource {
        /** Bytes for {@code path}, or null when absent. */
        byte[] read(String path);
    }

    public ApiNode(int port, Supplier<Graph> graphSupplier) {
        this(port, graphSupplier, null);
    }

    public ApiNode(int port, Supplier<Graph> graphSupplier, AssetSource assets) {
        super("api");
        this.port = port;
        this.graphSupplier = graphSupplier;
        this.assets = assets;
    }

    /** Wires the base's controls, which are not expressible as bus messages. */
    public ApiNode withBaseControls(Runnable motorsOn, Runnable motorsOff,
                                    Runnable emergencyStop, Runnable resetOdometry) {
        this.onMotorsOn = motorsOn;
        this.onMotorsOff = motorsOff;
        this.onEmergencyStop = emergencyStop;
        this.onResetOdometry = resetOdometry;
        return this;
    }

    @Override public void start(Node.Context context) throws Exception {
        this.ctx = context;
        MessageEncoders.installDefaults();
        server = new HttpServer(port);
        server.route("/api/nodes", r -> HttpServer.Response.json(nodesJson()));
        server.route("/api/topics", r -> HttpServer.Response.json(topicsJson()));
        server.route("/api/topic", r -> topicJson(r.text("name", null)));
        server.route("/api/drive", this::drive);
        server.route("/api/stop", r -> {
            hold(Twist2.ZERO);
            return HttpServer.Response.text("stopped\n");
        });
        server.route("/api/estop", r -> {
            hold(Twist2.ZERO);
            run(onEmergencyStop);
            return HttpServer.Response.text("emergency stop\n");
        });
        server.route("/api/motors", r -> {
            boolean on = r.flag("on", true);
            run(on ? onMotorsOn : onMotorsOff);
            return HttpServer.Response.text("motors " + (on ? "on" : "off") + "\n");
        });
        server.route("/api/reset", r -> {
            run(onResetOdometry);
            return HttpServer.Response.text("odometry reset\n");
        });
        server.fallback(this::asset);
        server.start();
    }

    @Override public void tick(Node.Context context) {
        long now = context.clock().now().nanos();
        Twist2 command = teleop;
        if (command.isZero() && now > teleopUntilNanos) {
            // Nothing to say. Staying quiet lets the base driver's own timeout
            // decide what a silent /cmd_vel means, rather than this node asserting
            // a zero forever and masking a planner that has stopped publishing.
            return;
        }
        if (now > teleopUntilNanos) {
            // A held command has expired: the browser tab closed, or the network
            // dropped mid-drive. Publish one zero, then fall silent.
            teleop = Twist2.ZERO;
            context.publish(Topics.CMD_VEL, Twist2.ZERO);
            return;
        }
        // Republished every tick rather than once, so a single dropped message
        // cannot leave the base holding a stale velocity.
        context.publish(Topics.CMD_VEL, command);
    }

    @Override protected void onStop() {
        if (server != null) {
            server.stop();
            server = null;
        }
    }

    /** Where the API is reachable, for the console to display. */
    public String address() {
        return HttpServer.localAddress() + ":" + port;
    }

    // ---------- handlers ----------

    private HttpServer.Response drive(HttpServer.Request r) {
        // Metres and radians are the internal units, but a human typing a URL
        // thinks in mm/s and deg/s, which is also what the robot's own manual
        // uses. Accept both and be explicit in the reply about which was read.
        double linear = r.number("v", Double.NaN);
        double angular = r.number("w", Double.NaN);
        if (!Double.isNaN(r.number("mm", Double.NaN))) {
            linear = r.number("mm", 0) / 1000.0;
        }
        if (!Double.isNaN(r.number("deg", Double.NaN))) {
            angular = Angles.toRadians(r.number("deg", 0));
        }
        if (Double.isNaN(linear)) {
            linear = 0;
        }
        if (Double.isNaN(angular)) {
            angular = 0;
        }
        Twist2 command = Twist2.of(linear, angular);
        hold(command);
        return HttpServer.Response.text(String.format(
                "drive %.3f m/s (%.0f mm/s), %.3f rad/s (%.1f deg/s)%n",
                command.linearX, command.linearX * 1000,
                command.angular, Angles.toDegrees(command.angular)));
    }

    /** Holds a command for a short window, so a lost client cannot leave it set. */
    private void hold(Twist2 command) {
        teleop = command;
        Node.Context c = ctx;
        if (c != null) {
            teleopUntilNanos = c.clock().now().nanos()
                    + (long) (TELEOP_HOLD_SECONDS * 1e9);
        }
    }

    private static void run(Runnable action) {
        if (action != null) {
            action.run();
        }
    }

    private String nodesJson() {
        Graph g = graphSupplier.get();
        if (g == null) {
            return "{\"running\":false}";
        }
        StringBuilder sb = new StringBuilder();
        sb.append("{\"running\":").append(g.isRunning())
          .append(",\"loops\":").append(g.loopCount())
          .append(",\"overruns\":").append(g.overruns())
          .append(",\"nodes\":[");
        List<Graph.NodeInfo> nodes = g.nodes();
        for (int i = 0; i < nodes.size(); i++) {
            Graph.NodeInfo n = nodes.get(i);
            if (i > 0) {
                sb.append(',');
            }
            sb.append("{\"name\":").append(Json.quote(n.name))
              .append(",\"hz\":").append(String.format("%.1f", n.rate.hz()))
              .append(",\"ticks\":").append(n.ticks)
              .append(",\"missed\":").append(n.missed)
              .append(",\"errors\":").append(n.errors)
              .append(",\"started\":").append(n.started)
              .append('}');
        }
        return sb.append("]}").toString();
    }

    private String topicsJson() {
        Graph g = graphSupplier.get();
        if (g == null) {
            return "{\"topics\":[]}";
        }
        StringBuilder sb = new StringBuilder("{\"topics\":[");
        List<Bus.TopicInfo> topics = g.bus().topics();
        for (int i = 0; i < topics.size(); i++) {
            Bus.TopicInfo t = topics.get(i);
            if (i > 0) {
                sb.append(',');
            }
            sb.append("{\"name\":").append(Json.quote(t.name))
              .append(",\"type\":").append(Json.quote(t.type))
              .append(",\"published\":").append(t.published)
              .append(",\"subscribers\":").append(t.subscribers)
              .append(",\"lastStampNanos\":").append(t.lastStamp.nanos())
              .append('}');
        }
        return sb.append("]}").toString();
    }

    /** The latest value on a topic, encoded from whatever type it happens to be. */
    private HttpServer.Response topicJson(String name) {
        if (name == null) {
            return HttpServer.Response.error(400, "usage: /api/topic?name=/odom");
        }
        Graph g = graphSupplier.get();
        if (g == null) {
            return HttpServer.Response.error(503, "graph not running");
        }
        for (Bus.TopicInfo info : g.bus().topics()) {
            if (!info.name.equals(name)) {
                continue;
            }
            // The registry is keyed by name and the payload type is fixed at first
            // publish, so reading through a wildcard topic is safe here.
            Topic<Object> topic = Topic.of(info.name, Object.class);
            Message<Object> latest = readLatest(g.bus(), topic);
            if (latest == null) {
                return HttpServer.Response.json(
                        "{\"name\":" + Json.quote(name) + ",\"value\":null}");
            }
            return HttpServer.Response.json("{\"name\":" + Json.quote(name)
                    + ",\"stampNanos\":" + latest.stamp().nanos()
                    + ",\"sequence\":" + latest.sequence()
                    + ",\"value\":" + Json.encode(latest.payload()) + "}");
        }
        return HttpServer.Response.notFound("no such topic: " + name);
    }

    @SuppressWarnings("unchecked")
    private static Message<Object> readLatest(Bus bus, Topic<Object> topic) {
        try {
            return bus.latest(topic);
        } catch (IllegalArgumentException e) {
            // The type check rejected the wildcard; nothing useful to return.
            return null;
        }
    }

    private HttpServer.Response asset(HttpServer.Request r) {
        String path = "/".equals(r.path) ? "/index.html" : r.path;
        if (path.contains("..")) {
            return HttpServer.Response.error(400, "bad path");
        }
        if (assets != null) {
            byte[] body = assets.read(path.substring(1));
            if (body != null) {
                return HttpServer.Response.bytes(contentType(path), body);
            }
        }
        return HttpServer.Response.notFound("not found: " + r.path);
    }

    private static String contentType(String path) {
        if (path.endsWith(".html")) {
            return "text/html";
        }
        if (path.endsWith(".js")) {
            return "application/javascript";
        }
        if (path.endsWith(".css")) {
            return "text/css";
        }
        if (path.endsWith(".json")) {
            return "application/json";
        }
        if (path.endsWith(".png")) {
            return "image/png";
        }
        if (path.endsWith(".svg")) {
            return "image/svg+xml";
        }
        return "application/octet-stream";
    }
}
