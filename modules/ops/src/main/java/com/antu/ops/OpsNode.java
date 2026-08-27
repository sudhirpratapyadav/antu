package com.antu.ops;

import com.antu.core.geometry.Angles;
import com.antu.core.geometry.Twist2;
import com.antu.core.graph.Channel;
import com.antu.core.graph.Graph;
import com.antu.core.graph.Message;
import com.antu.core.graph.Out;
import com.antu.core.log.Log;
import com.antu.core.msg.PointCloud;
import com.antu.core.msg.PosedFrame;
import com.antu.core.msg.VideoFrame;
import com.antu.core.node.Node;

import java.io.IOException;
import java.net.Socket;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Supplier;

/**
 * The operations layer: introspection, telemetry and control over HTTP and
 * WebSocket.
 *
 * <p>One node rather than two. An API server and a bridge that both wanted to
 * command the robot were two writers on one input, which the graph builder now
 * refuses — correctly, since a robot taking velocity from two places stutters
 * between them. Merging them makes the single {@link #cmdVel} output honest, and
 * they were already sharing a server anyway.
 *
 * <h2>Generic over channels</h2>
 *
 * <p>There is no list of message types here and no per-channel branch. Clients
 * name channels and get whatever those channels carry, encoded by {@link Json}.
 * Wiring a new node into the graph makes its channels visible here with nothing
 * to add — no event enum, no dispatch case, no mirrored type in the client.
 *
 * <p>The static graph removed the awkward part. Every channel exists before
 * anything runs, so there is no "channel does not exist yet", no subscribing
 * before a publisher appears, and no catalogue that changes under a client.
 */
public final class OpsNode extends Node {

    private static final String TAG = "ops";
    /** Default cap on how fast one channel is streamed to one client. */
    private static final double DEFAULT_MAX_HZ = 20.0;
    /** How long a teleop command is held before it is treated as abandoned. */
    private static final double TELEOP_HOLD_SECONDS = 0.3;
    /** Multipart separator for the MJPEG stream. */
    private static final String MJPEG_BOUNDARY = "antuframe";

    /** The velocity command produced by whoever is driving through the API. */
    public final Out<Twist2> cmdVel = out("cmd_vel", Twist2.class);

    private final int port;
    private final Supplier<Graph> graphSupplier;
    private final AssetSource assets;
    private final HttpServer server;
    private final List<Client> clients = new ArrayList<>();

    private Node.Context ctx;
    /** Channel the MJPEG route reads by default. */
    private volatile String videoChannel = "ar.frame";
    private volatile Twist2 teleop = Twist2.ZERO;
    private volatile long teleopUntilNanos;
    private volatile Runnable onMotorsOn;
    private volatile Runnable onMotorsOff;
    private volatile Runnable onEmergencyStop;
    private volatile Runnable onResetOdometry;
    /**
     * Shuts the whole robot down, including this server.
     *
     * <p>Separate from the base controls because it is not a robot command: it
     * ends the process that is serving the request, so the route has to answer
     * before it runs.
     */
    private volatile Runnable onShutdown;
    /**
     * Driver-specific state the graph cannot know about — whether a camera
     * opened, whether depth is supported, why a transport will not connect.
     *
     * <p>Added because chasing these through logcat kept failing: the camera
     * subsystem floods the buffer and rotates the interesting line out within
     * seconds.
     */
    private volatile Supplier<String> diagnostics;

    /** Supplies the web UI's files, so the same code serves an APK or a directory. */
    public interface AssetSource {
        /** Bytes for {@code path}, or null when absent. */
        byte[] read(String path);
    }

    public OpsNode(int port, Supplier<Graph> graphSupplier) {
        this(port, graphSupplier, null);
    }

    public OpsNode(int port, Supplier<Graph> graphSupplier, AssetSource assets) {
        super("ops");
        this.port = port;
        this.graphSupplier = graphSupplier;
        this.assets = assets;
        this.server = new HttpServer(port);
    }

    /** Wires the base's controls, which are actions rather than channel values. */
    public OpsNode withBaseControls(Runnable motorsOn, Runnable motorsOff,
                                    Runnable emergencyStop, Runnable resetOdometry) {
        this.onMotorsOn = motorsOn;
        this.onMotorsOff = motorsOff;
        this.onEmergencyStop = emergencyStop;
        this.onResetOdometry = resetOdometry;
        return this;
    }

    /**
     * Wires the action that stops the robot software altogether.
     *
     * <p>Without this the only way to stop a phone-hosted robot is to pick the
     * phone up, and the phone is bolted to the robot with its USB port occupied
     * by the base — which is exactly when adb is not available either.
     */
    public OpsNode withShutdown(Runnable shutdown) {
        this.onShutdown = shutdown;
        return this;
    }

    /** Supplies a JSON object of driver state, served at /api/diag. */
    public OpsNode withDiagnostics(Supplier<String> supplier) {
        this.diagnostics = supplier;
        return this;
    }

    /**
     * Stops the robot and then the software running it.
     *
     * <p>Ordered deliberately. The base is brought to a halt and its motors
     * disabled first, synchronously, so that a robot which is moving when someone
     * hits quit is already stopped before anything starts tearing down — the
     * driver's disconnect would stop it too, but only after every node above it
     * has gone, and a rolling robot is not a good thing to leave to a shutdown
     * sequence.
     *
     * <p>The teardown itself runs on its own thread after a short pause, because
     * it stops this very server: run inline, the caller would get a dropped
     * connection instead of an answer and could not tell "shut down" from
     * "crashed".
     */
    private HttpServer.Response shutdown() {
        if (onShutdown == null) {
            return HttpServer.Response.error(501, "no shutdown action wired");
        }
        hold(Twist2.ZERO);
        run(onEmergencyStop);
        run(onMotorsOff);

        Thread t = new Thread(() -> {
            try {
                Thread.sleep(400);       // long enough for the response to flush
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            Log.i(TAG, "shutdown requested over the API");
            run(onShutdown);
        }, "antu-shutdown");
        t.setDaemon(true);
        t.start();
        return HttpServer.Response.text("shutting down\n");
    }

    /**
     * The cloud as packed binary, for the viewer.
     *
     * <p>Count, then positions, then colours — the layout jarvis's scan viewer
     * already reads. JSON was never an option here: five thousand points is
     * 60 KB packed and roughly ten times that as text, and the telemetry socket
     * has a control loop's worth of messages to carry as well.
     *
     * <p>Colour is by height, which makes a room legible at a glance: the floor
     * and the ceiling separate from the walls without anyone having to label
     * them.
     */
    private HttpServer.Response cloudBinary(String name) {
        Graph g = graphSupplier.get();
        if (g == null) {
            return HttpServer.Response.error(503, "graph not running");
        }
        Channel<?> ch = g.channel(name);
        if (ch == null || !PointCloud.class.isAssignableFrom(ch.type())) {
            return HttpServer.Response.notFound("no point cloud at " + name);
        }
        Message<?> latest = ch.latest();
        PointCloud c = latest == null ? PointCloud.empty() : (PointCloud) latest.payload();

        java.nio.ByteBuffer buf = java.nio.ByteBuffer
                .allocate(4 + c.size * 12 + c.size * 3)
                .order(java.nio.ByteOrder.LITTLE_ENDIAN);
        buf.putInt(c.size);

        double lowest = Double.MAX_VALUE;
        double highest = -Double.MAX_VALUE;
        for (int i = 0; i < c.size; i++) {
            lowest = Math.min(lowest, c.y(i));
            highest = Math.max(highest, c.y(i));
        }
        double span = Math.max(0.5, highest - lowest);

        for (int i = 0; i < c.size; i++) {
            buf.putFloat((float) c.x(i));
            buf.putFloat((float) c.y(i));
            buf.putFloat((float) c.z(i));
        }
        for (int i = 0; i < c.size; i++) {
            double t = (c.y(i) - lowest) / span;
            buf.put((byte) (int) (40 + 215 * t));                 // low is dark
            buf.put((byte) (int) (120 + 100 * Math.sin(Math.PI * t)));
            buf.put((byte) (int) (255 - 150 * t));                 // high is warm
        }
        return HttpServer.Response.bytes("application/octet-stream", buf.array());
    }

    /**
     * The accumulated cloud as a PLY file.
     *
     * <p>A download rather than a viewer. PLY opens in MeshLab, CloudCompare and
     * Blender, all of which inspect a point cloud far better than anything that
     * could reasonably be written here — and being able to open the map in a real
     * tool is what turns "the robot produced something" into "the robot produced
     * something correct".
     */
    private HttpServer.Response ply(String name) {
        Graph g = graphSupplier.get();
        if (g == null) {
            return HttpServer.Response.error(503, "graph not running");
        }
        Channel<?> ch = g.channel(name);
        if (ch == null || !PointCloud.class.isAssignableFrom(ch.type())) {
            return HttpServer.Response.notFound("no point cloud at " + name);
        }
        Message<?> latest = ch.latest();
        if (latest == null) {
            return HttpServer.Response.error(503, "nothing accumulated yet");
        }
        PointCloud c = (PointCloud) latest.payload();

        StringBuilder sb = new StringBuilder(c.size * 40 + 256);
        sb.append("ply\nformat ascii 1.0\n")
          .append("comment produced by antu: Depth-Anything metric + ARCore pose\n")
          .append("element vertex ").append(c.size).append('\n')
          .append("property float x\nproperty float y\nproperty float z\n")
          .append("property float confidence\n")
          .append("end_header\n");
        for (int i = 0; i < c.size; i++) {
            sb.append((float) c.x(i)).append(' ')
              .append((float) c.y(i)).append(' ')
              .append((float) c.z(i)).append(' ')
              .append((float) c.confidence(i)).append('\n');
        }
        try {
            return HttpServer.Response.bytes("application/octet-stream",
                    sb.toString().getBytes("UTF-8"));
        } catch (java.io.UnsupportedEncodingException e) {
            throw new IllegalStateException("UTF-8 is always supported", e);
        }
    }

    /** Names the channel served at /video.mjpeg. */
    public OpsNode withVideoChannel(String channelName) {
        this.videoChannel = channelName;
        return this;
    }

    /**
     * Serves a channel of {@link com.antu.core.msg.VideoFrame} as MJPEG.
     *
     * <p>Multipart rather than WebRTC or a WebSocket with Media Source
     * Extensions. Those are better on bandwidth and latency, and both cost a large
     * dependency; multipart works in an img tag in every browser, needs nothing,
     * and can be opened directly to see whether the camera is alive. On a robot's
     * own network that trade is worth taking.
     */
    private HttpServer.Response mjpeg(String name) {
        Graph g = graphSupplier.get();
        if (g == null) {
            return HttpServer.Response.error(503, "graph not running");
        }
        Channel<?> ch = g.channel(name);
        if (ch == null) {
            return HttpServer.Response.notFound("no such channel: " + name);
        }
        // Either a bare frame or one carrying its pose. The stream only needs the
        // pixels; the pose travels over telemetry, where a client can use it.
        final java.util.function.Function<Object, VideoFrame> extract;
        if (VideoFrame.class.isAssignableFrom(ch.type())) {
            extract = v -> (VideoFrame) v;
        } else if (PosedFrame.class.isAssignableFrom(ch.type())) {
            extract = v -> ((PosedFrame) v).image;
        } else {
            return HttpServer.Response.error(400,
                    name + " carries " + ch.type().getSimpleName() + ", which is not a frame");
        }

        @SuppressWarnings("unchecked")
        Channel<Object> video = (Channel<Object>) ch;
        return HttpServer.Response.stream(
                "multipart/x-mixed-replace; boundary=" + MJPEG_BOUNDARY,
                out -> streamVideo(video, extract, out));
    }

    private void streamVideo(Channel<Object> channel,
                             java.util.function.Function<Object, VideoFrame> extract,
                             java.io.OutputStream out) throws IOException {
        // A one-slot handoff, not a queue: only the newest frame is worth sending,
        // and a viewer on a slow link should see fewer frames rather than older
        // ones. Same reasoning as the camera driver's own backpressure.
        final java.util.concurrent.atomic.AtomicReference<VideoFrame> slot =
                new java.util.concurrent.atomic.AtomicReference<>();
        final Object wake = new Object();

        Channel.Listener<Object> listener = m -> {
            slot.set(extract.apply(m.payload()));
            synchronized (wake) {
                wake.notifyAll();
            }
        };
        channel.addListener(listener);
        try {
            long lastIndex = -1;
            while (true) {
                VideoFrame frame = slot.get();
                if (frame == null || frame.index == lastIndex) {
                    synchronized (wake) {
                        try {
                            // Wake on a new frame, but time out so a stalled camera
                            // does not leave this thread parked forever.
                            wake.wait(1000);
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                            return;
                        }
                    }
                    continue;
                }
                lastIndex = frame.index;

                byte[] jpeg = frame.jpeg();
                out.write(("--" + MJPEG_BOUNDARY + "\r\n"
                        + "Content-Type: image/jpeg\r\n"
                        + "Content-Length: " + jpeg.length + "\r\n\r\n").getBytes("UTF-8"));
                out.write(jpeg);
                out.write("\r\n".getBytes("UTF-8"));
                out.flush();
            }
        } finally {
            channel.removeListener(listener);
        }
    }

    /** Where the API is reachable, for the console to display. */
    public String address() {
        return HttpServer.localAddress() + ":" + port;
    }

    public int clientCount() {
        synchronized (clients) {
            return clients.size();
        }
    }

    @Override public void start(Node.Context context) throws Exception {
        this.ctx = context;
        MessageEncoders.installDefaults();

        server.route("/api/nodes", r -> HttpServer.Response.json(nodesJson()));
        server.route("/api/channels", r -> HttpServer.Response.json(channelsJson()));
        server.route("/api/channel", r -> channelJson(r.text("name", null)));
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
        server.route("/api/shutdown", r -> shutdown());
        server.route("/video.mjpeg", r -> mjpeg(r.text("channel", videoChannel)));
        server.route("/cloud.bin", r -> cloudBinary(r.text("channel", "cloud.cloud")));
        server.route("/cloud.ply", r -> ply(r.text("channel", "cloud.cloud")));
        server.route("/api/observe", r -> observe());
        server.route("/api/diag", r -> {
            Supplier<String> d = diagnostics;
            return HttpServer.Response.json(d == null ? "{}" : d.get());
        });
        server.upgrade("/ws", this::onUpgrade);
        server.fallback(this::asset);
        server.start();
    }

    @Override public void tick(Node.Context context) {
        long now = context.clock().now().nanos();
        Twist2 command = teleop;
        if (command.isZero() && now > teleopUntilNanos) {
            // Nothing to say. Publishing a zero forever would mask a planner that
            // has stopped, and the base driver's own timeout is the better judge.
            return;
        }
        if (now > teleopUntilNanos) {
            // The held command expired: a browser tab closed, or the link dropped
            // mid-drive. One zero, then silence.
            teleop = Twist2.ZERO;
            cmdVel.publish(Twist2.ZERO);
            return;
        }
        // Republished every tick, so a single dropped message cannot leave the
        // base holding a stale velocity.
        cmdVel.publish(command);
    }

    @Override public void stop() {
        synchronized (clients) {
            for (Client c : new ArrayList<>(clients)) {
                c.close();
            }
            clients.clear();
        }
        server.stop();
    }

    // ---------- HTTP ----------

    private HttpServer.Response drive(HttpServer.Request r) {
        // Metres and radians internally, but someone typing a URL thinks in mm/s
        // and deg/s, which is also what the robot's manual uses. Accept both.
        double linear = r.number("v", Double.NaN);
        double angular = r.number("w", Double.NaN);
        if (!Double.isNaN(r.number("mm", Double.NaN))) {
            linear = r.number("mm", 0) / 1000.0;
        }
        if (!Double.isNaN(r.number("deg", Double.NaN))) {
            angular = Angles.toRadians(r.number("deg", 0));
        }
        Twist2 command = Twist2.of(Double.isNaN(linear) ? 0 : linear,
                Double.isNaN(angular) ? 0 : angular);
        hold(command);
        return HttpServer.Response.text(String.format(
                "drive %.3f m/s (%.0f mm/s), %.3f rad/s (%.1f deg/s)%n",
                command.linearX, command.linearX * 1000,
                command.angular, Angles.toDegrees(command.angular)));
    }

    /** Holds a command briefly, so a lost client cannot leave it set. */
    private void hold(Twist2 command) {
        teleop = command;
        Node.Context c = ctx;
        if (c != null) {
            teleopUntilNanos = c.clock().now().nanos() + (long) (TELEOP_HOLD_SECONDS * 1e9);
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
        StringBuilder sb = new StringBuilder("{\"running\":").append(g.isRunning())
                .append(",\"loops\":").append(g.loopCount())
                .append(",\"overruns\":").append(g.overruns())
                .append(",\"clients\":").append(clientCount())
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

    private String channelsJson() {
        Graph g = graphSupplier.get();
        if (g == null) {
            return "{\"channels\":[]}";
        }
        StringBuilder sb = new StringBuilder("{\"channels\":[");
        boolean first = true;
        for (Channel<?> ch : g.channels().values()) {
            if (!first) {
                sb.append(',');
            }
            first = false;
            appendChannelInfo(sb, ch);
        }
        return sb.append("]}").toString();
    }

    private static void appendChannelInfo(StringBuilder sb, Channel<?> ch) {
        sb.append("{\"name\":").append(Json.quote(ch.name()))
          .append(",\"type\":").append(Json.quote(ch.type().getSimpleName()))
          .append(",\"published\":").append(ch.published())
          .append(",\"readers\":").append(ch.readerCount())
          .append('}');
    }

    private HttpServer.Response channelJson(String name) {
        if (name == null) {
            return HttpServer.Response.error(400, "usage: /api/channel?name=base.odom");
        }
        Graph g = graphSupplier.get();
        if (g == null) {
            return HttpServer.Response.error(503, "graph not running");
        }
        Channel<?> ch = g.channel(name);
        if (ch == null) {
            return HttpServer.Response.notFound("no such channel: " + name);
        }
        Message<?> latest = ch.latest();
        if (latest == null) {
            return HttpServer.Response.json(
                    "{\"name\":" + Json.quote(name) + ",\"value\":null}");
        }
        return HttpServer.Response.json("{\"name\":" + Json.quote(name)
                + ",\"stampNanos\":" + latest.stamp().nanos()
                + ",\"seq\":" + latest.sequence()
                + ",\"value\":" + Json.encode(latest.payload()) + "}");
    }

    /**
     * One observation, atomically: the newest posed camera frame together with
     * the fused pose and base state, as a single JSON object.
     *
     * <p>This is the endpoint an external agent polls — a VLM harness on a
     * laptop, a model behind an HTTP API. One request yields one self-contained
     * snapshot: no WebSocket subscription to manage, no pairing frames with
     * poses on the client, and the JPEG rides along base64-encoded because a
     * multimodal model wants it in that form anyway. At ~90 KB per call this is
     * not a streaming interface; it is a "look" primitive, made for a consumer
     * that thinks between looks.
     */
    private HttpServer.Response observe() {
        Graph g = graphSupplier.get();
        if (g == null) {
            return HttpServer.Response.error(503, "graph not running");
        }
        StringBuilder sb = new StringBuilder(140_000);
        sb.append('{');

        // Whichever frame source the graph has: posed frames when the tracker
        // runs, bare camera frames otherwise.
        Channel<?> frames = g.channel("ar.frame");
        if (frames == null) {
            frames = g.channel("camera.frame");
        }
        Message<?> m = frames == null ? null : frames.latest();
        sb.append("\"frame\":");
        if (m == null) {
            sb.append("null");
        } else {
            // The registered encoder supplies the metadata — size, camera pose,
            // intrinsics — and the pixels travel alongside it.
            Object payload = m.payload();
            VideoFrame image = payload instanceof PosedFrame
                    ? ((PosedFrame) payload).image : (VideoFrame) payload;
            sb.append("{\"stampNanos\":").append(m.stamp().nanos())
              .append(",\"seq\":").append(m.sequence())
              .append(",\"meta\":").append(Json.encode(payload))
              .append(",\"jpegBase64\":\"")
              .append(java.util.Base64.getEncoder().encodeToString(image.jpeg()))
              .append("\"}");
        }
        appendLatest(sb, g, "pose", "fusion.pose");
        appendLatest(sb, g, "odom", "base.odom");
        appendLatest(sb, g, "status", "base.status");
        return HttpServer.Response.json(sb.append('}').toString());
    }

    /** Appends {@code ,"key": <latest payload or null>} for a named channel. */
    private static void appendLatest(StringBuilder sb, Graph g, String key, String name) {
        Channel<?> ch = g.channel(name);
        Message<?> m = ch == null ? null : ch.latest();
        sb.append(",\"").append(key).append("\":")
          .append(m == null ? "null" : Json.encode(m.payload()));
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
        return "application/octet-stream";
    }

    // ---------- WebSocket ----------

    private void onUpgrade(Socket socket, HttpServer.Request request,
                           Map<String, String> headers) {
        WebSocket ws;
        try {
            ws = WebSocket.accept(socket, headers);
        } catch (IOException e) {
            Log.w(TAG, "handshake failed: " + e.getMessage(), null);
            try {
                socket.close();
            } catch (IOException ignored) {
                // Already gone.
            }
            return;
        }

        Client client = new Client(ws);
        synchronized (clients) {
            clients.add(client);
        }
        Log.i(TAG, "client connected (" + clientCount() + " total)");
        try {
            Graph g = graphSupplier.get();
            if (g != null) {
                // The catalogue immediately, and it is complete and final: every
                // channel exists before anything runs.
                client.send(catalogue(g));
            }
            String raw;
            while ((raw = ws.receive()) != null) {
                try {
                    handle(client, raw);
                } catch (RuntimeException e) {
                    // Never silently drop a connection over one bad message.
                    Log.w(TAG, "message handling failed: " + e, null);
                    client.send(error("HANDLER_FAILED", String.valueOf(e.getMessage())));
                }
            }
        } catch (IOException e) {
            Log.d(TAG, "client read ended: " + e.getMessage());
        } catch (RuntimeException e) {
            Log.e(TAG, "client loop failed", e);
        } finally {
            client.close();
            synchronized (clients) {
                clients.remove(client);
            }
            Log.i(TAG, "client disconnected (" + clientCount() + " remaining)");
        }
    }

    private void handle(Client client, String raw) {
        Map<String, Object> message = Json.parseObject(raw);
        String type = String.valueOf(message.get("type"));
        Map<String, Object> payload = asMap(message.get("payload"));

        switch (type) {
            case "subscribe":
                subscribe(client, payload);
                break;
            case "unsubscribe":
                for (String name : asStrings(payload.get("channels"))) {
                    client.unsubscribe(name);
                }
                break;
            case "drive":
                hold(Twist2.of(number(payload.get("linearX")), number(payload.get("angular"))));
                break;
            case "estop":
                hold(Twist2.ZERO);
                run(onEmergencyStop);
                break;
            case "ping":
                client.send("{\"type\":\"pong\",\"payload\":{}}");
                break;
            default:
                client.send(error("UNKNOWN_TYPE", "no such message type: " + type));
        }
    }

    private void subscribe(Client client, Map<String, Object> payload) {
        Graph g = graphSupplier.get();
        if (g == null) {
            client.send(error("NOT_RUNNING", "graph is not running"));
            return;
        }
        double maxHz = payload.get("maxHz") instanceof Number
                ? ((Number) payload.get("maxHz")).doubleValue()
                : DEFAULT_MAX_HZ;

        StringBuilder snapshot =
                new StringBuilder("{\"type\":\"snapshot\",\"payload\":{\"channels\":[");
        boolean first = true;
        for (String name : asStrings(payload.get("channels"))) {
            Channel<?> ch = g.channel(name);
            if (ch == null) {
                client.send(error("NO_SUCH_CHANNEL", name));
                continue;
            }
            client.subscribe(ch, maxHz);
            Message<?> latest = ch.latest();
            if (latest != null) {
                if (!first) {
                    snapshot.append(',');
                }
                first = false;
                appendMessage(snapshot, ch.name(), latest);
            }
        }
        client.send(snapshot.append("]}}").toString());
    }

    private String catalogue(Graph g) {
        StringBuilder sb = new StringBuilder("{\"type\":\"channels\",\"payload\":{\"channels\":[");
        boolean first = true;
        for (Channel<?> ch : g.channels().values()) {
            if (!first) {
                sb.append(',');
            }
            first = false;
            appendChannelInfo(sb, ch);
        }
        return sb.append("]}}").toString();
    }

    private static void appendMessage(StringBuilder sb, String name, Message<?> m) {
        sb.append("{\"channel\":").append(Json.quote(name))
          .append(",\"stampNanos\":").append(m.stamp().nanos())
          .append(",\"seq\":").append(m.sequence())
          .append(",\"value\":").append(Json.encode(m.payload()))
          .append('}');
    }

    private static String error(String code, String message) {
        return "{\"type\":\"error\",\"payload\":{\"code\":" + Json.quote(code)
                + ",\"message\":" + Json.quote(String.valueOf(message)) + "}}";
    }

    private static double number(Object value) {
        return value instanceof Number ? ((Number) value).doubleValue() : 0;
    }

    @SuppressWarnings("unchecked")
    private static Map<String, Object> asMap(Object o) {
        return o instanceof Map ? (Map<String, Object>) o : new HashMap<>();
    }

    private static List<String> asStrings(Object o) {
        List<String> out = new ArrayList<>();
        if (o instanceof List) {
            for (Object item : (List<?>) o) {
                out.add(String.valueOf(item));
            }
        } else if (o instanceof String) {
            out.add((String) o);
        }
        return out;
    }

    /**
     * One connected viewer.
     *
     * <p>Messages are queued and written by a thread of the client's own. The
     * channel listener runs on whichever thread published — for odometry that is
     * the serial driver's callback thread — and writing to a socket there let one
     * stalled viewer block the drive base and kill the app.
     */
    private static final class Client {

        private static final int OUTBOX_DEPTH = 256;
        /** Tells the sender thread to finish. Not a message anything would send. */
        private static final String POISON = "__antu_close__";

        private final WebSocket ws;
        private final Map<String, Runnable> detach = new ConcurrentHashMap<>();
        private final BlockingQueue<String> outbox = new ArrayBlockingQueue<>(OUTBOX_DEPTH);
        private final Thread sender;
        private volatile boolean closing;

        Client(WebSocket ws) {
            this.ws = ws;
            this.sender = new Thread(this::drain, "antu-ws-send");
            this.sender.setDaemon(true);
            this.sender.start();
        }

        private void drain() {
            try {
                while (!closing) {
                    String message = outbox.take();
                    if (POISON.equals(message)) {
                        return;
                    }
                    ws.send(message);
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            } catch (IOException e) {
                Log.d(TAG, "send failed, dropping client: " + e.getMessage());
            } finally {
                ws.close();
            }
        }

        <T> void subscribe(Channel<T> channel, double maxHz) {
            unsubscribe(channel.name());
            long minGapNanos = maxHz <= 0 ? 0 : (long) (1e9 / maxHz);
            // A separate flag rather than a sentinel timestamp. Seeding this with
            // Long.MIN_VALUE and subtracting overflows for any real wall-clock
            // stamp — 1.79e18 minus -9.22e18 does not fit in a long — so the
            // difference came out negative and every message was silently dropped
            // forever. Invisible under a ManualClock, which starts at zero.
            boolean[] sentAny = {false};
            long[] lastSentNanos = {0};

            Channel.Listener<T> listener = m -> {
                long stamp = m.stamp().nanos();
                synchronized (lastSentNanos) {
                    if (sentAny[0] && stamp - lastSentNanos[0] < minGapNanos) {
                        return;                  // dropped by the rate limit
                    }
                    sentAny[0] = true;
                    lastSentNanos[0] = stamp;
                }
                StringBuilder sb = new StringBuilder("{\"type\":\"msg\",\"payload\":");
                appendMessage(sb, channel.name(), m);
                send(sb.append('}').toString());
            };
            channel.addListener(listener);
            detach.put(channel.name(), () -> channel.removeListener(listener));
        }

        void unsubscribe(String name) {
            Runnable r = detach.remove(name);
            if (r != null) {
                r.run();
            }
        }

        /** Never blocks. A viewer that cannot keep up loses the oldest messages. */
        void send(String message) {
            if (closing) {
                return;
            }
            while (!outbox.offer(message)) {
                outbox.poll();
            }
        }

        void close() {
            if (closing) {
                return;
            }
            closing = true;
            for (Runnable r : detach.values()) {
                r.run();
            }
            detach.clear();
            outbox.clear();
            outbox.offer(POISON);
            sender.interrupt();
            ws.close();
        }
    }
}
