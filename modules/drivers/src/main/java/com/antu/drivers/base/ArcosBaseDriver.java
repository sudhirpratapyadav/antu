package com.antu.drivers.base;

import com.antu.core.Topics;
import com.antu.core.geometry.Angles;
import com.antu.core.geometry.Pose2;
import com.antu.core.geometry.Twist2;
import com.antu.core.geometry.Vec3;
import com.antu.core.msg.BaseStatus;
import com.antu.core.msg.ImuSample;
import com.antu.core.msg.Odometry;
import com.antu.core.msg.RangeScan;
import com.antu.core.log.Log;
import com.antu.core.node.AbstractNode;
import com.antu.core.node.Node;
import com.antu.core.time.Stamp;

import com.arcos.ArcosListener;
import com.arcos.ArcosRobot;
import com.arcos.RobotInfo;
import com.arcos.RobotState;
import com.arcos.Transport;
import com.arcos.transport.UsbPermission;
import com.arcos.transport.UsbSerialTransport;

import java.util.concurrent.atomic.AtomicReference;

/**
 * The Pioneer drive base, as a node.
 *
 * <p>Wraps {@code arcos-android}, which owns its own thread and calls back at
 * whatever rate the robot streams status. Rather than tick that library from the
 * graph, the callback publishes onto the bus and this node's tick sends the
 * latest command down. Two consequences worth being explicit about:
 *
 * <ul>
 *   <li>The graph's tick loop never blocks on serial I/O, which is the rule the
 *       scheduler is built around.
 *   <li>Sensor data carries the driver's own timestamp, taken when the packet was
 *       decoded, so a recording replays against the robot's timeline rather than
 *       the consumer's.
 * </ul>
 *
 * <h2>Units</h2>
 *
 * ARCOS speaks millimetres and degrees; everything above this line is metres and
 * radians. All conversion happens here and nowhere else.
 *
 * <h2>Safety</h2>
 *
 * The library already zeroes its setpoints if no command arrives for two seconds,
 * which covers the app freezing. This node adds the case that watchdog cannot
 * see: {@code /cmd_vel} going quiet while the app is still healthy — a planner
 * that died, or a teleop client that dropped off the network. After
 * {@link #setCommandTimeout} the base is commanded to stop.
 */
public final class ArcosBaseDriver extends AbstractNode {

    private static final String TAG = "base";

    /** Millimetres per metre. */
    private static final double MM_PER_M = 1000.0;
    /** Distance between the driven wheels on a P3-DX, metres. */
    private static final double DEFAULT_WHEELBASE = 0.33;
    /** Sonar bearings on a Pioneer front ring, degrees from straight ahead. */
    private static final double[] SONAR_BEARINGS_DEG = {90, 50, 30, 10, -10, -30, -50, -90};
    /** The ring is not trusted outside this band, metres. */
    private static final double SONAR_MIN_M = 0.10;
    private static final double SONAR_MAX_M = 4.50;
    /**
     * The value a Pioneer transducer reports when nothing echoed back, in mm.
     *
     * <p>It is the ring's ceiling, not a measurement: an angled surface reflects
     * the pulse away and reads exactly the same as an empty room. Passing it
     * upstream as a 5 m reading would invite a costmap to treat it as free space.
     */
    private static final int SONAR_NO_ECHO_MM = 5000;
    /** Wait between connection attempts. */
    private static final long RECONNECT_BACKOFF_NANOS = 3_000_000_000L;
    /** Longer wait after the user refuses USB access, so we do not spam dialogs. */
    private static final long DENIED_BACKOFF_NANOS = 30_000_000_000L;
    /**
     * How long to wait on a permission dialog before assuming no answer is coming.
     *
     * <p>A dialog swiped away, or killed with the app in the background, never
     * invokes the callback. Without this the driver waits for an event that will
     * never arrive and the base stays dead until someone restarts the app.
     */
    private static final long PERMISSION_TIMEOUT_NANOS = 60_000_000_000L;

    /**
     * Fully qualified throughout this file: {@code AbstractNode} inherits
     * {@code Node.Context}, and an inherited member type shadows a single-type
     * import, so a bare {@code Context} here means the wrong one.
     */
    private final android.content.Context context;
    private final Transport transport;
    private final double wheelbase;

    /** Latest command, written by the bus thread and read by the tick. */
    private final AtomicReference<Twist2> command = new AtomicReference<>(Twist2.ZERO);

    private ArcosRobot robot;
    private volatile Node.Context ctx;
    /** When the next connection attempt is allowed. */
    private long nextAttemptNanos;
    /** A permission dialog is open; do not raise another. */
    private volatile boolean permissionPending;
    /** When that dialog was raised, so a lost answer can be detected. */
    private volatile long permissionAskedAtNanos;
    private volatile boolean connecting;
    private volatile long lastCommandNanos;
    private volatile boolean stoppedForSilence;
    private long commandTimeoutNanos = 1_500_000_000L;
    private boolean enableMotorsOnConnect;

    /** Odometry is reported relative to the first packet, so it starts at zero. */
    private Pose2 lastPose;
    private Stamp lastPoseStamp;

    public ArcosBaseDriver(android.content.Context context, Transport transport) {
        this(context, transport, DEFAULT_WHEELBASE);
    }

    public ArcosBaseDriver(android.content.Context context, Transport transport,
                           double wheelbaseMetres) {
        super("base");
        this.context = context.getApplicationContext();
        this.transport = transport;
        this.wheelbase = wheelbaseMetres;
    }

    /** Stops the base if {@code /cmd_vel} goes quiet for this long. 0 disables. */
    public ArcosBaseDriver setCommandTimeout(double seconds) {
        this.commandTimeoutNanos = (long) (seconds * 1e9);
        return this;
    }

    /**
     * Whether to enable the motors as soon as the base connects.
     *
     * <p>Off by default, deliberately. A robot that becomes drivable the instant
     * an app starts is a robot that moves when someone was only checking the
     * battery.
     */
    public ArcosBaseDriver setEnableMotorsOnConnect(boolean enabled) {
        this.enableMotorsOnConnect = enabled;
        return this;
    }

    /** The wrapped library, for the diagnostics page. Null before {@link #start}. */
    public ArcosRobot robot() {
        return robot;
    }

    @Override public void start(Node.Context context) {
        this.ctx = context;
        this.lastPose = null;
        this.lastPoseStamp = null;
        this.lastCommandNanos = context.clock().now().nanos();
        this.stoppedForSilence = false;

        context.subscribe(Topics.CMD_VEL, m -> {
            command.set(m.payload());
            lastCommandNanos = ctx.clock().now().nanos();
            stoppedForSilence = false;
        });

        robot = new ArcosRobot(transport);
        // The library's own watchdog would fight this node's, and this node is the
        // one that knows whether /cmd_vel is still flowing.
        robot.setCommandTimeout(0);
        robot.addListener(new Callbacks());
        nextAttemptNanos = context.clock().now().nanos();
    }

    @Override public void tick(Node.Context context) {
        ArcosRobot r = robot;
        if (r == null) {
            return;
        }
        if (!r.isConnected()) {
            // A robot whose cable is knocked, or whose USB permission arrives a
            // moment later, must recover on its own. Needing an app restart to
            // regain a drive base is not acceptable on something that moves.
            maybeConnect(context, r);
            return;
        }
        connecting = false;

        Twist2 wanted = command.get();

        // A silent /cmd_vel means whoever was steering has gone. Stop, and keep
        // stopping, until someone starts publishing again.
        if (commandTimeoutNanos > 0 && !wanted.isZero()) {
            long silentFor = context.clock().now().nanos() - lastCommandNanos;
            if (silentFor > commandTimeoutNanos) {
                if (!stoppedForSilence) {
                    stoppedForSilence = true;
                    command.set(Twist2.ZERO);
                }
                wanted = Twist2.ZERO;
            }
        }

        // The one place metres and radians become millimetres and degrees.
        r.drive(wanted.linearX * MM_PER_M, Angles.toDegrees(wanted.angular));
    }

    /** Drives reconnection, including asking for USB access when it is missing. */
    private void maybeConnect(Node.Context context, ArcosRobot r) {
        long now = context.clock().now().nanos();
        if (permissionPending) {
            if (now - permissionAskedAtNanos > PERMISSION_TIMEOUT_NANOS) {
                Log.w(TAG, "no answer to the USB permission dialog; will ask again");
                permissionPending = false;
                nextAttemptNanos = now + DENIED_BACKOFF_NANOS;
            }
            return;
        }
        if (connecting || now < nextAttemptNanos) {
            return;
        }
        if (!hasUsbPermission()) {
            requestUsbPermission(now);
            return;
        }
        connecting = true;
        nextAttemptNanos = now + RECONNECT_BACKOFF_NANOS;
        r.connect();
    }

    /** True when the transport is ready to open, or is not a USB one. */
    private boolean hasUsbPermission() {
        if (!(transport instanceof UsbSerialTransport)) {
            return true;
        }
        return UsbPermission.has(this.context, ((UsbSerialTransport) transport).device());
    }

    private void requestUsbPermission(long now) {
        permissionPending = true;
        permissionAskedAtNanos = now;
        // Back off before checking again either way; the answer arrives on another
        // thread and the tick must not spin waiting for it.
        nextAttemptNanos = now + RECONNECT_BACKOFF_NANOS;
        Log.i(TAG, "requesting USB access");
        UsbPermission.request(this.context, ((UsbSerialTransport) transport).device(),
                granted -> {
                    permissionPending = false;
                    if (granted) {
                        Log.i(TAG, "USB access granted");
                    } else {
                        Log.w(TAG, "USB access refused; backing off");
                        Node.Context c = ctx;
                        if (c != null) {
                            nextAttemptNanos = c.clock().now().nanos() + DENIED_BACKOFF_NANOS;
                        }
                    }
                });
    }

    @Override protected void onStop() {
        ArcosRobot r = robot;
        robot = null;
        if (r != null) {
            // disconnect() already sends STOP and CLOSE, so the base is not left
            // coasting on its own watchdog.
            r.disconnect();
        }
    }

    /** Enables or disables the motors. They come up disabled. */
    public void enableMotors(boolean enabled) {
        ArcosRobot r = robot;
        if (r != null) {
            r.enableMotors(enabled);
        }
    }

    /** Emergency stop: ignores the deceleration limit. */
    public void emergencyStop() {
        command.set(Twist2.ZERO);
        ArcosRobot r = robot;
        if (r != null) {
            r.eStop();
        }
    }

    /** Zeroes the base's odometry and this driver's accumulated pose. */
    public void resetOdometry() {
        lastPose = null;
        ArcosRobot r = robot;
        if (r != null) {
            r.resetOdometry();
        }
    }

    /** Translates the library's callbacks into bus messages. */
    private final class Callbacks implements ArcosListener {

        @Override public void onConnected(RobotInfo info) {
            Log.i(TAG, "connected to " + info.name + " (" + info.subtype + ") over "
                    + transport.name());
            if (!info.paramsRecognised) {
                // Worth shouting about: an unrecognised model means the odometry
                // scale is a guess, and a wrong guess is silently wrong.
                Log.w(TAG, "unrecognised model '" + info.subtype
                        + "'; odometry scale may be wrong");
            }
            if (enableMotorsOnConnect) {
                robot.enableMotors(true);
            }
        }

        @Override public void onDisconnected(String reason) {
            Log.w(TAG, "base disconnected: " + reason);
            connecting = false;
            // Nothing is steering a disconnected base; make sure a stale command
            // cannot be applied the instant it comes back.
            command.set(Twist2.ZERO);
        }

        @Override public void onError(Throwable error) {
            Log.e(TAG, "base error", error);
            connecting = false;
        }

        @Override public void onLog(String message) {
            // Handshake steps, baud switches, dropped frames. Debug rather than
            // info: useful when a robot will not talk, noise otherwise.
            Log.d(TAG, message);
        }

        @Override public void onState(RobotState s) {
            Node.Context c = ctx;
            if (c == null) {
                return;
            }
            // The library stamps each state; use it rather than "now", so that
            // serial latency does not get baked into the odometry timeline.
            Stamp stamp = Stamp.ofMillis(s.timestamp);

            Pose2 pose = new Pose2(s.x / MM_PER_M, s.y / MM_PER_M,
                    Angles.toRadians(s.theta));
            Twist2 velocity = Twist2.fromWheelSpeeds(
                    s.leftVel / MM_PER_M, s.rightVel / MM_PER_M, wheelbase);

            c.publish(Topics.ODOM, new Odometry(pose, velocity), stamp);
            lastPose = pose;
            lastPoseStamp = stamp;

            c.publish(Topics.BASE_STATUS, new BaseStatus(
                    s.batteryVoltage, s.motorsEnabled, s.eStopPressed, s.stalled(),
                    robotModel()), stamp);

            // The base's heading comes from its internal gyro, which is a genuinely
            // separate sensor from the phone's and worth publishing as one.
            c.publish(Topics.IMU_BASE, new ImuSample(
                    new Vec3(0, 0, velocity.angular), Vec3.ZERO,
                    Angles.toRadians(s.theta)), stamp);

            publishRanges(c, s, stamp);
        }

        private void publishRanges(Node.Context c, RobotState s, Stamp stamp) {
            int n = Math.min(SONAR_BEARINGS_DEG.length, s.sonar.length);
            double[] bearings = new double[n];
            double[] ranges = new double[n];
            boolean any = false;
            for (int i = 0; i < n; i++) {
                bearings[i] = Angles.toRadians(SONAR_BEARINGS_DEG[i]);
                int mm = s.sonar[i];
                // -1 means the transducer has not reported since connecting, and
                // the ring's ceiling means it heard nothing back. Neither is a
                // measurement, so both become NO_RETURN rather than a distance.
                ranges[i] = (mm < 0 || mm >= SONAR_NO_ECHO_MM)
                        ? RangeScan.NO_RETURN
                        : mm / MM_PER_M;
                any |= mm >= 0;
            }
            if (any) {
                c.publish(Topics.RANGES,
                        new RangeScan(bearings, ranges, SONAR_MIN_M, SONAR_MAX_M), stamp);
            }
        }

        private String robotModel() {
            RobotInfo info = robot == null ? null : robot.info();
            return info == null ? "unknown" : info.subtype;
        }
    }
}
