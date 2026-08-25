package com.antu.drivers.imu;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Handler;
import android.os.HandlerThread;

import com.antu.core.geometry.Vec3;
import com.antu.core.graph.Out;
import com.antu.core.log.Log;
import com.antu.core.msg.ImuSample;
import com.antu.core.node.Node;
import com.antu.core.time.Stamp;

import java.util.concurrent.atomic.AtomicReference;

/**
 * The phone's own inertial sensors, as a node.
 *
 * <p>The second IMU on this robot. The base has a gyro of its own, and the two
 * sit on different bodies with different mounting and different noise, so they
 * publish separately and any fusion is a deliberate step rather than an accident
 * of two drivers sharing a name.
 *
 * <p>The phone's is the better sensor — MEMS parts in a modern phone are good,
 * and it reports at 100 Hz or more against the base's 10 Hz — but it measures the
 * <em>phone's</em> motion. If the phone is on a mast it will pick up sway the
 * wheels never saw.
 *
 * <h2>Axes</h2>
 *
 * <p>Android reports in device axes, which depend on how the phone is held; the
 * rest of the system uses robot axes: x forward, y left, z up. The default
 * assumes the phone lies flat, screen up, with its top edge pointing forward —
 * the usual way a phone gets taped to a robot. That gives Android +y forward and
 * +x right, so the rotation is a quarter turn. Other mountings pass their own
 * {@link Mounting}.
 *
 * <p>Getting this wrong is quiet and awful: the robot turns left and the filter
 * believes it pitched. It is worth checking once by spinning the robot and
 * confirming the yaw rate is the axis that moves.
 */
public final class PhoneImuDriver extends Node {

    private static final String TAG = "imu";

    /** How the phone sits on the robot. */
    public enum Mounting {
        /** Flat, screen up, top edge forward. The default. */
        FLAT_TOP_FORWARD,
        /** Flat, screen up, top edge to the left. */
        FLAT_TOP_LEFT,
        /** Upright in a cradle, screen facing back, top edge up. */
        UPRIGHT_FACING_BACK,
        /** Device axes passed through unchanged, for a custom rig. */
        RAW
    }

    /** Inertial samples in robot axes. */
    public final Out<ImuSample> imu = out("imu", ImuSample.class);

    private final android.content.Context context;
    private final Mounting mounting;
    private final int samplingPeriodUs;

    /** Latest reading from each sensor, written by the sensor thread. */
    private final AtomicReference<Vec3> gyro = new AtomicReference<>(Vec3.ZERO);
    private final AtomicReference<Vec3> accel = new AtomicReference<>(Vec3.ZERO);
    private final AtomicReference<Double> heading = new AtomicReference<>(Double.NaN);
    private final AtomicReference<Stamp> stamp = new AtomicReference<>(Stamp.ZERO);

    private SensorManager sensors;
    private HandlerThread thread;
    private Listener listener;
    private volatile boolean sawSample;

    /** A flat-mounted phone sampled at about 100 Hz. */
    public PhoneImuDriver(android.content.Context context) {
        this(context, Mounting.FLAT_TOP_FORWARD, 10_000);
    }

    /**
     * @param samplingPeriodUs requested period in microseconds; Android treats it
     *        as a hint and often delivers faster
     */
    public PhoneImuDriver(android.content.Context context, Mounting mounting,
                          int samplingPeriodUs) {
        super("phone_imu");
        this.context = context.getApplicationContext();
        this.mounting = mounting;
        this.samplingPeriodUs = samplingPeriodUs;
    }

    /** True once any sample has arrived, for the diagnostics page. */
    public boolean isReporting() {
        return sawSample;
    }

    @Override public void start(Node.Context ctx) {
        sensors = (SensorManager) context.getSystemService(android.content.Context.SENSOR_SERVICE);
        if (sensors == null) {
            Log.w(TAG, "no sensor service; phone IMU will report nothing", null);
            return;
        }

        // Its own thread: sensor callbacks must not land on the graph's tick loop,
        // and at 100 Hz they would be the busiest thing on it.
        thread = new HandlerThread("antu-imu");
        thread.start();
        Handler handler = new Handler(thread.getLooper());
        listener = new Listener();

        register(Sensor.TYPE_GYROSCOPE, handler, "gyroscope");
        register(Sensor.TYPE_ACCELEROMETER, handler, "accelerometer");
        // Fused orientation, so heading is available without integrating the gyro
        // and watching it drift. Absent on some devices, which is not fatal.
        register(Sensor.TYPE_ROTATION_VECTOR, handler, "rotation vector");
    }

    private void register(int type, Handler handler, String label) {
        Sensor sensor = sensors.getDefaultSensor(type);
        if (sensor == null) {
            Log.w(TAG, "no " + label + " on this device", null);
            return;
        }
        sensors.registerListener(listener, sensor, samplingPeriodUs, handler);
    }

    @Override public void tick(Node.Context ctx) {
        if (!sawSample) {
            return;
        }
        // Published at the node's rate rather than the sensor's. A 200 Hz sensor
        // feeding a 200 Hz channel would cost more in envelopes than the data is
        // worth to anything currently reading it; a filter that needs every sample
        // should read the sensor directly.
        imu.publish(new ImuSample(gyro.get(), accel.get(), heading.get()), stamp.get());
    }

    @Override public void stop() {
        if (sensors != null && listener != null) {
            sensors.unregisterListener(listener);
        }
        listener = null;
        if (thread != null) {
            thread.quitSafely();
            thread = null;
        }
    }

    /** Rotates a device-axes vector into robot axes: x forward, y left, z up. */
    private Vec3 toRobotAxes(float[] v) {
        switch (mounting) {
            case FLAT_TOP_FORWARD:
                // Android +y is forward, +x is right. Robot +y is left.
                return new Vec3(v[1], -v[0], v[2]);
            case FLAT_TOP_LEFT:
                return new Vec3(-v[0], -v[1], v[2]);
            case UPRIGHT_FACING_BACK:
                // Screen faces back, so device +z points backwards and +y is up.
                return new Vec3(-v[2], -v[0], v[1]);
            case RAW:
            default:
                return new Vec3(v[0], v[1], v[2]);
        }
    }

    private final class Listener implements SensorEventListener {

        private final float[] rotationMatrix = new float[9];
        private final float[] orientation = new float[3];

        @Override public void onSensorChanged(SensorEvent event) {
            switch (event.sensor.getType()) {
                case Sensor.TYPE_GYROSCOPE:
                    gyro.set(toRobotAxes(event.values));
                    // The gyro's timestamp, not the moment this callback ran: the
                    // delivery delay is variable and would show up as jitter in
                    // anything integrating these samples.
                    stamp.set(Stamp.ofNanos(event.timestamp));
                    sawSample = true;
                    break;

                case Sensor.TYPE_ACCELEROMETER:
                    accel.set(toRobotAxes(event.values));
                    sawSample = true;
                    break;

                case Sensor.TYPE_ROTATION_VECTOR:
                    SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values);
                    SensorManager.getOrientation(rotationMatrix, orientation);
                    // Android's azimuth grows clockwise from north; robot headings
                    // grow counter-clockwise, so the sign flips.
                    heading.set(-(double) orientation[0]);
                    break;

                default:
                    break;
            }
        }

        @Override public void onAccuracyChanged(Sensor sensor, int accuracy) {
            if (accuracy == SensorManager.SENSOR_STATUS_UNRELIABLE) {
                Log.w(TAG, sensor.getName() + " reports unreliable readings", null);
            }
        }
    }
}
