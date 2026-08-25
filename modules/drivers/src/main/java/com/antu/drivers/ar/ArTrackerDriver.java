package com.antu.drivers.ar;

import android.opengl.EGL14;
import android.opengl.EGLConfig;
import android.opengl.EGLContext;
import android.opengl.EGLDisplay;
import android.opengl.EGLSurface;
import android.opengl.GLES11Ext;
import android.opengl.GLES20;

import com.antu.core.geometry.Pose3;
import com.antu.core.graph.Out;
import com.antu.core.log.Log;
import com.antu.core.msg.TrackedPose;
import com.antu.core.node.Node;
import com.antu.core.time.Stamp;

import com.google.ar.core.Camera;
import com.google.ar.core.Config;
import com.google.ar.core.Frame;
import com.google.ar.core.Pose;
import com.google.ar.core.Session;
import com.google.ar.core.TrackingFailureReason;
import com.google.ar.core.TrackingState;

import java.util.concurrent.atomic.AtomicReference;

/**
 * ARCore's visual-inertial tracking, as a node.
 *
 * <p>Gives the robot a pose that does not drift the way wheel odometry does,
 * because it is watching the room rather than counting wheel turns. That matters
 * here beyond accuracy: the sonar ring on this robot is deaf, so the camera is
 * the only sensor that can eventually answer what is in front of it, and a pose
 * for every frame is what turns those frames into a map.
 *
 * <h2>Running without an Activity</h2>
 *
 * <p>ARCore normally lives in an Activity driving a GLSurfaceView: it wants a GL
 * texture to write camera frames into, and {@code update()} must be called on the
 * thread owning that context. Neither actually needs a <em>window</em>. This
 * creates a 1x1 offscreen pbuffer context on its own thread and drives the
 * session from there, which is what lets tracking run in the same process as the
 * control loop with no Activity in sight.
 *
 * <p>Ported from the jarvis hub, where that arrangement was worked out. The
 * texture parameters below are the part that looks superfluous and is not:
 * generating a texture name is not enough, and without the target binding and
 * filters the vision pipeline stalls with tracking stuck at PAUSED while the IMU
 * keeps streaming — a failure that looks like a broken camera.
 *
 * <h2>It owns the camera</h2>
 *
 * <p>ARCore takes exclusive access, so {@code CameraDriver} cannot run alongside
 * it. That is a fair trade rather than a loss: frames arrive with the pose they
 * were taken from, which is the pairing reconstruction needs and which two
 * independent camera consumers could never produce.
 */
public final class ArTrackerDriver extends Node {

    private static final String TAG = "ar";
    /**
     * Stand-in display geometry. There is no window, but ARCore still wants a
     * geometry to orient frames against, so it gets the camera's own.
     */
    private static final int GEOM_W = 640;
    private static final int GEOM_H = 480;
    /** How long to wait for the tracking thread to wind down. */
    private static final int JOIN_MS = 2500;

    /** Camera pose in ARCore's world frame, with its tracking state. */
    public final Out<TrackedPose> pose = out("pose", TrackedPose.class);

    private final android.content.Context context;

    private Thread thread;
    private volatile boolean running;
    private volatile String failure;
    private final AtomicReference<TrackedPose> latest =
            new AtomicReference<>(new TrackedPose(Pose3.IDENTITY, TrackedPose.State.STOPPED, "", 0));

    public ArTrackerDriver(android.content.Context context) {
        super("ar");
        this.context = context.getApplicationContext();
    }

    /** Why tracking is not running, or null when it is. */
    public String failure() {
        return failure;
    }

    public boolean isRunning() {
        return running;
    }

    @Override public void start(Node.Context ctx) {
        running = true;
        failure = null;
        thread = new Thread(this::loop, "antu-arcore");
        thread.start();
    }

    @Override public void tick(Node.Context ctx) {
        // Published at the node's rate rather than ARCore's, so the channel has a
        // declared rate like everything else. The tracker runs at camera speed and
        // the newest estimate is the only one worth having.
        pose.publish(latest.get());
    }

    @Override public void stop() {
        running = false;
        Thread t = thread;
        thread = null;
        if (t != null) {
            try {
                t.join(JOIN_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        latest.set(new TrackedPose(latest.get().pose, TrackedPose.State.STOPPED, "", 0));
    }

    // ---------- the tracking thread ----------

    private void loop() {
        EGLDisplay display = EGL14.EGL_NO_DISPLAY;
        EGLContext eglContext = EGL14.EGL_NO_CONTEXT;
        EGLSurface surface = EGL14.EGL_NO_SURFACE;
        Session session = null;
        long frames = 0;

        try {
            // ---- offscreen GL, purely so ARCore has a texture to write into ----
            display = EGL14.eglGetDisplay(EGL14.EGL_DEFAULT_DISPLAY);
            int[] version = new int[2];
            if (!EGL14.eglInitialize(display, version, 0, version, 1)) {
                throw new IllegalStateException("eglInitialize failed");
            }

            EGLConfig[] configs = new EGLConfig[1];
            int[] count = new int[1];
            int[] attributes = {
                EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT,
                EGL14.EGL_SURFACE_TYPE, EGL14.EGL_PBUFFER_BIT,
                EGL14.EGL_NONE
            };
            if (!EGL14.eglChooseConfig(display, attributes, 0, configs, 0, 1, count, 0)
                    || count[0] == 0) {
                throw new IllegalStateException("eglChooseConfig failed");
            }
            eglContext = EGL14.eglCreateContext(display, configs[0], EGL14.EGL_NO_CONTEXT,
                    new int[] {EGL14.EGL_CONTEXT_CLIENT_VERSION, 2, EGL14.EGL_NONE}, 0);
            surface = EGL14.eglCreatePbufferSurface(display, configs[0],
                    new int[] {EGL14.EGL_WIDTH, 1, EGL14.EGL_HEIGHT, 1, EGL14.EGL_NONE}, 0);
            if (!EGL14.eglMakeCurrent(display, surface, surface, eglContext)) {
                throw new IllegalStateException("eglMakeCurrent failed");
            }

            // ARCore writes camera frames into an external-OES texture. Generating
            // a name is not enough: without the binding and these filters the
            // vision pipeline stalls and tracking sits at PAUSED forever while the
            // IMU keeps streaming, which reads as a broken camera.
            int[] texture = new int[1];
            GLES20.glGenTextures(1, texture, 0);
            final int target = GLES11Ext.GL_TEXTURE_EXTERNAL_OES;
            GLES20.glBindTexture(target, texture[0]);
            GLES20.glTexParameteri(target, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE);
            GLES20.glTexParameteri(target, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE);
            GLES20.glTexParameteri(target, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR);
            GLES20.glTexParameteri(target, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR);

            // ---- the session ----
            session = new Session(context);
            Config config = session.getConfig();
            config.setUpdateMode(Config.UpdateMode.BLOCKING);
            config.setFocusMode(Config.FocusMode.AUTO);
            config.setLightEstimationMode(Config.LightEstimationMode.DISABLED);
            // Plane finding costs CPU that the control loop and, later, the depth
            // model need more than this does.
            config.setPlaneFindingMode(Config.PlaneFindingMode.DISABLED);
            session.configure(config);

            session.setCameraTextureName(texture[0]);
            session.setDisplayGeometry(0, GEOM_W, GEOM_H);
            session.resume();
            Log.i(TAG, "ARCore session running");

            while (running) {
                Frame frame;
                try {
                    frame = session.update();
                } catch (Throwable t) {
                    failure = "update: " + t.getMessage();
                    Log.e(TAG, failure, t);
                    break;
                }
                frames++;

                Camera camera = frame.getCamera();
                TrackingState tracking = camera.getTrackingState();

                if (tracking == TrackingState.TRACKING) {
                    Pose p = camera.getPose();
                    // Full six degrees of freedom. Keeping only translation, as
                    // the original did, discards exactly what is needed to compare
                    // this against the robot's own heading.
                    latest.set(new TrackedPose(
                            Pose3.of(p.tx(), p.ty(), p.tz(),
                                    p.qx(), p.qy(), p.qz(), p.qw()),
                            TrackedPose.State.TRACKING, "", frames));
                } else {
                    TrackingFailureReason reason = camera.getTrackingFailureReason();
                    // Carry the last good pose forward, but say plainly that it is
                    // no longer current. A consumer that ignores the state will
                    // navigate on a pose that stopped being true some time ago.
                    latest.set(new TrackedPose(latest.get().pose,
                            TrackedPose.State.PAUSED,
                            reason == null ? "" : reason.name(), frames));
                }
            }
        } catch (Throwable t) {
            failure = String.valueOf(t.getMessage());
            Log.e(TAG, "tracker failed", t);
            latest.set(new TrackedPose(latest.get().pose, TrackedPose.State.STOPPED,
                    String.valueOf(failure), frames));
        } finally {
            running = false;
            if (session != null) {
                try {
                    session.pause();
                    session.close();
                } catch (Throwable ignored) {
                    // Shutting down; the session is going away regardless.
                }
            }
            if (display != EGL14.EGL_NO_DISPLAY) {
                EGL14.eglMakeCurrent(display, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_SURFACE,
                        EGL14.EGL_NO_CONTEXT);
                if (surface != EGL14.EGL_NO_SURFACE) {
                    EGL14.eglDestroySurface(display, surface);
                }
                if (eglContext != EGL14.EGL_NO_CONTEXT) {
                    EGL14.eglDestroyContext(display, eglContext);
                }
                EGL14.eglTerminate(display);
            }
            Log.i(TAG, "ARCore session ended after " + frames + " frames");
        }
    }
}
