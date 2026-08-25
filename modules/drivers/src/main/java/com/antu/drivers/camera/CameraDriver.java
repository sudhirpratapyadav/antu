package com.antu.drivers.camera;

import android.content.pm.PackageManager;
import android.graphics.ImageFormat;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.os.HandlerThread;
import android.view.Surface;

import com.antu.core.graph.Out;
import com.antu.core.log.Log;
import com.antu.core.msg.VideoFrame;
import com.antu.core.node.Node;
import com.antu.core.time.Stamp;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * The phone's rear camera, as JPEG frames on a channel.
 *
 * <p>Capture runs on the camera's own thread and publishes; the graph's tick loop
 * never touches it. Frames are encoded to JPEG at capture time rather than
 * carried as raw pixels — 640x480 is about 900 KB raw and 40 KB compressed, which
 * is the difference between video being usable over Wi-Fi and not.
 *
 * <h2>Why YUV and not the JPEG capture format</h2>
 *
 * <p>Camera2 can produce {@code ImageFormat.JPEG} directly, but that goes through
 * the still-capture pipeline: high quality, and far too slow for a repeating
 * request. Capturing {@code YUV_420_888} and compressing with {@link YuvImage}
 * keeps the preview pipeline's frame rate.
 *
 * <h2>Backpressure</h2>
 *
 * <p>Only the newest frame matters. If encoding cannot keep up, incoming images
 * are dropped rather than queued: a robot operator wants to see now, and a
 * backlog of stale frames is worse than a lower frame rate.
 */
public final class CameraDriver extends Node {

    private static final String TAG = "camera";
    /** JPEG quality. 60 is visibly fine for a robot view and roughly halves size. */
    private static final int QUALITY = 60;
    /** Two images in flight: one being encoded, one arriving. */
    private static final int IMAGE_BUFFERS = 2;

    /** JPEG frames from the camera. */
    public final Out<VideoFrame> frame = out("frame", VideoFrame.class);

    private final android.content.Context context;
    private final int targetWidth;
    private final int targetHeight;
    private final boolean useFrontCamera;

    private HandlerThread thread;
    private Handler handler;
    private CameraManager manager;
    private CameraDevice device;
    private CameraCaptureSession session;
    private ImageReader reader;

    /** Latest encoded frame, handed to the tick to publish. */
    private final AtomicReference<VideoFrame> pending = new AtomicReference<>();
    private volatile long captured;
    private volatile long dropped;
    private volatile String failure;

    public CameraDriver(android.content.Context context) {
        this(context, 640, 480, false);
    }

    public CameraDriver(android.content.Context context, int width, int height,
                        boolean useFrontCamera) {
        super("camera");
        this.context = context.getApplicationContext();
        this.targetWidth = width;
        this.targetHeight = height;
        this.useFrontCamera = useFrontCamera;
    }

    /** Frames captured since start. */
    public long captured() {
        return captured;
    }

    /** Frames discarded because encoding could not keep up. */
    public long dropped() {
        return dropped;
    }

    /** Why the camera is not running, or null when it is. */
    public String failure() {
        return failure;
    }

    @Override public void start(Node.Context ctx) {
        if (context.checkSelfPermission(android.Manifest.permission.CAMERA)
                != PackageManager.PERMISSION_GRANTED) {
            // Not fatal: the rest of the robot must keep working without video.
            failure = "camera permission not granted";
            Log.w(TAG, failure, null);
            return;
        }
        manager = (CameraManager) context.getSystemService(android.content.Context.CAMERA_SERVICE);
        if (manager == null) {
            failure = "no camera service";
            return;
        }

        thread = new HandlerThread("antu-camera");
        thread.start();
        handler = new Handler(thread.getLooper());

        try {
            String id = chooseCamera();
            if (id == null) {
                failure = "no suitable camera";
                Log.w(TAG, failure, null);
                return;
            }
            reader = ImageReader.newInstance(targetWidth, targetHeight,
                    ImageFormat.YUV_420_888, IMAGE_BUFFERS);
            reader.setOnImageAvailableListener(this::onImage, handler);
            Log.i(TAG, "opening camera " + id + " at " + targetWidth + "x" + targetHeight);
            manager.openCamera(id, new DeviceCallback(), handler);
            // openCamera is asynchronous. If neither callback below ever fires,
            // Android has refused silently — almost always because the app is in
            // the background, which it forbids for the camera from API 30.
            failure = "waiting for the camera to open";
        } catch (CameraAccessException | SecurityException | IllegalArgumentException e) {
            failure = "camera open failed: " + e.getMessage();
            Log.e(TAG, failure, e);
        }
    }

    @Override public void tick(Node.Context ctx) {
        // Published from the tick rather than the camera callback so the channel's
        // rate is the node's declared rate, not whatever the sensor happens to do.
        VideoFrame f = pending.getAndSet(null);
        if (f != null) {
            frame.publish(f);
        }
    }

    @Override public void stop() {
        CameraCaptureSession s = session;
        session = null;
        if (s != null) {
            s.close();
        }
        CameraDevice d = device;
        device = null;
        if (d != null) {
            d.close();
        }
        ImageReader r = reader;
        reader = null;
        if (r != null) {
            r.close();
        }
        if (thread != null) {
            thread.quitSafely();
            thread = null;
        }
    }

    private String chooseCamera() throws CameraAccessException {
        int wanted = useFrontCamera
                ? CameraCharacteristics.LENS_FACING_FRONT
                : CameraCharacteristics.LENS_FACING_BACK;
        for (String id : manager.getCameraIdList()) {
            Integer facing = manager.getCameraCharacteristics(id)
                    .get(CameraCharacteristics.LENS_FACING);
            if (facing != null && facing == wanted) {
                return id;
            }
        }
        // Better a camera facing the wrong way than none at all.
        String[] all = manager.getCameraIdList();
        return all.length > 0 ? all[0] : null;
    }

    private void onImage(ImageReader from) {
        Image image = from.acquireLatestImage();
        if (image == null) {
            return;
        }
        try {
            // A frame still waiting to be published is now stale; replacing it is
            // the whole backpressure policy.
            if (pending.get() != null) {
                dropped++;
            }
            byte[] jpeg = JpegEncoder.encode(image, QUALITY);
            if (jpeg != null) {
                captured++;
                pending.set(new VideoFrame(jpeg, image.getWidth(), image.getHeight(), captured));
            }
        } catch (RuntimeException e) {
            Log.w(TAG, "frame encode failed: " + e, null);
        } finally {
            // Always close, or the reader runs out of buffers and the stream stops
            // dead with no error anywhere.
            image.close();
        }
    }

    private final class DeviceCallback extends CameraDevice.StateCallback {

        @Override public void onOpened(CameraDevice camera) {
            Log.i(TAG, "camera opened");
            device = camera;
            try {
                List<Surface> surfaces = new ArrayList<>(
                        Collections.singletonList(reader.getSurface()));
                camera.createCaptureSession(surfaces, new SessionCallback(), handler);
            } catch (CameraAccessException e) {
                failure = "capture session failed: " + e.getMessage();
                Log.e(TAG, failure, e);
            }
        }

        @Override public void onDisconnected(CameraDevice camera) {
            failure = "camera disconnected";
            Log.w(TAG, failure, null);
            camera.close();
            device = null;
        }

        @Override public void onError(CameraDevice camera, int error) {
            failure = "camera error " + error;
            Log.e(TAG, failure, null);
            camera.close();
            device = null;
        }
    }

    private final class SessionCallback extends CameraCaptureSession.StateCallback {

        @Override public void onConfigured(CameraCaptureSession configured) {
            session = configured;
            try {
                CaptureRequest.Builder request =
                        device.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
                request.addTarget(reader.getSurface());
                // Continuous autofocus: a robot's view keeps changing, and a fixed
                // focus is blurred more often than not.
                request.set(CaptureRequest.CONTROL_AF_MODE,
                        CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE);
                configured.setRepeatingRequest(request.build(), null, handler);
                failure = null;
                Log.i(TAG, "camera streaming at " + targetWidth + "x" + targetHeight);
            } catch (CameraAccessException | IllegalStateException e) {
                failure = "repeating request failed: " + e.getMessage();
                Log.e(TAG, failure, e);
            }
        }

        @Override public void onConfigureFailed(CameraCaptureSession configured) {
            failure = "camera could not be configured";
            Log.e(TAG, failure, null);
        }
    }
}
