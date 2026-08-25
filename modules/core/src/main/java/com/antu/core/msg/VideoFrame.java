package com.antu.core.msg;

/**
 * One encoded camera frame.
 *
 * <p>JPEG rather than raw pixels. A 640x480 frame is about 900 KB as RGBA and
 * about 40 KB compressed, and on a phone driving a robot over Wi-Fi the
 * difference decides whether video is usable at all. It also means a browser can
 * display it directly with no decoder.
 *
 * <h2>The bytes are shared, so nobody may write to them</h2>
 *
 * <p>Like every payload, this is handed to readers by reference and never copied.
 * Arrays cannot be made immutable in Java, so the rule is a convention the
 * producer must keep: encode into a fresh array per frame and never reuse a
 * buffer. A camera driver recycling one buffer would corrupt a frame mid-encode
 * for every reader at once.
 *
 * <h2>Not JSON</h2>
 *
 * <p>This must never reach the generic JSON encoder — base64 of a megabyte per
 * frame would flood the telemetry socket and is useless to look at. The ops layer
 * registers a metadata-only encoder for it and serves the pixels over a separate
 * binary route.
 */
public final class VideoFrame {

    /** JPEG-encoded image data. Treat as read-only. */
    private final byte[] jpeg;

    public final int width;
    public final int height;
    /** Frame counter from the camera, for spotting drops. */
    public final long index;

    public VideoFrame(byte[] jpeg, int width, int height, long index) {
        this.jpeg = jpeg;
        this.width = width;
        this.height = height;
        this.index = index;
    }

    /**
     * The encoded bytes, not copied.
     *
     * <p>Copying per reader would defeat the point on a 30 fps stream. Do not
     * modify what this returns.
     */
    public byte[] jpeg() {
        return jpeg;
    }

    public int sizeBytes() {
        return jpeg.length;
    }

    @Override public String toString() {
        return "VideoFrame{" + width + "x" + height + " #" + index
                + " " + (jpeg.length / 1024) + "KB}";
    }
}
