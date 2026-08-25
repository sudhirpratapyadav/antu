package com.antu.drivers.camera;

import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.media.Image;

import java.io.ByteArrayOutputStream;
import java.nio.ByteBuffer;

/**
 * Camera images to JPEG.
 *
 * <p>Shared by the plain camera driver and the AR tracker, which both receive
 * {@code YUV_420_888} from different sources and need the same thing done to it.
 * Having two copies of the plane arithmetic below is asking for one of them to
 * quietly acquire a fix the other lacks.
 *
 * <p>Camera2 can produce {@code ImageFormat.JPEG} directly, but that runs the
 * still-capture pipeline: high quality, and far too slow for a repeating request.
 * Compressing YUV keeps preview frame rate.
 */
public final class JpegEncoder {

    /** Visibly fine for a robot view, and roughly half the bytes of quality 90. */
    public static final int DEFAULT_QUALITY = 60;

    private JpegEncoder() { }

    /**
     * Compresses a {@code YUV_420_888} image.
     *
     * @return a fresh array each call. Readers hold frames by reference, so a
     *         reused buffer would be rewritten under them mid-send.
     */
    public static byte[] encode(Image image, int quality) {
        byte[] nv21 = toNv21(image);
        if (nv21 == null) {
            return null;
        }
        YuvImage yuv = new YuvImage(nv21, ImageFormat.NV21,
                image.getWidth(), image.getHeight(), null);
        ByteArrayOutputStream out = new ByteArrayOutputStream(64 * 1024);
        yuv.compressToJpeg(new Rect(0, 0, image.getWidth(), image.getHeight()), quality, out);
        return out.toByteArray();
    }

    public static byte[] encode(Image image) {
        return encode(image, DEFAULT_QUALITY);
    }

    /**
     * Packs the planar YUV an Android camera produces into NV21.
     *
     * <p>The chroma planes arrive interleaved or planar depending on the device,
     * and with a row stride that is not always the width. Ignoring either gives a
     * picture with a green skew, which is the classic symptom of getting this
     * wrong.
     */
    private static byte[] toNv21(Image image) {
        Image.Plane[] planes = image.getPlanes();
        if (planes.length < 3) {
            return null;
        }
        int width = image.getWidth();
        int height = image.getHeight();
        byte[] out = new byte[width * height * 3 / 2];

        copyPlane(planes[0], width, height, out, 0, 1);

        // NV21 is V then U, interleaved, at half resolution.
        int chromaOffset = width * height;
        copyPlane(planes[2], width / 2, height / 2, out, chromaOffset, 2);
        copyPlane(planes[1], width / 2, height / 2, out, chromaOffset + 1, 2);
        return out;
    }

    private static void copyPlane(Image.Plane plane, int width, int height,
                                  byte[] out, int offset, int outStride) {
        ByteBuffer buffer = plane.getBuffer();
        int rowStride = plane.getRowStride();
        int pixelStride = plane.getPixelStride();
        byte[] row = new byte[rowStride];
        int pos = offset;

        for (int y = 0; y < height; y++) {
            int available = Math.min(rowStride, buffer.remaining());
            buffer.get(row, 0, available);
            for (int x = 0; x < width; x++) {
                int from = x * pixelStride;
                if (from >= available) {
                    break;
                }
                out[pos] = row[from];
                pos += outStride;
            }
        }
    }
}
