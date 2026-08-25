package com.antu.core.msg;

import com.antu.core.geometry.Pose3;

/**
 * A camera frame together with where the camera was when it was taken, and the
 * intrinsics needed to make geometric sense of it. Immutable.
 *
 * <p>The three travel as one message on purpose. Publishing the image and the
 * pose separately and matching them by timestamp afterwards is the arrangement
 * every reconstruction pipeline regrets: the two rates differ, the correlation
 * window is a guess, and a frame paired with the wrong pose does not look wrong —
 * it puts a wall slightly in the wrong place, over and over, until the map is
 * subtly bent and nothing says why.
 *
 * <p>The tracker knows the pose for a frame exactly, because it produced both
 * from the same update. Keeping them together preserves that certainty instead
 * of discarding it and approximating it back.
 */
public final class PosedFrame {

    /** The picture. Its bytes are shared and must not be modified. */
    public final VideoFrame image;

    /**
     * Camera pose in the tracker's world frame.
     *
     * <p>This is where the <em>camera</em> was, not the robot. Converting to the
     * robot's frame needs the mount offset from calibration.
     */
    public final Pose3 cameraPose;

    /** Horizontal focal length, pixels. */
    public final double fx;
    /** Vertical focal length, pixels. */
    public final double fy;
    /** Principal point x, pixels. */
    public final double cx;
    /** Principal point y, pixels. */
    public final double cy;

    public PosedFrame(VideoFrame image, Pose3 cameraPose,
                      double fx, double fy, double cx, double cy) {
        this.image = image;
        this.cameraPose = cameraPose;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
    }

    /**
     * The direction a pixel looks along, in the camera's own frame.
     *
     * <p>Unnormalised and pointing down -z, which is the camera convention ARCore
     * uses: multiply by a depth in metres and the result is a point in the
     * camera's frame, ready to be put into the world by {@link #cameraPose}.
     */
    public double[] rayThrough(double pixelX, double pixelY) {
        return new double[] {
            (pixelX - cx) / fx,
            -(pixelY - cy) / fy,     // image y runs down, camera y runs up
            -1.0
        };
    }

    @Override public String toString() {
        return "PosedFrame{" + image + " at " + cameraPose.position + "}";
    }
}
