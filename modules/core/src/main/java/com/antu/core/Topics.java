package com.antu.core;

import com.antu.core.bus.Topic;
import com.antu.core.geometry.Pose2;
import com.antu.core.geometry.Twist2;
import com.antu.core.msg.BaseStatus;
import com.antu.core.msg.ImuSample;
import com.antu.core.msg.Odometry;
import com.antu.core.msg.RangeScan;

/**
 * The standard topics, declared once.
 *
 * <p>Nodes refer to these constants rather than spelling out names, so a typo is
 * a compile error and the payload type is fixed in one place. A driver and a
 * planner disagreeing about whether {@code /cmd_vel} carries m/s or mm/s is the
 * kind of bug that only shows up when a robot lurches, and this is where it gets
 * prevented.
 *
 * <p>Nodes are free to declare private topics of their own; this is the shared
 * vocabulary, not a restriction.
 */
public final class Topics {

    private Topics() { }

    // ---------- commands, flowing down to the base ----------

    /**
     * The velocity the base should hold, in the robot frame.
     *
     * <p>One publisher at a time. Two nodes both writing here is the classic way
     * to get a robot that stutters between a teleop command and a planner's, and
     * arbitration belongs in a node that decides, not in the bus.
     */
    public static final Topic<Twist2> CMD_VEL = Topic.of("/cmd_vel", Twist2.class);

    // ---------- state, flowing up from the base ----------

    /** Wheel odometry: continuous and smooth, but drifts. */
    public static final Topic<Odometry> ODOM = Topic.of("/odom", Odometry.class);

    /** Best estimate of where the robot is in the map frame. */
    public static final Topic<Pose2> POSE = Topic.of("/pose", Pose2.class);

    /** Battery, motor and e-stop state from the drive base. */
    public static final Topic<BaseStatus> BASE_STATUS =
            Topic.of("/base/status", BaseStatus.class);

    // ---------- sensors ----------

    /** The phone's inertial sensors. */
    public static final Topic<ImuSample> IMU_PHONE = Topic.of("/imu/phone", ImuSample.class);

    /**
     * The gyro inside the drive base.
     *
     * <p>Deliberately a separate topic from {@link #IMU_PHONE}. The two sit on
     * different bodies with different mounting and noise, and on a P3-DX the base
     * gyro is what the firmware's own heading comes from. Fusing them is a choice
     * a node makes, not something that happens by two drivers sharing a name.
     */
    public static final Topic<ImuSample> IMU_BASE = Topic.of("/imu/base", ImuSample.class);

    /** Sonar ring, and later any lidar. */
    public static final Topic<RangeScan> RANGES = Topic.of("/ranges", RangeScan.class);

    // ---------- goals ----------

    /** Where the robot has been asked to go, in the map frame. */
    public static final Topic<Pose2> GOAL = Topic.of("/goal", Pose2.class);
}
