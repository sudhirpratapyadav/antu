package com.antu.core;

import com.antu.core.geometry.Angles;
import com.antu.core.geometry.Pose2;
import com.antu.core.geometry.Twist2;
import com.antu.core.geometry.Vec2;

/**
 * Geometry, with attention to the places sign and unit errors hide: angle
 * wrapping, transform composition, and integrating a turning twist.
 */
public final class GeometryTest {

    private static final double EPS = 1e-9;

    public static void main(String[] args) {
        Check c = new Check("GeometryTest");

        angleWrapping(c);
        angleDifferenceTakesShortWay(c);
        vectorRotation(c);
        poseTransforms(c);
        poseInverseRoundTrip(c);
        straightLineIntegration(c);
        turningIntegrationFollowsAnArc(c);
        spinInPlaceDoesNotTranslate(c);
        wheelSpeedRoundTrip(c);
        twistLimitPreservesShape(c);

        c.finish();
    }

    private static void angleWrapping(Check c) {
        c.eq("wrap: pi stays pi", true, near(Angles.normalise(Math.PI), Math.PI));
        c.eq("wrap: -pi becomes pi", true, near(Angles.normalise(-Math.PI), Math.PI));
        c.eq("wrap: 3pi/2 becomes -pi/2", true,
                near(Angles.normalise(3 * Math.PI / 2), -Math.PI / 2));
        c.eq("wrap: many turns collapse", true,
                near(Angles.normalise(Angles.TWO_PI * 5 + 0.3), 0.3));
        c.eq("wrap: pose normalises on construction", true,
                near(new Pose2(0, 0, 3 * Math.PI).theta, Math.PI));
    }

    private static void angleDifferenceTakesShortWay(Check c) {
        // 170 to -170 is 20 degrees the short way, not 340 the long way. Getting
        // this wrong makes a robot spin most of a turn to correct a small error.
        double from = Angles.toRadians(170);
        double to = Angles.toRadians(-170);
        c.eq("difference: crosses pi the short way", true,
                near(Angles.difference(from, to), Angles.toRadians(20)));
        c.eq("difference: sign is direction", true, Angles.difference(0, 1) > 0);
        c.eq("distance: always positive", true, Angles.distance(from, to) > 0);
    }

    private static void vectorRotation(Check c) {
        Vec2 v = new Vec2(1, 0).rotated(Math.PI / 2);
        c.eq("rotate: x axis to y axis", true, near(v.x, 0) && near(v.y, 1));
        c.eq("rotate: length preserved", true, near(new Vec2(3, 4).rotated(1.1).length(), 5));
        c.eq("cross: sign gives turn direction", true,
                new Vec2(1, 0).cross(new Vec2(0, 1)) > 0);
    }

    private static void poseTransforms(Check c) {
        // Standing at (1,0) facing 90 degrees; a point 1m ahead is at (1,1).
        Pose2 robot = new Pose2(1, 0, Math.PI / 2);
        Vec2 world = robot.apply(new Vec2(1, 0));
        c.eq("apply: forward is the pose's heading", true, near(world.x, 1) && near(world.y, 1));

        Pose2 composed = robot.compose(new Pose2(1, 0, 0));
        c.eq("compose: position", true, near(composed.x, 1) && near(composed.y, 1));
        c.eq("compose: heading carries", true, near(composed.theta, Math.PI / 2));
    }

    private static void poseInverseRoundTrip(Check c) {
        Pose2 a = new Pose2(2.5, -1.25, 0.7);
        Pose2 b = new Pose2(-0.5, 3.0, -2.1);
        // If rel is b seen from a, then walking that rel from a lands on b.
        Pose2 rel = b.relativeTo(a);
        c.eq("inverse: a then (b relative to a) is b", true, samePose(a.compose(rel), b));

        // And a pose relative to itself is the origin.
        c.eq("inverse: a relative to itself is the origin", true,
                samePose(a.relativeTo(a), Pose2.ORIGIN));

        Pose2 identity = a.compose(a.inverse());
        c.eq("inverse: composing with itself is the origin", true,
                samePose(identity, Pose2.ORIGIN));
    }

    private static void straightLineIntegration(Check c) {
        Pose2 p = Pose2.ORIGIN.integrate(Twist2.forward(1.0), 2.0);
        c.eq("integrate: 1 m/s for 2 s is 2 m", true, near(p.x, 2) && near(p.y, 0));
        c.eq("integrate: heading unchanged", true, near(p.theta, 0));
    }

    /**
     * The case the straight-line approximation gets wrong. Driving 1 m/s while
     * turning 1 rad/s for a quarter turn traces an arc of radius 1: the robot ends
     * at (1, 1) minus the arc's curvature, not at the chord's midpoint.
     */
    private static void turningIntegrationFollowsAnArc(Check c) {
        Twist2 curve = Twist2.of(1.0, 1.0);          // radius = v/w = 1 m
        Pose2 p = Pose2.ORIGIN.integrate(curve, Math.PI / 2);

        // A quarter circle of radius 1 starting at the origin heading +x ends at
        // (1, 1) facing +y.
        c.eq("arc: quarter turn ends on the circle", true, near(p.x, 1) && near(p.y, 1));
        c.eq("arc: heading advanced a quarter turn", true, near(p.theta, Math.PI / 2));

        // Many small steps must agree with one exact step, or odometry drifts with
        // tick rate — a bug that looks like a hardware fault.
        Pose2 stepped = Pose2.ORIGIN;
        int steps = 2000;
        for (int i = 0; i < steps; i++) {
            stepped = stepped.integrate(curve, (Math.PI / 2) / steps);
        }
        c.eq("arc: fine steps agree with one exact step", true,
                Math.abs(stepped.x - p.x) < 1e-6 && Math.abs(stepped.y - p.y) < 1e-6);
    }

    private static void spinInPlaceDoesNotTranslate(Check c) {
        Pose2 p = Pose2.ORIGIN.integrate(Twist2.turn(1.0), 1.0);
        c.eq("spin: stays put", true, near(p.x, 0) && near(p.y, 0));
        c.eq("spin: heading advanced", true, near(p.theta, 1.0));
    }

    private static void wheelSpeedRoundTrip(Check c) {
        double wheelbase = 0.33;                      // a P3-DX, in metres
        Twist2 t = Twist2.of(0.4, 0.6);
        double[] wheels = t.toWheelSpeeds(wheelbase);
        Twist2 back = Twist2.fromWheelSpeeds(wheels[0], wheels[1], wheelbase);

        c.eq("wheels: linear survives", true, near(back.linearX, t.linearX));
        c.eq("wheels: angular survives", true, near(back.angular, t.angular));
        c.eq("wheels: turning left means right wheel faster", true, wheels[1] > wheels[0]);

        // The P3-DX numbers measured on hardware: +-86 mm/s over a 330 mm wheelbase
        // came out as 30 deg/s, which is what its RVEL was commanded to do.
        Twist2 measured = Twist2.fromWheelSpeeds(-0.086, 0.086, 0.33);
        c.eq("wheels: matches the real P3-DX measurement", true,
                Math.abs(Angles.toDegrees(measured.angular) - 30) < 2);
    }

    private static void twistLimitPreservesShape(Check c) {
        Twist2 fast = Twist2.of(2.0, 2.0);
        Twist2 limited = fast.limited(1.0, 10.0);

        c.eq("limit: linear clamped", true, near(limited.linearX, 1.0));
        // Both scale together, so the arc the robot follows is unchanged. Clipping
        // each independently would tighten the curve near the speed limit.
        c.eq("limit: ratio preserved", true,
                near(limited.angular / limited.linearX, fast.angular / fast.linearX));
        c.eq("limit: under the limits is untouched", true,
                Twist2.of(0.1, 0.1).limited(1, 1).equals(Twist2.of(0.1, 0.1)));
    }

    private static boolean near(double a, double b) {
        return Math.abs(a - b) < 1e-6;
    }

    private static boolean samePose(Pose2 a, Pose2 b) {
        return near(a.x, b.x) && near(a.y, b.y) && near(a.theta, b.theta);
    }
}
