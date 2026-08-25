package com.antu.ops;

import com.antu.core.Check;
import com.antu.core.geometry.Pose2;
import com.antu.core.geometry.Twist2;
import com.antu.core.msg.Odometry;
import com.antu.core.msg.RangeScan;

/** Encoding for the ops API: compact, complete, and free of recursion. */
public final class JsonTest {

    public static void main(String[] args) {
        Check c = new Check("JsonTest");
        MessageEncoders.installDefaults();

        primitives(c);
        poseStaysFlat(c);
        odometryNests(c);
        rangeScanIsComplete(c);
        nonFiniteNumbersSurvive(c);
        unencodableTypeIsObvious(c);

        c.finish();
    }

    private static void primitives(Check c) {
        c.eq("null", "null", Json.encode(null));
        c.eq("boolean", "true", Json.encode(true));
        c.eq("integer", "42", Json.encode(42));
        c.eq("whole double reads as an integer", "3", Json.encode(3.0));
        c.eq("fraction is trimmed", "0.485", Json.encode(0.485));
        c.eq("string is escaped", "\"a\\\"b\\nc\"", Json.encode("a\"b\nc"));
        c.eq("array", "[1,2,3]", Json.encode(new int[] {1, 2, 3}));
    }

    /**
     * The regression that motivated the registry: reflecting over accessors made
     * this recurse through Vec2.normalised() and produced kilobytes of nesting for
     * a three-number pose.
     */
    private static void poseStaysFlat(Check c) {
        String json = Json.encode(new Pose2(1.5, -2.25, 0.5));
        c.eq("pose is exactly its fields", "{\"x\":1.5,\"y\":-2.25,\"theta\":0.5}", json);
        c.eq("pose stays small", true, json.length() < 60);
    }

    private static void odometryNests(Check c) {
        String json = Json.encode(new Odometry(new Pose2(1, 2, 0), Twist2.of(0.5, 0.25)));
        c.eq("odometry nests both parts", true,
                json.contains("\"pose\":{") && json.contains("\"velocity\":{"));
        c.eq("odometry stays small", true, json.length() < 160);
    }

    /** The case field reflection cannot reach, and the reason the registry exists. */
    private static void rangeScanIsComplete(Check c) {
        RangeScan scan = new RangeScan(
                new double[] {-0.5, 0, 0.5},
                new double[] {1.25, RangeScan.NO_RETURN, 0.05},
                0.1, 4.5);
        String json = Json.encode(scan);

        c.eq("scan reports its ranges", true, json.contains("\"ranges\":[1.25,"));
        c.eq("scan reports its bearings", true, json.contains("\"bearings\":[-0.5,"));
        // A client must not have to re-derive the min/max test and get it wrong:
        // 1.25 is real, the no-return is not, and 0.05 is below the trusted band.
        c.eq("scan marks validity", true, json.contains("\"valid\":[true,false,false]"));
        c.eq("scan reports the closest valid reading", true, json.contains("\"closest\":1.25"));
    }

    private static void nonFiniteNumbersSurvive(Check c) {
        // JSON has no infinity, and RangeScan uses it for "no echo". It has to
        // arrive as something a client can test rather than as a parse error.
        c.eq("positive infinity", "\"Infinity\"", Json.encode(Double.POSITIVE_INFINITY));
        c.eq("negative infinity", "\"-Infinity\"", Json.encode(Double.NEGATIVE_INFINITY));
        c.eq("not a number", "\"NaN\"", Json.encode(Double.NaN));
    }

    private static void unencodableTypeIsObvious(Check c) {
        // No public fields and no registered encoder: report toString rather than
        // an empty object, so a missing encoder is visible instead of silent.
        String json = Json.encode(new Object() {
            @Override public String toString() {
                return "opaque";
            }
        });
        c.eq("missing encoder is visible", true, json.contains("opaque"));
    }
}
