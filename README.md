# antu

A phone as the brain of a mobile robot. Drivers, core and operations in one
process, with no ROS.

## Why not ROS

The value of a phone is its hardware: Camera2, ARCore, the IMU, the GPU, audio.
Those are Android APIs. ROS cannot reach them without a bridge, so a ROS-on-phone
design ends up writing that bridge anyway and pays for a large dependency to do
it. rosjava has been unmaintained since around 2019, and ROS 2 on Android means a
painful cross-compile or a chroot that cannot see the camera.

The good ideas in ROS are worth keeping, and they are cheap to have: a typed
pub/sub bus, a transform tree, recording and replay, and introspection. The
expensive parts — DDS, IPC, serialisation on every hop — are implementation, not
architecture. This bus hands payloads between subscribers by reference and
serialises only at the edges, in the recorder and the bridge.

## Layout

```
modules/core      bus, clock, nodes, scheduler        no Android
modules/brain     control, mapping, navigation        no Android
modules/drivers   base, IMU, camera, audio            Android
modules/ops       server, bridge, web UI              Android
modules/app       service and developer console       Android
```

**`core` and `brain` are compiled without `android.jar` on the classpath.** An
`import android.*` there is a compile error rather than something to catch in
review. That is what keeps the planner runnable on a laptop and replayable from a
recording, which is the difference between a robot bug you can reproduce and one
you cannot.

## Build

```bash
./build.sh          # pure modules, tests, then the APK
./build.sh test     # tests only, no SDK needed
./build.sh core     # antu-core.jar for desktop use
```

Needs a JDK and the SDK build tools (`aapt2`, `android.jar`, `zipalign`,
`lib/{d8,apksigner}.jar`). Point `SDK=` at them if they are not in
`~/android-sdk/android-11`. No Gradle.

## The scheduler, and why it looks slow

One thread ticks every node in the order it was added, at the fastest declared
rate; each node runs when its own period has elapsed. A 200 Hz filter and a 5 Hz
planner share that thread with no coordination.

Subscriptions are queued by default and drained immediately before the owning
node's tick. Publishers never block, and because draining happens in a declared
order, **the same inputs produce the same execution every run**. Under a
`ManualClock` a graph is fully determined by its inputs, so an intermittent bug
reproduces exactly instead of one time in twenty. `Bus.Delivery.DIRECT` opts out
for a control loop that must react to a sample immediately, at the cost of that
guarantee.

Declaration order is worth a cycle of latency: a node added after its producer
sees that producer's message on the same tick, not the next one.

Nodes that must block — a serial read, a camera frame — own a thread and hand
results over through the bus. The tick loop never blocks.

## Status

Phase 1. Core primitives and the build, with a placeholder node so there is
something running on the device. 40 tests.

Next: the arcos base driver and phone IMU, then the server and web UI.

## Related

- [arcos-android](https://github.com/sudhirpratapyadav/arcos-android) — the
  Pioneer base driver this uses, verified against a real P3-DX.
