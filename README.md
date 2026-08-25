# antu

A phone as the brain of a mobile robot: drivers, control and operations in one
process, with no ROS.

Runs on an Android phone bolted to a Pioneer 3-DX. The phone drives the base over
USB, reads its own camera and inertial sensors, and serves a console that anyone
on the network can open to watch and drive.

## Why not ROS

The value of a phone is its hardware: Camera2, ARCore, the IMU, the GPU, audio.
Those are Android APIs. ROS cannot reach them without a bridge, so a ROS-on-phone
design ends up writing that bridge anyway and pays for a large dependency to do
it. rosjava has been unmaintained since around 2019, and ROS 2 on Android means a
painful cross-compile or a chroot that cannot see the camera.

The good ideas in ROS are worth keeping and cheap to have directly: typed
channels, recording, introspection. The expensive parts — DDS, IPC, serialisation
on every hop — are implementation, not architecture. Channels here hand payloads
between nodes by reference and serialise only at the edges.

## The graph is static

Nodes declare typed `In`/`Out` ports and are wired at build time.

```java
Graph g = Graph.builder(Clock.SYSTEM)
        .add(base, Rate.hz(10))
        .add(ops, Rate.hz(20))
        .add(arbiter, Rate.hz(15))
        .connect(ops.cmdVel, arbiter.teleop)
        .connect(arbiter.cmdVel, base.cmdVel)
        .build();
```

`connect` is generic, so mismatched types do not compile. `build()` then checks
what a type cannot express, and reports every problem at once:

- a **missing connection** — better than a node that silently never receives
  anything and looks like a broken sensor
- **two writers on one input** — a robot taking velocity from two places stutters
  between them, and this is caught before it moves
- a **cycle** of direct edges, naming the loop. Control loops are cyclic, so mark
  the feedback edge `connectDelayed()` and the consumer reads the previous tick

Execution order is a **topological sort of the dataflow**, so a producer always
runs before its consumer and data crosses the whole graph in one tick. Nothing
depends on the order nodes were added.

This replaced a runtime pub/sub bus. Three bugs in a row — subscribing before a
publisher existed, a type check rejecting the wildcard subscription introspection
needs, and a catalogue changing under a connected client — were the same bug in
different clothes. They are now unrepresentable rather than fixed.

## Layout

```
modules/core      channels, clock, nodes, scheduler, geometry, messages   no Android
modules/brain     arbitration, and control to come                        no Android
modules/drivers   Pioneer base, phone IMU, camera                         Android
modules/ops       HTTP, WebSocket bridge, web console                     Android-free
modules/app       service, developer console, graph wiring                Android
```

**`core`, `brain` and `ops` compile without `android.jar` on the classpath.** An
`import android.*` there is a compile error rather than something to catch in
review, which is what keeps them runnable on a laptop and replayable from a
recording.

## Build

```bash
./build.sh          # pure modules, tests, then the APK
./build.sh test     # tests only, no SDK needed
./build.sh core     # antu-core.jar for desktop use
```

Needs a JDK and the SDK build tools (`aapt2`, `android.jar`, `zipalign`,
`lib/{d8,apksigner}.jar`). Point `SDK=` at them if they are not in
`~/android-sdk/android-11`. No Gradle.

## Operations

The console is at `http://<phone>:8080/` — video, sonar minimap, telemetry, a
thumb-stick, and arrow-key or WASD driving. It subscribes to channels by name and
renders whatever arrives, so adding a node to the graph makes its channel appear
with no change to the page.

```bash
curl phone:8080/api/nodes                  # node table, rates, tick counts
curl phone:8080/api/channels               # every channel, fixed at build time
curl "phone:8080/api/channel?name=base.odom"
curl "phone:8080/api/drive?mm=200&deg=0"
curl phone:8080/video.mjpeg                # the camera, in any browser
```

`tools/ws-client.py` speaks the WebSocket bridge from a laptop with no
dependencies. `tools/setup-wifi-adb.sh` moves adb to Wi-Fi while the phone is
still on USB, since the port is about to hold a robot.

## Safety

Three independent stops, because one is not enough on something that moves:

- **The robot's own watchdog** cuts the motors after two seconds of silence.
- **The base driver** stops if `cmd_vel` goes quiet, which the robot's watchdog
  cannot see because the driver keeps pulsing regardless.
- **The console** commands zero when a finger lifts, a key is released, the tab
  is hidden, or the window loses focus — a key held while alt-tabbing never fires
  `keyup`, and the robot would hold that command.

The arbiter is the last gate before the base and clamps whatever passes through,
whoever sent it, so a planner with a unit bug cannot ask for 40 m/s.

## Units

`core` and `brain` are SI throughout: metres, radians, seconds. ARCOS speaks
millimetres and degrees, and the driver is the single place that converts. Mixed
units inside a planner is a bug factory and cannot be retrofitted once nodes
depend on it.

## Status

Working on a real Pioneer 3-DX (`p3dx-sh`) over a CH340 adapter at 38400 baud:
drive base, both IMUs, camera at 640x480, teleop from a browser, and arbitration
between teleop and autonomy.

`brain` is where this goes next — fusion, then obstacle sensing from the camera,
then navigation. Note the sonar ring on our robot is deaf: all eight transducers
report no-echo with a wall a metre away, so obstacle sensing has to come from the
camera.

97 tests, none needing hardware.

## Related

- [arcos-android](https://github.com/sudhirpratapyadav/arcos-android) — the
  Pioneer base driver, a standalone library verified against the same robot
- [dimensionalOS/dimos](https://github.com/dimensionalOS/dimos) — a much larger
  take on the same idea, and the source of the arbitration pattern here
